//Bootloader_stm32f103
//Andrei Kamiak
//13.02.2019

#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "system_stm32f10x.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_can.h"
#include "misc.h"

#define MAIN_PROGRAM_START_ADDRESS      ((uint32_t)0x08008000)

#define SMALL_TIMEOUT           ((uint32_t)0x200000)
#define BIG_TIMEOUT             ((uint32_t)0x800000)

uint32_t timeout = 0;
uint8_t fNewPO = 0;
uint8_t fNewData = 0;
uint8_t fEndData = 0;
uint32_t cDataMess = 0;
uint8_t dataBuffer[8];
uint8_t dataBuffer2[8];
uint32_t currentAddress;
uint32_t sizeFile;


typedef  void (*pFunction)(void);
uint32_t jumpAddress;
pFunction Jump_To_Application;


CanTxMsg TxMessage;
CanRxMsg RxMessage;


void InitCan1(void);
void CAN1_RX0_IRQHandler(void);
void sendMes(void);


void main()
{
    InitCan1();
    
    //Шлем сообщение о запуске загрузчика БВВ STM32
    TxMessage.StdId = 0x700;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.DLC = 8;
    TxMessage.Data[0] = 0x00;
    TxMessage.Data[1] = 0x00;
    TxMessage.Data[2] = 0x01;
    TxMessage.Data[3] = 0x00;
    TxMessage.Data[4] = 0x10;
    TxMessage.Data[5] = 0xFF;
    TxMessage.Data[6] = 0xFF;
    TxMessage.Data[7] = 0xFF;
    sendMes();

    timeout = BIG_TIMEOUT;
    while(!fNewPO && timeout) {
        if(!(timeout--)) break;
    }

    if(fNewPO) {
        FLASH_Unlock();
        //Стираем старое ПО
        uint16_t buf = sizeFile*8/1024+2;
        for(uint8_t i = 0; i < buf; i++) {
            FLASH_ErasePage(MAIN_PROGRAM_START_ADDRESS + i * 0x400);
        }
        currentAddress = MAIN_PROGRAM_START_ADDRESS;
        //Прием нового ПО
        while(!fEndData) {
            fNewData = 0;
            TxMessage.StdId = 0x300;
            TxMessage.RTR = CAN_RTR_DATA;
            TxMessage.IDE = CAN_ID_STD;
            TxMessage.DLC = 8;
            *(uint32_t *)(&TxMessage.Data[0]) = cDataMess;
            TxMessage.Data[4] = 0xFF;
            TxMessage.Data[5] = 0xFF;
            TxMessage.Data[6] = 0xFF;
            TxMessage.Data[7] = 0xFF;
            sendMes();

            timeout = SMALL_TIMEOUT;
            while((!fNewData && !fEndData) && timeout) {
                if(!(timeout--)) break;
            }

            if(fNewData && cDataMess < sizeFile ) {
                fNewData = 0;
                if(cDataMess) {
                    FLASH_ProgramWord(currentAddress, *(uint32_t*)&dataBuffer[0]);
                    currentAddress +=4;
                    FLASH_ProgramWord(currentAddress, *(uint32_t*)&dataBuffer[4]);
                    currentAddress +=4;
                } else {
                    dataBuffer2[0]=dataBuffer[0];
                    dataBuffer2[1]=dataBuffer[1];
                    dataBuffer2[2]=dataBuffer[2];
                    dataBuffer2[3]=dataBuffer[3];
                    dataBuffer2[4]=dataBuffer[4];
                    dataBuffer2[5]=dataBuffer[5];
                    dataBuffer2[6]=dataBuffer[6];
                    dataBuffer2[7]=dataBuffer[7];
                    currentAddress +=8;
                }
                cDataMess++;
            } else if(fEndData) {
                //Обработчик завершения прошивки
                currentAddress = MAIN_PROGRAM_START_ADDRESS;
                FLASH_ProgramWord(currentAddress, *(uint32_t*)&dataBuffer2[0]);
                currentAddress +=4;
                FLASH_ProgramWord(currentAddress, *(uint32_t*)&dataBuffer2[4]);
            } else {
                //Обработчик не корректного завершения прошивки
                fEndData = 1;
                TxMessage.StdId = 0x610;
                TxMessage.RTR = CAN_RTR_DATA;
                TxMessage.IDE = CAN_ID_STD;
                TxMessage.DLC = 8;
                TxMessage.Data[0] = 0x03;
                TxMessage.Data[1] = 0xFF;
                TxMessage.Data[2] = 0xFF;
                TxMessage.Data[3] = 0xFF;
                TxMessage.Data[4] = 0xFF;
                TxMessage.Data[5] = 0xFF;
                TxMessage.Data[6] = 0xFF;
                TxMessage.Data[7] = 0xFF;
                sendMes();
            }
        }
        FLASH_Lock();
    }

    if ( (*(__IO uint32_t*)MAIN_PROGRAM_START_ADDRESS)!= 0xFFFFFFFF ) 
    {
        CAN_DeInit(CAN1);
        jumpAddress = *(__IO uint32_t*) (MAIN_PROGRAM_START_ADDRESS + 4);
        Jump_To_Application = (pFunction) jumpAddress;
        __set_MSP(*(__IO uint32_t*) MAIN_PROGRAM_START_ADDRESS);
        Jump_To_Application();
    }
    else 
    {
        for(volatile uint32_t i = 0;i<0x8000000;i++);
        NVIC_SystemReset();
    }
}

void InitCan1(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    
    //can RX pin
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // can TX pin
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    CAN_DeInit(CAN1);
    CAN_InitTypeDef CAN_InitStructure;
 //   CAN_StructInit(&CAN_InitStructure);
 //   GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);
    
    // CAN init 
    CAN_StructInit(&CAN_InitStructure);
    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = DISABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = ENABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
//  CAN_InitStructure.CAN_Mode = CAN_Mode_Silent_LoopBack;  
//  CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack;
    CAN_InitStructure.CAN_SJW  = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1  = CAN_BS1_4tq;
    CAN_InitStructure.CAN_BS2  = CAN_BS2_4tq;
    CAN_InitStructure.CAN_Prescaler = 16;
    while(CAN_Init(CAN1, &CAN_InitStructure) == CAN_InitStatus_Failed);
  //  CAN_Init(CAN1, &CAN_InitStructure);
    
    // CAN filter
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    CAN_FilterInitStructure.CAN_FilterNumber =          0;
    CAN_FilterInitStructure.CAN_FilterMode =            CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale =           CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh =          0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow =       0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh =      0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow =       0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment =  0;
    CAN_FilterInitStructure.CAN_FilterActivation =      ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
    
    // NVIC init
    // CAN FIFO0 message pending interrupt enable
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void CAN1_RX0_IRQHandler(void)
{
    uint32_t RxId;
    uint8_t data0;

    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

    if(RxMessage.IDE == CAN_ID_STD) 
    {
        RxId = RxMessage.StdId >> 8;
        switch(RxId) {
            case 0x6:
                data0 = RxMessage.Data[0];
                switch(data0) {
                    case 1:
                        fNewPO = 1;
                        sizeFile = *(uint32_t *)(&RxMessage.Data[4]);
                    break;
                    case 2:
                    case 3:
                        fEndData = 1;
                    break;
                    default:
                    break;
                }
            break;
            case 0x2: // перестановка байт для защиты от всего
                dataBuffer[1]=RxMessage.Data[0];
                dataBuffer[0]=RxMessage.Data[1];
                dataBuffer[3]=RxMessage.Data[2];
                dataBuffer[2]=RxMessage.Data[3];
                dataBuffer[5]=RxMessage.Data[4];
                dataBuffer[4]=RxMessage.Data[5];
                dataBuffer[7]=RxMessage.Data[6];
                dataBuffer[6]=RxMessage.Data[7];
                fNewData = 1;
            break;
            default:
            break;
        }
    }
}

void sendMes(void)
{
    uint8_t TransmitMailbox = 0;
    uint32_t i = 0;

    TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);
    i = 0;
    while((CAN_TransmitStatus(CAN1, TransmitMailbox)  !=  CANTXOK) && (i  !=  0xFFFF))
    {
        i++;
    }
}
