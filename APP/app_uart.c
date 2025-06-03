/*
* 	app_uart.c
*	����������
*	����ʱ��: 2025.05.14
*	������Ա��WUB 
*	������˾: ��ɽ��ά
*/

#define _UART_DMA_INTERRUPT_C_

/* Files include */
#include <stdio.h>
#include "application.h"
#include "app_uart.h"

// UART_RX_S RxData;		   // �������ݻ�����
// UART_TX_S TxData;		   // �������ݻ�����
// u8 *RXp = (u8 *)&RxData;   // �������ݻ�����ָ��
// u8 *TXp = (u8 *)&TxData;   // �������ݻ�����ָ��
// u8 txLen = 0;			   // �������ݳ���
// u8 rxLen = 0;			   // �������ݳ���
volatile FLAG8 flag8_uart; // ������Ʊ�־λ


/***********************************************
�������ƣ�void Uart_Init(void)
�������ܣ�485�������ų�ʼ��
��ڲ�������
���ڲ�������
��ע��
************************************************/
void UART_Gpio(void)
{

}

/***********************************************
�������ƣ�void protocol_Init(void)
�������ܣ�Э���ʼ������
��ڲ�������
���ڲ�������
��ע��
************************************************/
void protocol_Init(void)
{

}

/***********************************************
�������ƣ�void Send_TX_int(void)
�������ܣ����������ͺ���
��ڲ�������
���ڲ�������
��ע��
************************************************/
void Send_TX_int(void)
{
}

/***********************************************
�������ƣ�void UART_Process(void)
�������ܣ��жϷ�ʽ����Э�����ݣ�
��ڲ�����f_TxBufOK = 1;txlen = 0 //��������������׼���á��ɷ���
���ڲ�������
��ע�����Բ��Է��ͼ�����ܵ���18ms
************************************************/
void UART_Process(void)
{
}

/***********************************************
�������ƣ�UART_Isr
�������ܣ�UART�жϷ���
��ڲ�������
���ڲ�������
��ע��
************************************************/
void UART_Isr(void)
{
}

/***********************************************
�������ƣ�void Uart1_Init(uint32_t Baudrate)
�������ܣ�UART1��ʼ��
��ڲ�������
���ڲ�������
��ע��TX1 PB6,RX1 PB7, AF0
************************************************/
void Uart1_Init(uint32_t Baudrate)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    UART_InitTypeDef UART_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2ENR_UART1, ENABLE);

    UART_StructInit(&UART_InitStruct);
    UART_InitStruct.BaudRate      = Baudrate;
    UART_InitStruct.WordLength    = UART_WordLength_8b;
    UART_InitStruct.StopBits      = UART_StopBits_1;
    UART_InitStruct.Parity        = UART_Parity_No;
    UART_InitStruct.HWFlowControl = UART_HWFlowControl_None;
    UART_InitStruct.Mode          = UART_Mode_Rx | UART_Mode_Tx;
    UART_Init(UART1, &UART_InitStruct);

    UART_DMACmd(UART1, ENABLE);

    RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOB, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_0);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_0);

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_6;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    UART_Cmd(UART1, ENABLE);
}


/***********************************************
�������ƣ�void Uart2_Init(uint32_t Baudrate)
�������ܣ�UART2��ʼ��
��ڲ�������
���ڲ�������
��ע��TX2 PA2,RX2 PA3, AF1
************************************************/
void Uart2_Init(uint32_t Baudrate)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    UART_InitTypeDef UART_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1ENR_UART2, ENABLE);

    UART_StructInit(&UART_InitStruct);
    UART_InitStruct.BaudRate      = Baudrate;
    UART_InitStruct.WordLength    = UART_WordLength_8b;
    UART_InitStruct.StopBits      = UART_StopBits_1;
    UART_InitStruct.Parity        = UART_Parity_No;
    UART_InitStruct.HWFlowControl = UART_HWFlowControl_None;
    UART_InitStruct.Mode          = UART_Mode_Rx | UART_Mode_Tx;
    UART_Init(UART2, &UART_InitStruct);

    UART_DMACmd(UART2, ENABLE);

    RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOA, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    UART_Cmd(UART2, ENABLE);
}

/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void UART2_RxData_DMA_Interrupt(uint8_t *Buffer, uint8_t Length)
{
    DMA_InitTypeDef  DMA_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBENR_DMA, ENABLE);

    DMA_DeInit(DMA1_Channel5);

    DMA_StructInit(&DMA_InitStruct);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(UART2->RDR);
    DMA_InitStruct.DMA_MemoryBaseAddr     = (uint32_t)Buffer;
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralSRC;
    DMA_InitStruct.DMA_BufferSize         = Length;
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_InitStruct.DMA_M2M                = DMA_M2M_Disable;
    DMA_InitStruct.DMA_Auto_reload        = DMA_Auto_Reload_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStruct);

    DMA_ClearFlag(DMA1_FLAG_TC5);
    DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = DMA1_Channel4_7_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    UART_RX_DMA_InterruptFlag = 0;

    DMA_Cmd(DMA1_Channel5, ENABLE);
}

/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void UART2_TxData_DMA_Interrupt(uint8_t *Buffer, uint8_t Length)
{
    DMA_InitTypeDef  DMA_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBENR_DMA, ENABLE);

    DMA_DeInit(DMA1_Channel4);

    DMA_StructInit(&DMA_InitStruct);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(UART2->TDR);
    DMA_InitStruct.DMA_MemoryBaseAddr     = (uint32_t)Buffer;
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralDST;
    DMA_InitStruct.DMA_BufferSize         = Length;
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_InitStruct.DMA_M2M                = DMA_M2M_Disable;
    DMA_InitStruct.DMA_Auto_reload        = DMA_Auto_Reload_Disable;
    DMA_Init(DMA1_Channel4, &DMA_InitStruct);

    DMA_ClearFlag(DMA1_FLAG_TC4);
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = DMA1_Channel4_7_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    UART_TX_DMA_InterruptFlag = 0;

    DMA_Cmd(DMA1_Channel4, ENABLE);
}

/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
// void UART_DMA_Interrupt_Sample(void)
// {
//     uint8_t Buffer[10];

//     printf("\r\nTest %s", __FUNCTION__);

//     UART_Configure(115200);

//     UART_RxData_DMA_Interrupt(Buffer, 10);

//     printf("\r\nSend 10 bytes to UART every time");

//     while (1)
//     {
//         if (0 != UART_RX_DMA_InterruptFlag)
//         {
//             UART_TxData_DMA_Interrupt(Buffer, 10);

//             while (0 == UART_TX_DMA_InterruptFlag)
//             {
//             }

//             UART_RxData_DMA_Interrupt(Buffer, 10);
//         }
//     }
// }

void Uart_Init(void)
{
  Uart1_Init(38400);
  Uart2_Init(38400);

}