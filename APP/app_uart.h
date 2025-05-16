#ifndef _APP_UART_H
#define _APP_UART_H

#ifdef __cplusplus
extern "C"
{
#endif


/* Files include */
#include "hal_conf.h"
#include "application.h"
#undef EXTERN

#ifdef _UART_DMA_INTERRUPT_C_
#define EXTERN
#else
#define EXTERN extern
#endif

EXTERN volatile uint32_t UART_TX_DMA_InterruptFlag;
EXTERN volatile uint32_t UART_RX_DMA_InterruptFlag;

/* Exported functions *************************************************************************************************/
void UART_DMA_Interrupt_Sample(void);



#define U_RE RA7 // 485�շ���������

    typedef struct
    {
        uint16_t CHEAD;   // ����Э��ͷ��0xAA55
        uint8_t CT;       // ������
        uint8_t data[64]; // �ڵ�����
        uint8_t sumCheck; // ��У��

    } UART_RX_S;

    typedef struct
    {
        uint16_t CHEAD; // ����Э��ͷ��0xAA66
        uint8_t ID;     // ����ID
        uint8_t attr;   // ������������
        uint16_t data;  // ��������
    } UART_TX_S;

//extern volatile FLAG8 flag8_uart;     // ����ͨѶ���Ʊ�־λ
#define f_RxBUFempty flag8_uart.bits.bit0 // ����������ձ�־λ����д��
#define f_RxOKfull flag8_uart.bits.bit1   // ����������������ɱ�־λ����ȡ��
#define f_TxBufOK flag8_uart.bits.bit2    // ��������������׼���á��ɷ���
#define f_TxOK flag8_uart.bits.bit3       // ����������������ɡ��ȴ��ر�RE
#define f_TxREOK flag8_uart.bits.bit4     // ����������ɺ�RE�л���ɡ�

    void protocol_Init(void); // Э���ʼ������
    void UART_Isr(void);      // �����жϷ�����
    void UART_Process(void);  // �жϷ�ʽ����Э������
    void Send_TX_int(void);   // ���������ͺ���

    void Uart_Init(void);   //ͨѶ�˿ڳ�ʼ��

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif // _APP_UART_H