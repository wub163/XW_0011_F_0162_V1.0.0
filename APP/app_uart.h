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



#define U_RE RA7 // 485收发控制引脚

    typedef struct
    {
        uint16_t CHEAD;   // 接收协议头部0xAA55
        uint8_t CT;       // 控制字
        uint8_t data[64]; // 节点数据
        uint8_t sumCheck; // 和校验

    } UART_RX_S;

    typedef struct
    {
        uint16_t CHEAD; // 发送协议头部0xAA66
        uint8_t ID;     // 本机ID
        uint8_t attr;   // 反馈数据属性
        uint16_t data;  // 反馈数据
    } UART_TX_S;

//extern volatile FLAG8 flag8_uart;     // 串口通讯控制标志位
#define f_RxBUFempty flag8_uart.bits.bit0 // 缓冲区已清空标志位，可写入
#define f_RxOKfull flag8_uart.bits.bit1   // 缓冲区接收数据完成标志位，可取出
#define f_TxBufOK flag8_uart.bits.bit2    // 缓冲区发送数据准备好。可发送
#define f_TxOK flag8_uart.bits.bit3       // 缓冲区发送数据完成。等待关闭RE
#define f_TxREOK flag8_uart.bits.bit4     // 发送数据完成后RE切换完成。

    void protocol_Init(void); // 协议初始化程序
    void UART_Isr(void);      // 串口中断服务函数
    void UART_Process(void);  // 中断方式发送协议数据
    void Send_TX_int(void);   // 反馈包发送函数

    void Uart_Init(void);   //通讯端口初始化

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif // _APP_UART_H