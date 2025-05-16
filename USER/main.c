/***********************************************************************************************************************
    @file    main.c
    @author  FAE Team
    @date    23-Aug-2023
    @brief   THIS FILE PROVIDES ALL THE SYSTEM FUNCTIONS.
  **********************************************************************************************************************
    @attention

    <h2><center>&copy; Copyright(c) <2023> <MindMotion></center></h2>

      Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
    following conditions are met:
    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
       the following disclaimer in the documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
       promote products derived from this software without specific prior written permission.

      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
    INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
    WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *********************************************************************************************************************/

/* Define to prevent recursive inclusion */
#define _MAIN_C_

/* Files include */
#include "application.h"
#include "main.h"

/**
 * @addtogroup MM32F0160_LibSamples
 * @{
 */

/**
 * @addtogroup UART
 * @{
 */

/**
 * @addtogroup UART_DMA_Interrupt
 * @{
 */

/* Private typedef ****************************************************************************************************/

/* Private define *****************************************************************************************************/
extern volatile u32 SysTickCount;
volatile FLAG8 sys_flag;
#define f_systick_loop sys_flag.bits.bit0 // 主系统循环标识位。
#define f_moto_loop sys_flag.bits.bit1    // 编码器服务循环标识位。
#define f_uart_loop sys_flag.bits.bit2    // 串口服务循环标识位。
#define f_led_loop sys_flag.bits.bit3     // 状态服务循环标识位。
#define f_systick_bad sys_flag.bits.bit4     // 状态服务循环标识位。

/* Private macro ******************************************************************************************************/

/* Private variables **************************************************************************************************/

/* Private functions **************************************************************************************************/

/***********************************************************************************************************************
 * @brief  This function is main entrance
 * @note   main
 * @param  none
 * @retval none
 *********************************************************************************************************************/
int main(void)
{
  PLATFORM_Init();

 // UART_DMA_Interrupt_Sample();

  while (1)
  {
    f_systick_bad = 0;
    if (((SysTickCount % 10) == 0) && (f_systick_loop)) // 20ms处理一次位置反馈任务
    {
    if (f_systick_bad)continue;	   // 如果系统循环参数失效，跳过本次循环
      f_systick_loop = 0; // 清除时钟标志位
    }

    if (((SysTickCount % 20) == 0) && (f_moto_loop)) // 20ms处理一次位置反馈任务
    {
    if (f_systick_bad)continue;	   // 如果系统循环参数失效，跳过本次循环
      f_moto_loop = 0; // 清除时钟标志位
      OUT(D2,IN(K1));
    }

    if (((SysTickCount % 100) == 0) && (f_uart_loop)) // 20ms处理一次位置反馈任务
    {
    if (f_systick_bad)continue;	   // 如果系统循环参数失效，跳过本次循环
      f_uart_loop = 0; // 清除时钟标志位
      TOG(D3);
      UART_SendData(UART1, 0x18);
    }

    if (((SysTickCount % 500) == 0) && (f_led_loop)) // 20ms处理一次位置反馈任务
    {
    if (f_systick_bad)continue;	   // 如果系统循环参数失效，跳过本次循环
      f_led_loop = 0; // 清除时钟标志位

      TOG(D2);
      UART_SendData(UART2, 0x28);  
    }
  }
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/********************************************** (C) Copyright MindMotion **********************************************/
