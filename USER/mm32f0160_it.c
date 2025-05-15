/***********************************************************************************************************************
    @file    mm32f0160_it.c
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
#define _MM32F0160_IT_C_

/* Files include */
#include "application.h"
#include "mm32f0160_it.h"
extern volatile FLAG8 sys_flag;

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

/* Private macro ******************************************************************************************************/

/* Private variables **************************************************************************************************/

/* Private functions **************************************************************************************************/

/***********************************************************************************************************************
 * @brief  This function handles NMI exception
 * @note   none
 * @param  none
 * @retval none
 *********************************************************************************************************************/
void NMI_Handler(void)
{
}

/***********************************************************************************************************************
 * @brief  This function handles Hard Fault exception
 * @note   none
 * @param  none
 * @retval none
 *********************************************************************************************************************/
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/***********************************************************************************************************************
 * @brief  This function handles SVCall exception
 * @note   none
 * @param  none
 * @retval none
 *********************************************************************************************************************/
void SVC_Handler(void)
{
}

/***********************************************************************************************************************
 * @brief  This function handles PendSVC exception
 * @note   none
 * @param  none
 * @retval none
 *********************************************************************************************************************/
void PendSV_Handler(void)
{
}

/***********************************************************************************************************************
 * @brief  This function handles SysTick Handler
 * @note   none
 * @param  none
 * @retval none
 *********************************************************************************************************************/
void SysTick_Handler(void)
{
  SysTickCount++;
  sys_flag.mask = 0xff;
}

/***********************************************************************************************************************
 * @brief  This function handles DMA1_Channel4_7 Handler
 * @note   none
 * @param  none
 * @retval none
 *********************************************************************************************************************/
void DMA1_Channel4_7_IRQHandler(void)
{
  if (RESET != DMA_GetITStatus(DMA1_IT_TC4))
  {
    DMA_Cmd(DMA1_Channel4, DISABLE);

    //       UART_TX_DMA_InterruptFlag = 1;

    DMA_ClearITPendingBit(DMA1_IT_TC4);
  }

  if (RESET != DMA_GetITStatus(DMA1_IT_TC5))
  {
    DMA_Cmd(DMA1_Channel5, DISABLE);

    //        UART_RX_DMA_InterruptFlag = 1;

    DMA_ClearITPendingBit(DMA1_IT_TC5);
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
