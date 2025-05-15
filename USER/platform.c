/*
 * 	platform.c
 *	功能描述：初始化平台IO端口（按键，指示灯，模块控制线，模块配置）
 *	创建时间: 2025.05.15
 *	开发人员：WUB
 *	开发公司: 昆山祥维
 */

/* Define to prevent recursive inclusion */
#define _PLATFORM_C_

/* Files include */
#include <stdio.h>
#include "platform.h"

volatile uint32_t PLATFORM_DelayTick;
/***********************************************************************************************************************
 * @brief  Initialize SysTick for delay function
 * @note   none
 * @param  none
 * @retval none
 *********************************************************************************************************************/
void PLATFORM_InitDelay(void)
{
  RCC_ClocksTypeDef RCC_Clocks;

  RCC_GetClocksFreq(&RCC_Clocks);

  if (SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000))
  {
    while (1)
    {
    }
  }

  NVIC_SetPriority(SysTick_IRQn, 0x0);
}

/***********************************************************************************************************************
 * @brief  Millisecond delay
 * @note   none
 * @param  Millisecond: delay time unit
 * @retval none
 *********************************************************************************************************************/
void PLATFORM_DelayMS(uint32_t Millisecond)
{
  PLATFORM_DelayTick = Millisecond;

  while (0 != PLATFORM_DelayTick)
  {
  }
}

/***********************************************************************************************************************
 * @brief  Initialize console for printf
 * @note   none
 * @param  Baudrate : UART1 communication baudrate
 * @retval none
 *********************************************************************************************************************/
void PLATFORM_InitConsole(uint32_t Baudrate)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  UART_InitTypeDef UART_InitStruct;

  RCC_APB2PeriphClockCmd(RCC_APB2ENR_UART1, ENABLE);

  UART_StructInit(&UART_InitStruct);
  UART_InitStruct.BaudRate = Baudrate;
  UART_InitStruct.WordLength = UART_WordLength_8b;
  UART_InitStruct.StopBits = UART_StopBits_1;
  UART_InitStruct.Parity = UART_Parity_No;
  UART_InitStruct.HWFlowControl = UART_HWFlowControl_None;
  UART_InitStruct.Mode = UART_Mode_Tx;
  UART_Init(UART1, &UART_InitStruct);

  UART_Cmd(UART1, ENABLE);

  RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOB, ENABLE);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_0);

  GPIO_StructInit(&GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/***********************************************
函数名称：void PLATFORM_IO_Init(void)
函数功能：平台IO初始化，按键，指示灯，模块控制线
入口参数：无
出口参数：无
备注：
************************************************/
void PLATFORM_IO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOA, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOB, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOD, ENABLE);

  GPIO_StructInit(&GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_7 | GPIO_Pin_6;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_5;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOD, &GPIO_InitStruct);

//=============================开发板支持
  RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOC, ENABLE);
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &GPIO_InitStruct);
//=================================
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12| GPIO_Pin_15;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_8;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

out(D2, Bit_SET);
out(D3, Bit_SET);
out(D2, Bit_RESET);
out(D3, Bit_RESET);
tog(D2);
tog(D2);
tog(D3);
tog(D3);


out(ZKRE, Bit_SET);
out(FKRE, Bit_SET);
out(DJRE, Bit_SET);
out(ZKTX01, Bit_SET);
out(FKTX01, Bit_SET);
out(FKRX01, Bit_SET);
}



/***********************************************************************************************************************
 * @brief  Print information
 * @note   none
 * @param  none
 * @retval none
 *********************************************************************************************************************/
void PLATFORM_PrintInfo(void)
{
  RCC_ClocksTypeDef RCC_Clocks;

  printf("\r\nBOARD : Mini-F0163");
  printf("\r\nMCU : MM32F0163D7P");

  printf("\r\n");

  switch (RCC->CFGR & RCC_CFGR_SWS)
  {
  case 0x00:
    printf("\r\nHSI used as system clock source");
    break;

  case 0x04:
    printf("\r\nHSE used as system clock source");
    break;

  case 0x08:
    if (RCC->PLL1CFGR & RCC_PLL1CFGR_PLL1SRC)
    {
      printf("\r\nPLL1 (clocked by HSE) used as system clock source");
    }
    else
    {
      printf("\r\nPLL1 (clocked by HSI) used as system clock source");
    }

    break;

  case 0x0C:
    printf("\r\nLSI used as system clock source");
    break;

  default:
    break;
  }

  RCC_GetClocksFreq(&RCC_Clocks);

  printf("\r\n");
  printf("\r\nSYSCLK Frequency : %7.3f MHz", (double)RCC_Clocks.SYSCLK_Frequency / (double)1000000.0);
  printf("\r\nHCLK   Frequency : %7.3f MHz", (double)RCC_Clocks.HCLK_Frequency / (double)1000000.0);
  printf("\r\nPCLK1  Frequency : %7.3f MHz", (double)RCC_Clocks.PCLK1_Frequency / (double)1000000.0);
  printf("\r\nPCLK2  Frequency : %7.3f MHz", (double)RCC_Clocks.PCLK2_Frequency / (double)1000000.0);
  printf("\r\n");
}

/***********************************************************************************************************************
 * @brief  Initialize Platform
 * @note   none
 * @param  none
 * @retval none
 *********************************************************************************************************************/
void PLATFORM_Init(void)
{
  PLATFORM_InitDelay();

  PLATFORM_InitConsole(115200);

  PLATFORM_IO_Init();

  // PLATFORM_PrintInfo();
}
