/***********************************************************************************************************************
    @file    platform.h
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
#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Files include */
#include "hal_conf.h"

/* Exported types *****************************************************************************************************/
// #define D2 GPIOD, GPIO_Pin_5
// #define D3 GPIOA, GPIO_Pin_1
#define D2 GPIOC, GPIO_Pin_9
#define D3 GPIOC, GPIO_Pin_8
#define ZKRE GPIOA, GPIO_Pin_6
#define FKRE GPIOB, GPIO_Pin_2
#define DJRE GPIOB, GPIO_Pin_5
#define ZKTX01 GPIOA, GPIO_Pin_7
#define FKTX01 GPIOB, GPIO_Pin_1
#define FKRX01 GPIOB, GPIO_Pin_0

// #define K1 GPIOA, GPIO_Pin_0
#define K1 GPIOC, GPIO_Pin_11
#define DJID GPIOB, GPIO_Pin_8
#define IDX_8 GPIOA, GPIO_Pin_8
#define IDX_4 GPIOA, GPIO_Pin_9
#define IDX_2 GPIOA, GPIO_Pin_10
#define IDX_1 GPIOA, GPIO_Pin_11
#define IDY_8 GPIOA, GPIO_Pin_12
#define IDY_4 GPIOA, GPIO_Pin_15
#define IDY_2 GPIOB, GPIO_Pin_3
#define IDY_1 GPIOB, GPIO_Pin_4

/* Exported constants *************************************************************************************************/

/* Exported macro *****************************************************************************************************/
#define out(a,b) GPIO_WriteBit(a, (BitAction)b) 
#define in(a) GPIO_ReadInputDataBit(a)
#define tog(a) GPIO_WriteBit(a, (BitAction)!GPIO_ReadOutputDataBit(a))
/* Exported variables *************************************************************************************************/
#undef EXTERN

#ifdef _PLATFORM_C_
#define EXTERN
#else
#define EXTERN extern
#endif

EXTERN volatile uint32_t PLATFORM_DelayTick;

/* Exported functions *************************************************************************************************/
void PLATFORM_DelayMS(uint32_t Millisecond);
void PLATFORM_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* _PLATFORM_H_ */

/********************************************** (C) Copyright MindMotion **********************************************/

