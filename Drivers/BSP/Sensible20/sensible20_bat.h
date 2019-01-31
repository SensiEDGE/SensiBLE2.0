/**
 ******************************************************************************
 * @file    sensible20_bat.h
 * @date    20-November-2018
 * @brief   sensible20_bat header driver file.
 ******************************************************************************
 *
 * COPYRIGHT(c) 2019 SensiEDGE
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of SensiEDGE nor the names of its contributors may
 *      be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSIBLE_2_0_BAT_H
#define __SENSIBLE_2_0_BAT_H

/* Includes ------------------------------------------------------------------*/
#include "steval_bluemic1.h" 
/* Public defines ------------------------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/

/**
  * @brief  Initialization of the pin, destined for switching on/off 
  *         the transistor key for measuring battery level
  * @param  none
  * @retval none
  */
void BSP_BatLevel_Pin_Init(void);

/**
  * @brief  Deinitialization of the ADC,
  *         destined for measuring battery level
  * @param  none
  * @retval none
  */
void BSP_BatLevel_IN_DeInit(void);

/**
  * @brief  Initialization of the ADC,
  *         destined for measuring battery level
  * @param  none
  * @retval none
  */
void BSP_BatLevel_IN_Init(void);

/**
  * @brief  This function receives the pointers to soc, voltage and current,
  *         where writes current battery and consumption values
  * @param  soc      the pointer where to save the battery level in percentage
  * @param  voltage  the pointer where to save the battery voltage level
  * @param  current  the pointer where to save current consumption in mA
  *                  (is not supported yet)
  * @retval none
  */
void BSP_BatLevel_GetValues(uint32_t *soc, uint32_t *voltage, int32_t *current);

 #endif //__SENSIBLE_2_0_BAT_H

/************************ (C) COPYRIGHT SensiEdge *****************************/
