/**
 ******************************************************************************
 * @file    acceleroSampleProcessing.h
 * @brief   acceleroSampleProcessing.
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
#ifndef __ACCELERO_SAMPLES_PROCESSING_H
#define __ACCELERO_SAMPLES_PROCESSING_H

/* Includes ------------------------------------------------------------------*/
#include "steval_bluemic1.h" 

/* Public defines ------------------------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/

/**
  * @brief  Get the number of steps
  * @param  none
  * @retval none
  */
uint16_t getSteps(void);

/**
  * @brief  Readout the accelero FIFO
  * @param  handle  the Accelero handle
  * @retval none
  */
void flushAcceleroFifo(void *handle);

/**
  * @brief  Look for steps
  * @param  none
  * @retval TRUE    if steps were found
  * @retval FALSE   if steps were not found
  */
BOOL lookForSteps(void);

/**
  * @brief  Look for tilt
  * @param  none
  * @retval TRUE    if tilt was found
  * @retval FALSE   if tilt was not found
  */
BOOL lookForTilt(void);

/**
  * @brief  Reset the number of steps
  * @param  none
  * @retval none
  */
void resetSteps(void);

 #endif //__ACCELERO_SAMPLES_PROCESSING_H

/************************ (C) COPYRIGHT SensiEdge *****************************/
