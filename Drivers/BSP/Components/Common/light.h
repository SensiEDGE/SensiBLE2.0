/**
 ******************************************************************************
 * @file    light.h
 * @date    5-January-2018
 * @brief   This file provides a set of high-level functions needed to manage
            the APDS9250 light sensor
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
#ifndef __LIGHT_H
#define __LIGHT_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "sensor.h"

/**
 * @brief  LIGHT driver structure definition
 */
typedef struct
{
  DrvStatusTypeDef ( *Init            ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *DeInit          ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Sensor_Enable   ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Sensor_Disable  ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_WhoAmI      ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Check_WhoAmI    ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_Ir          ) ( DrvContextTypeDef*, uint32_t* );
  DrvStatusTypeDef ( *Get_Grn         ) ( DrvContextTypeDef*, uint32_t* );
  DrvStatusTypeDef ( *Get_Blue        ) ( DrvContextTypeDef*, uint32_t* );
  DrvStatusTypeDef ( *Get_Red         ) ( DrvContextTypeDef*, uint32_t* );
  DrvStatusTypeDef ( *Get_Light       ) ( DrvContextTypeDef*, uint32_t* );
  DrvStatusTypeDef ( *Get_Fs          ) ( DrvContextTypeDef*, float *);
  DrvStatusTypeDef ( *Set_Fs          ) ( DrvContextTypeDef*, SensorFs_t );
} LIGHT_Drv_t;

/**
 * @brief  ULTRAVIOLET data structure definition
 */
typedef struct
{
  void *pComponentData; /* Component specific data. */
  void *pExtData;       /* Other data. */
} LIGHT_Data_t;

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __ULTRAVIOLET_H */

/************************ (C) COPYRIGHT SensiEdge *****************************/
