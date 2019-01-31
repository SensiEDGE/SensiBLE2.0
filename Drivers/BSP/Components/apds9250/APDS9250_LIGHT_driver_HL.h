/**
 ******************************************************************************
 * @file    APDS9250_LIGHT_driver_HL.
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
#ifndef __APDS_9250_LIGHT_DRIVER_HL_H
#define __APDS_9250_LIGHT_DRIVER_HL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "light.h"

/* Include magnetic sensor component drivers. */
#include "APDS9250_LIGHT_driver.h"


/**
 * @brief APDS9250 data structure definition
 */
typedef struct
{
    uint32_t Ir;
    uint32_t Grn;
    uint32_t Blue;
    uint32_t Red;
    uint32_t Light;
} APDS9250_Data_t;

/**
 * @}
 */

/** @addtogroup APDS9250_Public_Variables Public variables
 * @{
 */

extern LIGHT_Drv_t APDS9250Drv;

#ifdef __cplusplus
}
#endif

#endif /* __APDS_9250_LIGHT_DRIVER_HL_H */

/************************ (C) COPYRIGHT SensiEdge *****************************/
