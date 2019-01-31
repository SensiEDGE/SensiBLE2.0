/**
 ******************************************************************************
 * @file    APDS9250_LIGHT_driver.h
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
#ifndef __APDS9250_DRIVER__H
#define __APDS9250_DRIVER__H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Uncomment the line below to expanse the "assert_param" macro in the drivers code */
#define USE_FULL_ASSERT_APDS9250

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT_apdS9250

/**
* @brief  The assert_param macro is used for function's parameters check.
* @param  expr: If expr is false, it calls assert_failed function which reports
*         the name of the source file and the source line number of the call
*         that failed. If expr is true, it returns no value.
* @retval None
*/
#define APDS9250_assert_param(expr) ((expr) ? (void)0 : APDS9250_assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
void APDS9250_assert_failed(uint8_t* file, uint32_t line);
#else
#define APDS9250_assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT_APDS9250 */

typedef enum {APDS9250_OK = (uint8_t)0, APDS9250_ERROR = !APDS9250_OK} APDS9250_Error_et;

typedef enum {APDS9250_DISABLE = (uint8_t)0, APDS9250_ENABLE = !APDS9250_DISABLE} APDS9250_State_et;
#define IS_APDS9250_State(MODE) ((MODE == APDS9250_ENABLE) || (MODE == APDS9250_DISABLE))

typedef enum {APDS9250_RESET = (uint8_t)0, APDS9250_SET = !APDS9250_RESET} APDS9250_BitStatus_et;
#define IS_APDS9250_BitStatus(MODE) ((MODE == APDS9250_RESET) || (MODE == APDS9250_SET))

typedef enum
{
  APDS9250_GAIN_1         = (uint8_t)0x00,         /*!< Gain 1  */
  APDS9250_GAIN_3         = (uint8_t)0x01,         /*!< Gain 3  */
  APDS9250_GAIN_6         = (uint8_t)0x02,         /*!< Gain 6  */
  APDS9250_GAIN_9         = (uint8_t)0x03,         /*!< Gain 9  */
  APDS9250_GAIN_18        = (uint8_t)0x04,         /*!< Gain 18 */
} APDS9250_Gain_et;

#define IS_APDS9250_GAIN(AVGH) ((AVGH == APDS9250_GAIN_1)  || \
                                (AVGH == APDS9250_GAIN_3)  || \
                                (AVGH == APDS9250_GAIN_6)  || \
                                (AVGH == APDS9250_GAIN_18))

/* Exported Constants ---------------------------------------------------------*/

/**
* @brief  Bitfield positioning.
*/
#define APDS9250_BIT(x) ((uint8_t)x)

/**
* @brief  I2C address.
*/
#define APDS9250_I2C_ADDRESS  (uint8_t)0x52

/**
* @addtogroup APDS9250_Registers
* @{
*/

#define APDS9250_REG_MAIN_CTRL          0x00
#define APDS9250_REG_LS_MEAS_RATE       0x04
#define APDS9250_REG_LS_GAIN            0x05
#define APDS9250_REG_PART_ID            0x06
#define APDS9250_REG_MAIN_STATUS        0x07
#define APDS9250_REG_DATA_IR_0          0x0A
#define APDS9250_REG_DATA_IR_1          0x0B
#define APDS9250_REG_DATA_IR_2          0x0C
#define APDS9250_REG_DATA_GREEN_0       0x0D
#define APDS9250_REG_DATA_GREEN_1       0x0E
#define APDS9250_REG_DATA_GREEN_2       0x0F
#define APDS9250_REG_DATA_BLUE_0        0x10
#define APDS9250_REG_DATA_BLUE_1        0x11
#define APDS9250_REG_DATA_BLUE_2        0x12
#define APDS9250_REG_DATA_RED_0         0x13
#define APDS9250_REG_DATA_RED_1         0x14
#define APDS9250_REG_DATA_RED_2         0x15
#define APDS9250_REG_INT_CFG            0x19
#define APDS9250_REG_INT_PERSISTENCE    0x1A
#define APDS9250_REG_THRES_UP_0         0x21
#define APDS9250_REG_THRES_UP_1         0x22
#define APDS9250_REG_THRES_UP_2         0x23
#define APDS9250_REG_THRES_LOW_0        0x24
#define APDS9250_REG_THRES_LOW_1        0x25
#define APDS9250_REG_THRES_LOW_2        0x26
#define APDS9250_REG_THRES_VAR          0x27

// Register values
#define APDS9250_PART_ID                0xB2

#define APDS9250_DEVICE_ID_MASK         0xF0

/* Exported Functions -------------------------------------------------------------*/
/** @defgroup APDS9250_Exported_Functions
* @{
*/

APDS9250_Error_et APDS9250_ReadReg( void *handle, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data );
APDS9250_Error_et APDS9250_WriteReg( void *handle, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data );

APDS9250_Error_et APDS9250_Activate      (void *handle);
APDS9250_Error_et APDS9250_DeActivate    (void *handle);
APDS9250_Error_et APDS9250_SensorEnable  (void *handle);
APDS9250_Error_et APDS9250_SensorDisable (void *handle);
APDS9250_Error_et APDS9250_GetWhoAmI     (void *handle, uint8_t *who_am_i);
APDS9250_Error_et APDS9250_GetIr         (void *handle, uint32_t *value);
APDS9250_Error_et APDS9250_GetGrn        (void *handle, uint32_t *value);
APDS9250_Error_et APDS9250_GetBlue       (void *handle, uint32_t *value);
APDS9250_Error_et APDS9250_GetRed        (void *handle, uint32_t *value);
APDS9250_Error_et APDS9250_GetLight      (void *handle, uint32_t *value);



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

#endif /* __APDS9250_DRIVER__H */

/************************ (C) COPYRIGHT SensiEdge *****************************/
