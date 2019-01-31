/**
 ******************************************************************************
 * @file    APDS9250_LIGHT_driver_HL.c
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

/* Includes ------------------------------------------------------------------*/
#include "APDS9250_LIGHT_driver_HL.h"
#include <math.h>

static DrvStatusTypeDef APDS9250_Get_Ir    ( DrvContextTypeDef *handle, uint32_t * value);
static DrvStatusTypeDef APDS9250_Get_Grn   ( DrvContextTypeDef *handle, uint32_t * value);
static DrvStatusTypeDef APDS9250_Get_Blue  ( DrvContextTypeDef *handle, uint32_t * value);
static DrvStatusTypeDef APDS9250_Get_Red   ( DrvContextTypeDef *handle, uint32_t * value);
static DrvStatusTypeDef APDS9250_Get_Light ( DrvContextTypeDef *handle, uint32_t * value);

/**
 * @}
 */

/** @addtogroup APDS9250_Callable_Private_FunctionPrototypes Callable private function prototypes
 * @{
 */

static DrvStatusTypeDef APDS9250_Init             ( DrvContextTypeDef *handle );
static DrvStatusTypeDef APDS9250_DeInit           ( DrvContextTypeDef *handle );
static DrvStatusTypeDef APDS9250_Sensor_Enable    ( DrvContextTypeDef *handle );
static DrvStatusTypeDef APDS9250_Sensor_Disable   ( DrvContextTypeDef *handle );
static DrvStatusTypeDef APDS9250_Get_WhoAmI       ( DrvContextTypeDef *handle, uint8_t *who_am_i );
static DrvStatusTypeDef APDS9250_Check_WhoAmI     ( DrvContextTypeDef *handle );
static DrvStatusTypeDef APDS9250_Read_Reg         ( DrvContextTypeDef *handle, uint8_t reg, uint8_t* data );
static DrvStatusTypeDef APDS9250_Write_Reg        ( DrvContextTypeDef *handle, uint8_t reg, uint8_t data );

/**
 * @}
 */

/** @addtogroup LIS3MDL_Private_Variables Private variables
 * @{
 */

/**
 * @brief LIS3MDL driver structure
 */
LIGHT_Drv_t APDS9250Drv =
{
  APDS9250_Init,
  APDS9250_DeInit,
  APDS9250_Sensor_Enable,
  APDS9250_Sensor_Disable,
  APDS9250_Get_WhoAmI,
  APDS9250_Check_WhoAmI,
  APDS9250_Get_Ir,
  APDS9250_Get_Grn,
  APDS9250_Get_Blue,
  APDS9250_Get_Red,
  APDS9250_Get_Light
};

/**
 * @}
 */

/** @addtogroup LIS3MDL_Callable_Private_Functions Callable private functions
 * @{
 */

static DrvStatusTypeDef APDS9250_Init             ( DrvContextTypeDef *handle )
{
    return COMPONENT_ERROR;
}

static DrvStatusTypeDef APDS9250_DeInit           ( DrvContextTypeDef *handle )
{
    return COMPONENT_ERROR;
}

static DrvStatusTypeDef APDS9250_Sensor_Enable    ( DrvContextTypeDef *handle )
{        
    handle->isEnabled = 1;   
    return COMPONENT_OK;
}

static DrvStatusTypeDef APDS9250_Sensor_Disable   ( DrvContextTypeDef *handle )
{
    handle->isEnabled = 0;
    return COMPONENT_OK;
}

static DrvStatusTypeDef APDS9250_Get_WhoAmI       ( DrvContextTypeDef *handle, uint8_t *who_am_i )
{
    if(APDS9250_OK == APDS9250_GetWhoAmI(handle, who_am_i)) {
        return COMPONENT_OK;
    }
    return COMPONENT_ERROR;
}

static DrvStatusTypeDef APDS9250_Check_WhoAmI     ( DrvContextTypeDef *handle )
{
    uint8_t who_am_i = 0;
    
    if(COMPONENT_OK == APDS9250_Get_WhoAmI(handle, &who_am_i)) {
        if((who_am_i & APDS9250_DEVICE_ID_MASK) ==
            APDS9250_PART_ID & APDS9250_DEVICE_ID_MASK) {
            return COMPONENT_OK;
        }
    }
    
    return COMPONENT_ERROR;
}

static DrvStatusTypeDef APDS9250_Get_Ir    ( DrvContextTypeDef *handle, uint32_t * value)
{
    if(APDS9250_OK == APDS9250_GetIr(handle, value)) {
        return COMPONENT_OK;
    }
    return COMPONENT_ERROR;
}

static DrvStatusTypeDef APDS9250_Get_Grn   ( DrvContextTypeDef *handle, uint32_t * value)
{
    if(APDS9250_OK == APDS9250_GetGrn(handle, value)) {
        return COMPONENT_OK;
    }
    return COMPONENT_ERROR;
}

static DrvStatusTypeDef APDS9250_Get_Blue  ( DrvContextTypeDef *handle, uint32_t * value)
{
    if(APDS9250_OK == APDS9250_GetBlue(handle, value)) {
        return COMPONENT_OK;
    }
    return COMPONENT_ERROR;
}

static DrvStatusTypeDef APDS9250_Get_Red   ( DrvContextTypeDef *handle, uint32_t * value)
{
    if(APDS9250_OK == APDS9250_GetRed(handle, value)) {
        return COMPONENT_OK;
    }
    return COMPONENT_ERROR;
}

static DrvStatusTypeDef APDS9250_Get_Light ( DrvContextTypeDef *handle, uint32_t * value)
{
    if(APDS9250_OK == APDS9250_GetLight(handle, value)) {
        return COMPONENT_OK;
    }
    return COMPONENT_ERROR;
}

static DrvStatusTypeDef APDS9250_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{
  if ( APDS9250_ReadReg( (void *)handle, reg, 1, data ) == APDS9250_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

static DrvStatusTypeDef APDS9250_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( APDS9250_WriteReg( (void *)handle, reg, 1, &data ) == APDS9250_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/************************ (C) COPYRIGHT SensiEdge *****************************/
