/**
 ******************************************************************************
 * @file    sensible20_accelero.h
 * @author  MEMS Application Team
 * @brief   This header file contains the functions for the
 *          accelerometer driver
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
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

#include "sensible20_accelero.h"



static DrvContextTypeDef ACCELERO_SensorHandle;
static ACCELERO_Data_t ACCELERO_Data;   // Accelerometer.
static LIS2DW12_Data_t LIS2DW12_0_Data; // Accelerometer - sensor 5.

/**
 * @}
 */

/** @addtogroup SENSIBLE20_ACCELERO_Private_FunctionPrototypes Private function prototypes
 * @{
 */

static DrvStatusTypeDef BSP_LIS2DW12_ACCELERO_Init( void **handle );

/**
 * @}
 */

/** @addtogroup SENSIBLE20_ACCELERO_Public_Functions Public functions
 * @{
 */

/**
 * @brief Initialize an accelerometer sensor
 * @param id the accelerometer sensor identifier
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Init( ACCELERO_ID_t id, void **handle )
{

  *handle = NULL;

  switch(id)
  {
    default:
    case ACCELERO_SENSORS_AUTO:
      if (BSP_LIS2DW12_ACCELERO_Init(handle) == COMPONENT_OK )
      {
        return COMPONENT_OK;
      }
      break;

    case LIS2DW12_0:
      return BSP_LIS2DW12_ACCELERO_Init(handle);
  }

  return COMPONENT_ERROR;
}



/**
 * @brief Deinitialize accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_DeInit( void **handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)(*handle);
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if ( driver->DeInit == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->DeInit( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  memset(ctx, 0, sizeof(DrvContextTypeDef));

  *handle = NULL;

  return COMPONENT_OK;
}



/**
 * @brief Enable accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Sensor_Enable( void *handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if ( driver->Sensor_Enable == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Sensor_Enable( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Disable accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Sensor_Disable( void *handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if ( driver->Sensor_Disable == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Sensor_Disable( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Check if the accelerometer sensor is initialized
 * @param handle the device handle
 * @param status the pointer to the initialization status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_IsInitialized( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isInitialized;

  return COMPONENT_OK;
}


/**
 * @brief Check if the accelerometer sensor is enabled
 * @param handle the device handle
 * @param status the pointer to the enable status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_IsEnabled( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isEnabled;

  return COMPONENT_OK;
}


/**
 * @brief Check if the accelerometer sensor is combo
 * @param handle the device handle
 * @param status the pointer to the combo status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_IsCombo( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isCombo;

  return COMPONENT_OK;
}


/**
 * @brief Get the accelerometer sensor instance
 * @param handle the device handle
 * @param instance the pointer to the device instance
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Instance( void *handle, uint8_t *instance )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( instance == NULL )
  {
    return COMPONENT_ERROR;
  }

  *instance = ctx->instance;

  return COMPONENT_OK;
}



/**
 * @brief Get the WHO_AM_I ID of the accelerometer sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_WhoAmI( void *handle, uint8_t *who_am_i )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if ( driver->Get_WhoAmI == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_WhoAmI( ctx, who_am_i ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Check the WHO_AM_I ID of the accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Check_WhoAmI( void *handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if ( driver->Check_WhoAmI == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Check_WhoAmI( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the accelerometer sensor axes
 * @param handle the device handle
 * @param acceleration pointer where the values of the axes are written [mg]
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Axes( void *handle, SensorAxes_t *acceleration )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if(acceleration == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_Axes == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_Axes( ctx, acceleration ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the accelerometer sensor raw axes
 * @param handle the device handle
 * @param value pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_AxesRaw( void *handle, SensorAxesRaw_t *value )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if(value == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_AxesRaw == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_AxesRaw( ctx, value) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Get the accelerometer sensor sensitivity
 * @param handle the device handle
 * @param sensitivity pointer where the sensitivity value is written [mg/LSB]
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Sensitivity( void *handle, float *sensitivity )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if(sensitivity == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_Sensitivity == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_Sensitivity( ctx, sensitivity ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_ODR( void *handle, float *odr )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if(odr == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_ODR == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_ODR( ctx, odr ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_ODR( void *handle, SensorOdr_t odr )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if ( driver->Set_ODR == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Set_ODR( ctx, odr ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_ODR_Value( void *handle, float odr )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if ( driver->Set_ODR_Value == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Set_ODR_Value( ctx, odr ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale pointer where the full scale is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_FS( void *handle, float *fullScale )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if(fullScale == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_FS == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_FS( ctx, fullScale ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale the functional full scale to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_FS( void *handle, SensorFs_t fullScale )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if ( driver->Set_FS == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Set_FS( ctx, fullScale ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale the full scale value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_FS_Value( void *handle, float fullScale )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if ( driver->Set_FS_Value == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Set_FS_Value( ctx, fullScale ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the accelerometer sensor axes status
 * @param handle the device handle
 * @param xyz_enabled the pointer to the axes enabled/disabled status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Axes_Status( void *handle, uint8_t *xyz_enabled )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if(xyz_enabled == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_Axes_Status == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_Axes_Status( ctx, xyz_enabled ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the enabled/disabled status of the accelerometer sensor axes
 * @param handle the device handle
 * @param enable_xyz vector of the axes enabled/disabled status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_Axes_Status( void *handle, uint8_t *enable_xyz )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if(enable_xyz == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Set_Axes_Status == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Set_Axes_Status( ctx, enable_xyz ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Read the data from register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Read_Reg( void *handle, uint8_t reg, uint8_t *data )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if(data == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Read_Reg == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Read_Reg( ctx, reg, data ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Write the data to register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Write_Reg( void *handle, uint8_t reg, uint8_t data )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if ( driver->Write_Reg == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Write_Reg( ctx, reg, data ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get accelerometer data ready status
 * @param handle the device handle
 * @param status the data ready status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_DRDY_Status( void *handle, uint8_t *status )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if ( driver->Get_DRDY_Status == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_DRDY_Status( ctx, status ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @}
 */

/** @addtogroup SENSIBLE2.0_ACCELERO_Public_Functions_Ext Public functions for extended features
 * @{
 */

/**
 * @brief Read All Interrupt Source Register
 * @param handle the device handle
 * @param value  the pionter where to save the ALL_INT_SRC register value
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Read_All_Int_Src_Register_Ext(void *handle, uint8_t *value)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL) || (value == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;
        
        if (extDriver->Read_All_Int_Src_Register == NULL){
            goto fail;
        }
        return extDriver->Read_All_Int_Src_Register(ctx, value);
    }

fail: return COMPONENT_ERROR;
}

/**
 * @brief Enable the free fall detection
 * @param handle the device handle
 * @note  This function sets the LIS2DW12 accelerometer ODR to 50Hz and the LIS2DW12 accelerometer full scale to 2g
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Enable_Free_Fall_Detection_Ext(void *handle)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;
        
        if (extDriver->Enable_Free_Fall_Detection == NULL){
            goto fail;
        }
        return extDriver->Enable_Free_Fall_Detection(ctx);
    }

fail: return COMPONENT_ERROR;
}

/**
 * @brief Disable the free fall detection
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Disable_Free_Fall_Detection_Ext(void *handle)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->Disable_Free_Fall_Detection == NULL){
            goto fail;
        }
        return extDriver->Disable_Free_Fall_Detection(ctx);
    }
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Get the status of the free fall detection
 * @param handle the device handle
 * @param status the pointer to the status of free fall detection: 0 means no detection, 1 means detection happened
 * @note This function is deprecated and has been replaced by BSP_ACCELERO_Get_Event_Status_Ext
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Free_Fall_Detection_Status_Ext(void *handle, uint8_t *status)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL) || (status == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->Get_Free_Fall_Detection_Status == NULL){
            goto fail;
        }

        return extDriver->Get_Free_Fall_Detection_Status(ctx, status);
    }

fail: return COMPONENT_ERROR;
}

/**
 * @brief Set the free fall detection threshold
 * @param handle the device handle
 * @param thr the threshold to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_Free_Fall_Threshold_Ext(void *handle, uint8_t thr)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I)
    {
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if (extDriver->Set_Free_Fall_Threshold == NULL){
            goto fail;
        }
        
        return extDriver->Set_Free_Fall_Threshold(ctx, thr);
    }
    
fail: return COMPONENT_ERROR;
}


/**
 * @brief Enable the wake up detection (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param int_pin the interrupt pin to be used
 * @note  This function sets the LIS2DW12 accelerometer ODR to 50Hz and the LIS2DW12 accelerometer full scale to 2g
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Enable_Wake_Up_Detection_Ext(void *handle)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->Enable_Wake_Up_Detection == NULL){
            goto fail;
        }

        return extDriver->Enable_Wake_Up_Detection(ctx);
    }

fail: return COMPONENT_ERROR;
}

/**
 * @brief Disable the wake up detection
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Disable_Wake_Up_Detection_Ext(void *handle)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->Disable_Wake_Up_Detection == NULL){
            goto fail;
        }

        return extDriver->Disable_Wake_Up_Detection(ctx);
    }

fail: return COMPONENT_ERROR;
}

/**
 * @brief Get the status of the wake up detection
 * @param handle the device handle
 * @param status the pointer to the status of the wake up detection: 0 means no detection, 1 means detection happened
 * @note This function is deprecated and has been replaced by BSP_ACCELERO_Get_Event_Status_Ext
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Wake_Up_Detection_Status_Ext(void *handle, uint8_t *status)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL) || (status == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->Get_Wake_Up_Detection_Status == NULL){
            goto fail;
        }

        return extDriver->Get_Wake_Up_Detection_Status(ctx, status);
    }

fail: return COMPONENT_ERROR;
}



/**
 * @brief Set the wake up threshold
 * @param handle the device handle
 * @param thr the threshold to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_Wake_Up_Threshold_Ext(void *handle, uint8_t thr)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->Set_Wake_Up_Threshold == NULL){
            goto fail;
        }

        return extDriver->Set_Wake_Up_Threshold(ctx, thr);
    }

fail: return COMPONENT_ERROR;
}



/**
 * @brief Enable the single tap detection
 * @param handle the device handle
 * @note  This function sets the LIS2DW12 accelerometer ODR to 50Hz and the LIS2DW12 accelerometer full scale to 2g
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Enable_Single_Tap_Detection_Ext(void *handle)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->Enable_Single_Tap_Detection == NULL){
            goto fail;
        }

        return extDriver->Enable_Single_Tap_Detection(ctx);
    }

fail: return COMPONENT_ERROR;
}

/**
 * @brief Disable the single tap detection
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Disable_Single_Tap_Detection_Ext(void *handle)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL)){
        return COMPONENT_ERROR;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->Disable_Single_Tap_Detection == NULL){
            goto fail;
        }

        return extDriver->Disable_Single_Tap_Detection( ctx );
    }
    
fail: return COMPONENT_ERROR;

}

/**
 * @brief Get the single tap detection status
 * @param handle the device handle
 * @param status the pointer to the single tap detection status: 0 means no single tap detected, 1 means single tap detected
 * @note This function is deprecated and has been replaced by BSP_ACCELERO_Get_Event_Status_Ext
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Single_Tap_Detection_Status_Ext(void *handle, uint8_t *status)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL) || (status == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if (extDriver->Get_Single_Tap_Detection_Status == NULL){
            goto fail;
        }

        return extDriver->Get_Single_Tap_Detection_Status(ctx, status);
    }
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Enable the double tap detection
 * @param handle the device handle
 * @note  This function sets the LIS2DW12 accelerometer ODR to 50Hz and the LIS2DW12 accelerometer full scale to 2g
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Enable_Double_Tap_Detection_Ext(void *handle)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->Enable_Double_Tap_Detection == NULL){
            goto fail;
        }
        
        return extDriver->Enable_Double_Tap_Detection(ctx);
    }
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Disable the double tap detection
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Disable_Double_Tap_Detection_Ext(void *handle)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->Disable_Double_Tap_Detection == NULL){
            goto fail;
        }

        return extDriver->Disable_Double_Tap_Detection(ctx);
    }
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Get the double tap detection status
 * @param handle the device handle
 * @param status the pointer to the double tap detection status: 0 means no double tap detected, 1 means double tap detected
 * @note This function is deprecated and has been replaced by BSP_ACCELERO_Get_Event_Status_Ext
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Double_Tap_Detection_Status_Ext(void *handle, uint8_t *status)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL) || (status == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->Get_Double_Tap_Detection_Status == NULL){
            goto fail;
        }
        
        return extDriver->Get_Double_Tap_Detection_Status(ctx, status);
    }

fail: return COMPONENT_ERROR;
}

/**
 * @brief Set the tap threshold
 * @param handle the device handle
 * @param thr the threshold to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_Tap_Threshold_Ext(void *handle, uint8_t thr)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->Set_Tap_Threshold == NULL){
            goto fail;
        }

        return extDriver->Set_Tap_Threshold(ctx, thr);
    }

fail: return COMPONENT_ERROR;
}

/**
 * @brief Set the tap shock time window
 * @param handle the device handle
 * @param time the shock time window to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_Tap_Shock_Time_Ext(void *handle, uint8_t time)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->Set_Tap_Shock_Time == NULL){
            goto fail;
        }

        return extDriver->Set_Tap_Shock_Time(ctx, time);
    }

fail: return COMPONENT_ERROR;
}

/**
 * @brief Set the tap quiet time window
 * @param handle the device handle
 * @param time the quiet time window to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_Tap_Quiet_Time_Ext(void *handle, uint8_t time)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->Set_Tap_Quiet_Time == NULL){
            goto fail;
        }

        return extDriver->Set_Tap_Quiet_Time(ctx, time);
    }

fail: return COMPONENT_ERROR;
}

/**
 * @brief Set the tap duration of the time window
 * @param handle the device handle
 * @param time the duration of the time window to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_Tap_Duration_Time_Ext(void *handle, uint8_t time)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL)){
        goto fail;
    }
    
    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if (extDriver->Set_Tap_Duration_Time == NULL){
            goto fail;
        }

        return extDriver->Set_Tap_Duration_Time(ctx, time);
    }

fail: return COMPONENT_ERROR;
}



/**
 * @brief Enable the 6D orientation detection
 * @param handle the device handle
 * @param int_pin the interrupt pin to be used
 * @note  This function sets the LIS2DW12 accelerometer ODR to 50Hz and the LIS2DW12 accelerometer full scale to 2g
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Enable_6D_Orientation_Ext(void *handle)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->Enable_6D_Orientation == NULL){
            goto fail;
        }

        return extDriver->Enable_6D_Orientation(ctx);
    }
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Disable the 6D orientation detection
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Disable_6D_Orientation_Ext(void *handle)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->Disable_6D_Orientation == NULL){
            goto fail;
        }
           
        return extDriver->Disable_6D_Orientation(ctx);
    }
           
fail: return COMPONENT_ERROR;
}

/**
 * @brief Get the status of the 6D orientation detection
 * @param handle the device handle
 * @param status the pointer to the status of the 6D orientation detection: 0 means no detection, 1 means detection happened
 * @note This function is deprecated and has been replaced by BSP_ACCELERO_Get_Event_Status_Ext
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_Status_Ext(void *handle, uint8_t *status)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL) || (status == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->Get_6D_Orientation_Status == NULL){
            goto fail;
        }

        return extDriver->Get_6D_Orientation_Status(ctx, status);
    }

fail: return COMPONENT_ERROR;
}

/**
 * @brief Get the 6D orientation XL axis
 * @param handle the device handle
 * @param xl the pointer to the 6D orientation XL axis
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_XL_Ext(void *handle, uint8_t *xl)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL) || (xl == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->Get_6D_Orientation_XL == NULL){
            goto fail;
        }

        return extDriver->Get_6D_Orientation_XL(ctx, xl);
    }

fail: return COMPONENT_ERROR;
}

/**
 * @brief Get the 6D orientation XH axis
 * @param handle the device handle
 * @param xh the pointer to the 6D orientation XH axis
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_XH_Ext(void *handle, uint8_t *xh)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL) || (xh == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->Get_6D_Orientation_XH == NULL){
            goto fail;
        }

        return extDriver->Get_6D_Orientation_XH(ctx, xh);
    }

fail: return COMPONENT_ERROR;
}

/**
 * @brief Get the 6D orientation YL axis
 * @param handle the device handle
 * @param yl the pointer to the 6D orientation YL axis
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_YL_Ext(void *handle, uint8_t *yl)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL) || (yl == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->Get_6D_Orientation_YL == NULL){
            goto fail;
        }

        return extDriver->Get_6D_Orientation_YL(ctx, yl);
    }

fail: return COMPONENT_ERROR;
}

/**
 * @brief Get the 6D orientation YH axis
 * @param handle the device handle
 * @param yh the pointer to the 6D orientation YH axis
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_YH_Ext(void *handle, uint8_t *yh)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL) || (yh == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->Get_6D_Orientation_YH == NULL){
            goto fail;
        }
        
        return extDriver->Get_6D_Orientation_YH(ctx, yh);
    }

fail: return COMPONENT_ERROR;
}

/**
 * @brief Get the 6D orientation ZL axis
 * @param handle the device handle
 * @param zl the pointer to the 6D orientation ZL axis
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_ZL_Ext(void *handle, uint8_t *zl)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL) || (zl == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->Get_6D_Orientation_ZL == NULL){
            goto fail;
        }

        return extDriver->Get_6D_Orientation_ZL( ctx, zl );
    }

fail: return COMPONENT_ERROR;
}

/**
 * @brief Get the 6D orientation ZH axis
 * @param handle the device handle
 * @param zh the pointer to the 6D orientation ZH axis
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_ZH_Ext(void *handle, uint8_t *zh)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL) || (zh == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->Get_6D_Orientation_ZH == NULL){
            goto fail;
        }

        return extDriver->Get_6D_Orientation_ZH(ctx, zh);
    }

fail: return COMPONENT_ERROR;
}



/**
 * @brief Set FIFO output data rate
 * @param handle the device handle
 * @param odr the output data rate
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_FIFO_Set_ODR_Value_Ext(void *handle, float odr)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->FIFO_Set_ODR_Value == NULL){
            goto fail;
        }

        return extDriver->FIFO_Set_ODR_Value(handle, odr);
    }

fail: return COMPONENT_ERROR;
}

/**
 * @brief Get FIFO full status
 * @param handle the device handle
 * @param *status FIFO full status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_FIFO_Get_Full_Status_Ext( void *handle, uint8_t *status )
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL) || (status == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->FIFO_Get_Full_Status == NULL){
            goto fail;
        }

        return extDriver->FIFO_Get_Full_Status(handle, status);
    }

fail: return COMPONENT_ERROR;
}

/**
 * @brief Get FIFO empty status
 * @param handle the device handle
 * @param *status FIFO empty status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_FIFO_Get_Empty_Status_Ext(void *handle, uint8_t *status)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL) || (status == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->FIFO_Get_Empty_Status == NULL){
            goto fail;
        }

        return extDriver->FIFO_Get_Empty_Status(handle, status);
    }

fail: return COMPONENT_ERROR;
}

/**
 * @brief Get FIFO_OVR bit status
 * @param handle the device handle
 * @param *status FIFO_OVR bit status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_FIFO_Get_Overrun_Status_Ext(void *handle, uint8_t *status)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL) || (status == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;
        
        if(extDriver->FIFO_Get_Overrun_Status == NULL)
        {
            goto fail;
        }
        
        return extDriver->FIFO_Get_Overrun_Status(handle, status);
    }

fail: return COMPONENT_ERROR;
}

/**
 * @brief Get FIFO data
 * @param handle the device handle
 * @param *aData FIFO data array
 * @param size   the number of data to be read
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_FIFO_Get_Data_Ext(void *handle, SensorAxes_t *aData, uint8_t size)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL) || (size == 0)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->FIFO_Get_Data == NULL){
            goto fail;
        }
        
        return extDriver->FIFO_Get_Data(handle, aData, size);
    }
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Get number of unread FIFO samples
 * @param handle the device handle
 * @param *nSamples Number of unread FIFO samples
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_FIFO_Get_Num_Of_Samples_Ext(void *handle, uint8_t *nSamples)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL) || (nSamples == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;
        
        if(extDriver->FIFO_Get_Num_Of_Samples == NULL){
            goto fail;
        }
        
        return extDriver->FIFO_Get_Num_Of_Samples(handle, nSamples);
    }

fail: return COMPONENT_ERROR;
}

/**
 * @brief Set FIFO mode
 * @param handle the device handle
 * @param mode FIFO mode
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_FIFO_Set_Mode_Ext(void *handle, uint8_t mode)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;
        
        if(extDriver->FIFO_Set_Mode == NULL){
            goto fail;
        }
        
        return extDriver->FIFO_Set_Mode(handle, mode);
    }

fail: return COMPONENT_ERROR;
}

/**
 * @brief Set FIFO_FULL interrupt on INT1 pin
 * @param handle the device handle
 * @param status FIFO_FULL interrupt on INT1 pin enable/disable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_FIFO_Set_INT1_FIFO_Full_Ext(void *handle, uint8_t status)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->FIFO_Set_INT1_FIFO_Full == NULL){
            goto fail;
        }

        return extDriver->FIFO_Set_INT1_FIFO_Full(handle, status);
    }

fail: return COMPONENT_ERROR;
}

/**
 * @brief Set FIFO_TRESHOLD interrupt on INT1 pin
 * @param handle the device handle
 * @param status FIFO_TRESHOLD interrupt on INT1 pin enable/disable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_FIFO_Set_INT1_FIFO_Treshold_Ext(void *handle, uint8_t status)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->FIFO_Set_INT1_FIFO_Treshold == NULL){
            goto fail;
        }

        return extDriver->FIFO_Set_INT1_FIFO_Treshold(handle, status);
    }

fail: return COMPONENT_ERROR;
}

/**
 * @brief Set FIFO_TRESHOLD interrupt on INT2 pin
 * @param handle the device handle
 * @param status FIFO_TRESHOLD interrupt on INT2 pin enable/disable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_FIFO_Set_INT2_FIFO_Treshold_Ext(void *handle, uint8_t status)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->FIFO_Set_INT2_FIFO_Treshold == NULL){
            goto fail;
        }

        return extDriver->FIFO_Set_INT2_FIFO_Treshold(handle, status);
    }

fail: return COMPONENT_ERROR;
}

/**
 * @brief Set FIFO watermark level
 * @param handle the device handle
 * @param watermark FIFO watermark level
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_FIFO_Set_Watermark_Level_Ext(void *handle, uint8_t watermark)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->FIFO_Set_Watermark_Level == NULL){
            goto fail;
        }

        return extDriver->FIFO_Set_Watermark_Level(handle, watermark);
    }

fail: return COMPONENT_ERROR;
}



/**
 * @brief Set interrupt latch
 * @param handle the device handle
 * @param status interrupt latch enable/disable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_Interrupt_Latch_Ext(void *handle, uint8_t status)
{
    DrvContextTypeDef *ctx = (DrvContextTypeDef*)handle;

    if((ctx == NULL) || (ctx->pExtVTable == NULL)){
        goto fail;
    }

    if(ctx->who_am_i == LIS2DW12_ACC_WHO_AM_I){
        LIS2DW12_ExtDrv_t *extDriver = (LIS2DW12_ExtDrv_t*)ctx->pExtVTable;

        if(extDriver->Set_Interrupt_Latch == NULL){
            goto fail;
        }

        return extDriver->Set_Interrupt_Latch(handle, status);
    }

fail: return COMPONENT_ERROR;
}



/**
 * @brief Enable the accelerometer sensor HP Filter
 * @param handle the device handle
 * @param mode the value of HP Filter mode to be set
 * @param cutoff the value of HP Filter cutoff to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Enable_HP_Filter_Ext( void *handle )
{
#if 1 // Isn't implemented for LIS2DW
  return COMPONENT_ERROR;
#else  
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  if ( ctx->instance == LIS2DH12_0 )
  {
    if ( ctx->who_am_i == LIS2DH12_WHO_AM_I )
    {
      LIS2DH12_ExtDrv_t *extDriver = ( LIS2DH12_ExtDrv_t * )ctx->pExtVTable;
      
      if ( extDriver->Enable_HP_Filter == NULL )
      {
        return COMPONENT_ERROR;
      }
      
      if ( extDriver->Enable_HP_Filter( ctx ) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      return COMPONENT_OK; 
    }
  } else if ( ctx->instance == LSM303AGR_X_0 )
  {
    if ( ctx->who_am_i == LSM303AGR_ACC_WHO_AM_I )
    {
      LSM303AGR_X_ExtDrv_t *extDriver = ( LSM303AGR_X_ExtDrv_t * )ctx->pExtVTable; 
      
      if ( extDriver->Enable_HP_Filter == NULL )
      {
        return COMPONENT_ERROR;
      }
      
      if ( extDriver->Enable_HP_Filter( ctx ) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      return COMPONENT_OK; 
    }
  } else if ( ctx->instance == LSM6DSL_X_0 )
  {
    if ( ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I )
    {
      LSM6DSL_X_ExtDrv_t *extDriver = ( LSM6DSL_X_ExtDrv_t * )ctx->pExtVTable;   
      
      if ( extDriver->Enable_HP_Filter == NULL )
      {
        return COMPONENT_ERROR;
      }
      
      if ( extDriver->Enable_HP_Filter( ctx ) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      return COMPONENT_OK;      
    }
  } else if ( ctx->instance == LSM6DS0_X_0 )
  {
    if ( ctx->who_am_i == LSM6DS0_ACC_GYRO_WHO_AM_I )
    {
      LSM6DS0_X_ExtDrv_t *extDriver = ( LSM6DS0_X_ExtDrv_t * )ctx->pExtVTable;   
      
      if ( extDriver->Enable_HP_Filter == NULL )
      {
        return COMPONENT_ERROR;
      }
      
      if ( extDriver->Enable_HP_Filter( ctx ) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      return COMPONENT_OK;      
    }
  } 
  
  return COMPONENT_ERROR;
#endif
}



/**
 * @brief Disable the accelerometer sensor HP Filter
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Disable_HP_Filter_Ext( void *handle )
{
#if 1 // Isn't implemented for LIS2DW
  return COMPONENT_ERROR;
#else  
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  if ( ctx->instance == LIS2DH12_0 )
  {
    if ( ctx->who_am_i == LIS2DH12_WHO_AM_I )
    {
      LIS2DH12_ExtDrv_t *extDriver = ( LIS2DH12_ExtDrv_t * )ctx->pExtVTable;
      
      if ( extDriver->Disable_HP_Filter == NULL )
      {
        return COMPONENT_ERROR;
      }
      
      if ( extDriver->Disable_HP_Filter( ctx ) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      return COMPONENT_OK; 
    } 
  } 
  else if ( ctx->instance == LSM303AGR_X_0 )
  {
    if ( ctx->who_am_i == LSM303AGR_ACC_WHO_AM_I )
    {
      LSM303AGR_X_ExtDrv_t *extDriver = ( LSM303AGR_X_ExtDrv_t * )ctx->pExtVTable;
      
      if ( extDriver->Disable_HP_Filter == NULL )
      {
        return COMPONENT_ERROR;
      }
      
      if ( extDriver->Disable_HP_Filter( ctx ) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      return COMPONENT_OK; 
    } 
  } else if ( ctx->instance == LSM6DSL_X_0 )
  {
    if ( ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I )
    {
      LSM6DSL_X_ExtDrv_t *extDriver = ( LSM6DSL_X_ExtDrv_t * )ctx->pExtVTable;   
      
      if ( extDriver->Disable_HP_Filter == NULL )
      {
        return COMPONENT_ERROR;
      }
      
      if ( extDriver->Disable_HP_Filter( ctx ) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      return COMPONENT_OK;      
    }
  } else if ( ctx->instance == LSM6DS0_X_0 )
  {
    if ( ctx->who_am_i == LSM6DS0_ACC_GYRO_WHO_AM_I )
    {
      LSM6DS0_X_ExtDrv_t *extDriver = ( LSM6DS0_X_ExtDrv_t * )ctx->pExtVTable;   
      
      if ( extDriver->Disable_HP_Filter == NULL )
      {
        return COMPONENT_ERROR;
      }
      
      if ( extDriver->Disable_HP_Filter( ctx ) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      return COMPONENT_OK;      
    }
  }
  
  return COMPONENT_ERROR;
#endif
}



/**
 * @brief Clear the accelerometer sensor DRDY
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_ClearDRDY_Ext( void *handle, ACTIVE_AXIS_t axisActive )
{
 #if 1 // Isn't implemented for LIS2DW
  return COMPONENT_ERROR;
#else 
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }  
  
  if ( ctx->instance == LIS2DH12_0 )
  {
    if ( ctx->who_am_i == LIS2DH12_WHO_AM_I )
    {
      LIS2DH12_ExtDrv_t *extDriver = ( LIS2DH12_ExtDrv_t * )ctx->pExtVTable;
      
      if ( extDriver->ClearDRDY == NULL )
      {
        return COMPONENT_ERROR;
      }
      
      if ( extDriver->ClearDRDY( ctx, axisActive ) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      return COMPONENT_OK; 
    }
  } 
  else if ( ctx->instance == LSM303AGR_X_0 )
  {
    if ( ctx->who_am_i == LSM303AGR_ACC_WHO_AM_I )
    {
      LSM303AGR_X_ExtDrv_t *extDriver = ( LSM303AGR_X_ExtDrv_t * )ctx->pExtVTable; 
      
      if ( extDriver->ClearDRDY == NULL )
      {
        return COMPONENT_ERROR;
      }
      
      if ( extDriver->ClearDRDY( ctx, axisActive ) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      return COMPONENT_OK; 
    } 
  } else if ( ctx->instance == LSM6DSL_X_0 )
  {
    if ( ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I )
    {
      LSM6DSL_X_ExtDrv_t *extDriver = ( LSM6DSL_X_ExtDrv_t * )ctx->pExtVTable;   
      
      if ( extDriver->ClearDRDY == NULL )
      {
        return COMPONENT_ERROR;
      }
      
      if ( extDriver->ClearDRDY( ctx, axisActive ) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      return COMPONENT_OK;      
    }
  } else if ( ctx->instance == LSM6DS0_X_0 )
  {
    if ( ctx->who_am_i == LSM6DS0_ACC_GYRO_WHO_AM_I )
    {
      LSM6DS0_X_ExtDrv_t *extDriver = ( LSM6DS0_X_ExtDrv_t * )ctx->pExtVTable;   
      
      if ( extDriver->ClearDRDY == NULL )
      {
        return COMPONENT_ERROR;
      }
      
      if ( extDriver->ClearDRDY( ctx, axisActive ) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      return COMPONENT_OK;      
    }
  }
  
  return COMPONENT_ERROR;
#endif
}



/**
 * @brief Set the accelerometer sensor DRDY on INT1
 * @param handle the device handle
 * @param drdyStatus enable/disable DRDY on INT1 value
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_INT1_DRDY_Ext( void *handle, INT1_DRDY_CONFIG_t drdyStatus )
{
#if 1 // Isn't implemented for LIS2DW
  return COMPONENT_ERROR;
#else
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  if ( ctx->instance == LIS2DH12_0 )
  {
    if ( ctx->who_am_i == LIS2DH12_WHO_AM_I )
    {
      LIS2DH12_ExtDrv_t *extDriver = ( LIS2DH12_ExtDrv_t * )ctx->pExtVTable;
      
      if ( extDriver->Set_INT1_DRDY == NULL )
      {
        return COMPONENT_ERROR;
      }
      
      if ( extDriver->Set_INT1_DRDY( ctx, drdyStatus ) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      return COMPONENT_OK;  
    }
  } 
  else if ( ctx->instance == LSM303AGR_X_0 )
  {
    if ( ctx->who_am_i == LSM303AGR_ACC_WHO_AM_I )
    {
      LSM303AGR_X_ExtDrv_t *extDriver = ( LSM303AGR_X_ExtDrv_t * )ctx->pExtVTable;
      
      if ( extDriver->Set_INT1_DRDY == NULL )
      {
        return COMPONENT_ERROR;
      }
      
      if ( extDriver->Set_INT1_DRDY( ctx, drdyStatus ) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      return COMPONENT_OK;  
    }
  } else if ( ctx->instance == LSM6DSL_X_0 )
  {
    if ( ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I )
    {
      LSM6DSL_X_ExtDrv_t *extDriver = ( LSM6DSL_X_ExtDrv_t * )ctx->pExtVTable;   
      
      if ( extDriver->Set_INT1_DRDY == NULL )
      {
        return COMPONENT_ERROR;
      }
      
      if ( extDriver->Set_INT1_DRDY( ctx, drdyStatus ) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      return COMPONENT_OK;      
    }
  } else if ( ctx->instance == LSM6DS0_X_0 )
  {
    if ( ctx->who_am_i == LSM6DS0_ACC_GYRO_WHO_AM_I )
    {
      LSM6DS0_X_ExtDrv_t *extDriver = ( LSM6DS0_X_ExtDrv_t * )ctx->pExtVTable;   
      
      if ( extDriver->Set_INT1_DRDY == NULL )
      {
        return COMPONENT_ERROR;
      }
      
      if ( extDriver->Set_INT1_DRDY( ctx, drdyStatus ) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      return COMPONENT_OK;      
    }
  }
  
  return COMPONENT_ERROR;
#endif
}



/**
 * @brief Set the sensor DRDY mode
 * @param handle the device handle
 * @param drdyMode mode of DRDY
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_DRDY_Mode_Ext( void *handle, DRDY_MODE_t drdyMode )
{
#if 1 // Isn't implemented for LIS2DW
  return COMPONENT_ERROR;
#else
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  if ( ctx->instance == LSM6DSL_X_0 )
  {
    if ( ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I )
    {
      LSM6DSL_X_ExtDrv_t *extDriver = ( LSM6DSL_X_ExtDrv_t * )ctx->pExtVTable;   
      
      if ( extDriver->Set_DRDY_Mode == NULL )
      {
        return COMPONENT_ERROR;
      }
      
      if ( extDriver->Set_DRDY_Mode( ctx, drdyMode ) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      return COMPONENT_OK;
    }
  }
  
  return COMPONENT_ERROR;
#endif
}


/**
 * @brief Initialize an LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef BSP_LIS2DW12_ACCELERO_Init( void **handle )
{
  ACCELERO_Drv_t *driver = NULL;

  /* Setup sensor handle. */
  ACCELERO_SensorHandle.who_am_i      = LIS2DW12_ACC_WHO_AM_I;
  ACCELERO_SensorHandle.ifType        = 0; /* I2C interface */
  ACCELERO_SensorHandle.address       = LIS2DW12_I2C_ADDRESS_LOW;
  ACCELERO_SensorHandle.instance      = LIS2DW12_0;
  ACCELERO_SensorHandle.isInitialized = 0;
  ACCELERO_SensorHandle.isEnabled     = 0;
  ACCELERO_SensorHandle.isCombo       = 0;
  ACCELERO_SensorHandle.pData         = ( void * )&ACCELERO_Data;
  ACCELERO_SensorHandle.pVTable       = ( void * )&LIS2DW12_Drv;
  ACCELERO_SensorHandle.pExtVTable    = ( void * )&LIS2DW12_ExtDrv;

  ACCELERO_Data.pComponentData = ( void * )&LIS2DW12_0_Data;
  ACCELERO_Data.pExtData       = 0;

  *handle = ( void * )&ACCELERO_SensorHandle;

  driver = ( ACCELERO_Drv_t * )( ( DrvContextTypeDef * )( *handle ) )->pVTable;

  if ( driver->Init == NULL )
  {
    memset( ( *handle ), 0, sizeof( DrvContextTypeDef ) );
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  if ( driver->Init( ( DrvContextTypeDef * )( *handle ) ) == COMPONENT_ERROR )
  {
    memset( ( *handle ), 0, sizeof( DrvContextTypeDef ) );
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
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

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
