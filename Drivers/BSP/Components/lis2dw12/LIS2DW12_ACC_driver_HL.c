/**
 ******************************************************************************
 * @file    LIS2DW12_ACC_driver_HL.c
 * @author  MEMS Application Team
 * @brief   This file provides a set of high-level functions needed to manage
            the LIS2DW12 sensor
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

#include "LIS2DW12_ACC_driver_HL.h"



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup LIS2DW12 LIS2DW12
 * @{
 */

/** @addtogroup LIS2DW12_Callable_Private_Function_Prototypes Callable private function prototypes
 * @{
 */

static DrvStatusTypeDef LIS2DW12_Init( DrvContextTypeDef *handle );
static DrvStatusTypeDef LIS2DW12_DeInit( DrvContextTypeDef *handle );
static DrvStatusTypeDef LIS2DW12_Sensor_Enable( DrvContextTypeDef *handle );
static DrvStatusTypeDef LIS2DW12_Sensor_Disable( DrvContextTypeDef *handle );
static DrvStatusTypeDef LIS2DW12_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i );
static DrvStatusTypeDef LIS2DW12_Check_WhoAmI( DrvContextTypeDef *handle );
static DrvStatusTypeDef LIS2DW12_Get_Axes( DrvContextTypeDef *handle, SensorAxes_t *acceleration );
static DrvStatusTypeDef LIS2DW12_Get_AxesRaw( DrvContextTypeDef *handle, SensorAxesRaw_t *value );
static DrvStatusTypeDef LIS2DW12_Get_Sensitivity( DrvContextTypeDef *handle, float *sensitivity );
static DrvStatusTypeDef LIS2DW12_Get_ODR( DrvContextTypeDef *handle, float *odr );
static DrvStatusTypeDef LIS2DW12_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef LIS2DW12_Set_ODR_Value( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef LIS2DW12_Get_FS( DrvContextTypeDef *handle, float *fullScale );
static DrvStatusTypeDef LIS2DW12_Set_FS( DrvContextTypeDef *handle, SensorFs_t fs );
static DrvStatusTypeDef LIS2DW12_Set_FS_Value( DrvContextTypeDef *handle, float fullScale );
static DrvStatusTypeDef LIS2DW12_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data );
static DrvStatusTypeDef LIS2DW12_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data );
static DrvStatusTypeDef LIS2DW12_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status );

static DrvStatusTypeDef LIS2DW12_Set_ODR_When_Enabled( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef LIS2DW12_Set_ODR_When_Disabled( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef LIS2DW12_Set_ODR_Value_When_Enabled( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef LIS2DW12_Set_ODR_Value_When_Disabled( DrvContextTypeDef *handle, float odr );

/**
 * @}
 */

/** @addtogroup LSM6DS3_Callable_Private_Function_Ext_Prototypes Callable private function for extended features prototypes
 * @{
 */
static DrvStatusTypeDef LIS2DW12_ReadAllInterruptSrcRegister(DrvContextTypeDef *handle, uint8_t *value);

static DrvStatusTypeDef LIS2DW12_Enable_Free_Fall_Detection(DrvContextTypeDef *handle);
static DrvStatusTypeDef LIS2DW12_Disable_Free_Fall_Detection(DrvContextTypeDef *handle);
static DrvStatusTypeDef LIS2DW12_Get_Free_Fall_Detection_Status(DrvContextTypeDef *handle, uint8_t *status);
static DrvStatusTypeDef LIS2DW12_Set_Free_Fall_Threshold(DrvContextTypeDef *handle, uint8_t thr);

static DrvStatusTypeDef LIS2DW12_Enable_Wake_Up_Detection( DrvContextTypeDef *handle );
static DrvStatusTypeDef LIS2DW12_Disable_Wake_Up_Detection( DrvContextTypeDef *handle );
static DrvStatusTypeDef LIS2DW12_Get_Wake_Up_Detection_Status( DrvContextTypeDef *handle, uint8_t *status );
static DrvStatusTypeDef LIS2DW12_Set_Wake_Up_Threshold( DrvContextTypeDef *handle, uint8_t thr );

static DrvStatusTypeDef LIS2DW12_Enable_Single_Tap_Detection( DrvContextTypeDef *handle );
static DrvStatusTypeDef LIS2DW12_Disable_Single_Tap_Detection( DrvContextTypeDef *handle );
static DrvStatusTypeDef LIS2DW12_Get_Single_Tap_Detection_Status( DrvContextTypeDef *handle, uint8_t *status );
static DrvStatusTypeDef LIS2DW12_Enable_Double_Tap_Detection( DrvContextTypeDef *handle );
static DrvStatusTypeDef LIS2DW12_Disable_Double_Tap_Detection( DrvContextTypeDef *handle );
static DrvStatusTypeDef LIS2DW12_Get_Double_Tap_Detection_Status( DrvContextTypeDef *handle, uint8_t *status );
static DrvStatusTypeDef LIS2DW12_Set_Tap_Threshold( DrvContextTypeDef *handle, uint8_t thr );
static DrvStatusTypeDef LIS2DW12_Set_Tap_Shock_Time( DrvContextTypeDef *handle, uint8_t time );
static DrvStatusTypeDef LIS2DW12_Set_Tap_Quiet_Time( DrvContextTypeDef *handle, uint8_t time );
static DrvStatusTypeDef LIS2DW12_Set_Tap_Duration_Time( DrvContextTypeDef *handle, uint8_t time );

static DrvStatusTypeDef LIS2DW12_Enable_6D_Orientation( DrvContextTypeDef *handle );
static DrvStatusTypeDef LIS2DW12_Disable_6D_Orientation( DrvContextTypeDef *handle );
static DrvStatusTypeDef LIS2DW12_Get_6D_Orientation_Status( DrvContextTypeDef *handle, uint8_t *status );
static DrvStatusTypeDef LIS2DW12_Get_6D_Orientation_XL( DrvContextTypeDef *handle, uint8_t *xl );
static DrvStatusTypeDef LIS2DW12_Get_6D_Orientation_XH( DrvContextTypeDef *handle, uint8_t *xh );
static DrvStatusTypeDef LIS2DW12_Get_6D_Orientation_YL( DrvContextTypeDef *handle, uint8_t *yl );
static DrvStatusTypeDef LIS2DW12_Get_6D_Orientation_YH( DrvContextTypeDef *handle, uint8_t *yh );
static DrvStatusTypeDef LIS2DW12_Get_6D_Orientation_ZL( DrvContextTypeDef *handle, uint8_t *zl );
static DrvStatusTypeDef LIS2DW12_Get_6D_Orientation_ZH( DrvContextTypeDef *handle, uint8_t *zh );

static DrvStatusTypeDef LIS2DW12_FIFO_Set_ODR_Value( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef LIS2DW12_FIFO_Get_Full_Status( DrvContextTypeDef *handle, uint8_t *status );
static DrvStatusTypeDef LIS2DW12_FIFO_Get_Empty_Status( DrvContextTypeDef *handle, uint8_t *status );
static DrvStatusTypeDef LIS2DW12_FIFO_Get_Overrun_Status( DrvContextTypeDef *handle, uint8_t *status );
static DrvStatusTypeDef LIS2DW12_FIFO_Get_Data( DrvContextTypeDef *handle, SensorAxes_t *aData, uint8_t size);
static DrvStatusTypeDef LIS2DW12_FIFO_Get_Num_Of_Samples( DrvContextTypeDef *handle, uint8_t *nSamples );
static DrvStatusTypeDef LIS2DW12_FIFO_Set_Mode( DrvContextTypeDef *handle, uint8_t mode );
static DrvStatusTypeDef LIS2DW12_FIFO_Set_INT1_FIFO_Full( DrvContextTypeDef *handle, uint8_t status );
static DrvStatusTypeDef LIS2DW12_FIFO_Set_INT1_FIFO_Treshold(DrvContextTypeDef *handle, uint8_t status);
static DrvStatusTypeDef LIS2DW12_FIFO_Set_INT2_FIFO_Treshold(DrvContextTypeDef *handle, uint8_t status);
static DrvStatusTypeDef LIS2DW12_FIFO_Set_Watermark_Level( DrvContextTypeDef *handle, uint8_t watermark );

static DrvStatusTypeDef LIS2DW12_Set_Interrupt_Latch(DrvContextTypeDef *handle, uint8_t status);

/**
 * @}
 */

/** @addtogroup LIS2DW12_Public_Variables Public variables
 * @{
 */

/**
 * @brief LIS2DW12 accelero extended features driver internal structure
 */
LIS2DW12_ExtDrv_t LIS2DW12_ExtDrv =
{
    LIS2DW12_ReadAllInterruptSrcRegister,
    LIS2DW12_Enable_Free_Fall_Detection,
    LIS2DW12_Disable_Free_Fall_Detection,
    LIS2DW12_Get_Free_Fall_Detection_Status,
    LIS2DW12_Set_Free_Fall_Threshold,
    LIS2DW12_Enable_Wake_Up_Detection,
    LIS2DW12_Disable_Wake_Up_Detection,
    LIS2DW12_Get_Wake_Up_Detection_Status,
    LIS2DW12_Set_Wake_Up_Threshold,
    LIS2DW12_Enable_Single_Tap_Detection,
    LIS2DW12_Disable_Single_Tap_Detection,
    LIS2DW12_Get_Single_Tap_Detection_Status,
    LIS2DW12_Enable_Double_Tap_Detection,
    LIS2DW12_Disable_Double_Tap_Detection,
    LIS2DW12_Get_Double_Tap_Detection_Status,
    LIS2DW12_Set_Tap_Threshold,
    LIS2DW12_Set_Tap_Shock_Time,
    LIS2DW12_Set_Tap_Quiet_Time,
    LIS2DW12_Set_Tap_Duration_Time,
    LIS2DW12_Enable_6D_Orientation,
    LIS2DW12_Disable_6D_Orientation,
    LIS2DW12_Get_6D_Orientation_Status,
    LIS2DW12_Get_6D_Orientation_XL,
    LIS2DW12_Get_6D_Orientation_XH,
    LIS2DW12_Get_6D_Orientation_YL,
    LIS2DW12_Get_6D_Orientation_YH,
    LIS2DW12_Get_6D_Orientation_ZL,
    LIS2DW12_Get_6D_Orientation_ZH,
    LIS2DW12_FIFO_Set_ODR_Value,
    LIS2DW12_FIFO_Get_Full_Status,
    LIS2DW12_FIFO_Get_Empty_Status,
    LIS2DW12_FIFO_Get_Overrun_Status,
    LIS2DW12_FIFO_Get_Data,
    LIS2DW12_FIFO_Get_Num_Of_Samples,
    LIS2DW12_FIFO_Set_Mode,
    LIS2DW12_FIFO_Set_INT1_FIFO_Full,
    LIS2DW12_FIFO_Set_INT1_FIFO_Treshold,
    LIS2DW12_FIFO_Set_INT2_FIFO_Treshold,
    LIS2DW12_FIFO_Set_Watermark_Level,
    LIS2DW12_Set_Interrupt_Latch
};
     
/**
 * @brief LIS2DW12 accelero driver structure
 */
ACCELERO_Drv_t LIS2DW12_Drv =
{
  LIS2DW12_Init,
  LIS2DW12_DeInit,
  LIS2DW12_Sensor_Enable,
  LIS2DW12_Sensor_Disable,
  LIS2DW12_Get_WhoAmI,
  LIS2DW12_Check_WhoAmI,
  LIS2DW12_Get_Axes,
  LIS2DW12_Get_AxesRaw,
  LIS2DW12_Get_Sensitivity,
  LIS2DW12_Get_ODR,
  LIS2DW12_Set_ODR,
  LIS2DW12_Set_ODR_Value,
  LIS2DW12_Get_FS,
  LIS2DW12_Set_FS,
  LIS2DW12_Set_FS_Value,
  0,//Get_Axes_Status
  0,//Set_Axes_Status
  LIS2DW12_Read_Reg,
  LIS2DW12_Write_Reg,
  LIS2DW12_Get_DRDY_Status
};

/**
 * @}
 */

/** @addtogroup LIS2DW12_Callable_Private_Functions Callable private functions
 * @{
 */

/**
 * @brief Initialize the LIS2DW12 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Init( DrvContextTypeDef *handle )
{

  if ( LIS2DW12_Check_WhoAmI( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable BDU. */
  if ( LIS2DW12_ACC_W_BlockDataUpdate( ( void * )handle, LIS2DW12_ACC_BDU_ENABLE ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* FIFO mode selection - FIFO disable. */
  if ( LIS2DW12_ACC_W_FIFO_mode( ( void * )handle, LIS2DW12_ACC_FMODE_BYPASS ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Output data rate selection - power down. */
  if ( LIS2DW12_ACC_W_OutputDataRate( ( void* )handle, LIS2DW12_ACC_ODR_POWER_DOWN ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Set default power mode - High resolution. */
  /* _NOTE_: In case of LIS2DW12_ACC_MODE_LOW_POWER_STD user should also choose one of four Low Power modes */
  if ( LIS2DW12_ACC_W_ModeSelection( ( void * )handle, LIS2DW12_ACC_MODE_HIGH_PERFORMANCE ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Select default output data rate. */
  if ( LIS2DW12_Set_ODR_When_Disabled( handle, ODR_MID_HIGH ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Full scale selection. */
  if ( LIS2DW12_Set_FS( handle, FS_LOW ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  handle->isInitialized = 1;

  return COMPONENT_OK;
}

/**
 * @brief Deinitialize the LIS2DW12 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_DeInit( DrvContextTypeDef *handle )
{
  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  LIS2DW12_Data_t *pComponentData = ( LIS2DW12_Data_t * )pData->pComponentData;

  if ( LIS2DW12_Check_WhoAmI( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Disable the component */
  if( LIS2DW12_Sensor_Disable( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Reset previous output data rate. */
  pComponentData->Previous_ODR = 0.0f;

  handle->isInitialized = 0;

  return COMPONENT_OK;
}

/**
 * @brief Enable the LIS2DW12 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Sensor_Enable( DrvContextTypeDef *handle )
{
  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  LIS2DW12_Data_t *pComponentData = ( LIS2DW12_Data_t * )pData->pComponentData;

  /* Check if the component is already enabled */
  if ( handle->isEnabled == 1 )
  {
    return COMPONENT_OK;
  }

  /* Output data rate selection. */
  if ( LIS2DW12_Set_ODR_Value_When_Enabled( handle, pComponentData->Previous_ODR ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  handle->isEnabled = 1;

  return COMPONENT_OK;
}

/**
 * @brief Disable the LIS2DW12 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Sensor_Disable( DrvContextTypeDef *handle )
{
  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  LIS2DW12_Data_t *pComponentData = ( LIS2DW12_Data_t * )pData->pComponentData;

  /* Check if the component is already disabled */
  if ( handle->isEnabled == 0 )
  {
    return COMPONENT_OK;
  }

  /* Store actual output data rate. */
  if ( LIS2DW12_Get_ODR( handle, &( pComponentData->Previous_ODR ) ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Output data rate selection - power down. */
  if ( LIS2DW12_ACC_W_OutputDataRate( ( void* )handle, LIS2DW12_ACC_ODR_POWER_DOWN ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  handle->isEnabled = 0;

  return COMPONENT_OK;
}

/**
 * @brief Get the WHO_AM_I ID of the LIS2DW12 sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{
  /* Read WHO AM I register */
  if ( LIS2DW12_ACC_R_WhoAmI( ( void * )handle, ( uint8_t* )who_am_i ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Check the WHO_AM_I ID of the LIS2DW12 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Check_WhoAmI( DrvContextTypeDef *handle )
{
  uint8_t who_am_i = 0x00;

  if ( LIS2DW12_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if ( who_am_i != handle->who_am_i )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the LIS2DW12 accelerometer sensor axes
 * @param handle the device handle
 * @param acceleration pointer to where acceleration data write to
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Get_Axes( DrvContextTypeDef *handle, SensorAxes_t *acceleration )
{
  SensorAxesRaw_t dataRaw;
  float sensitivity = 0;

  /* Read raw data from LIS2DW12 output register. */
  if ( LIS2DW12_Get_AxesRaw( handle, &dataRaw ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Get LIS2DW12 actual sensitivity. */
  if ( LIS2DW12_Get_Sensitivity( handle, &sensitivity ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Calculate the data. */
  acceleration->AXIS_X = ( int32_t )( dataRaw.AXIS_X * sensitivity );
  acceleration->AXIS_Y = ( int32_t )( dataRaw.AXIS_Y * sensitivity );
  acceleration->AXIS_Z = ( int32_t )( dataRaw.AXIS_Z * sensitivity );

  return COMPONENT_OK;
}

/**
 * @brief Get the LIS2DW12 accelerometer sensor raw axes
 * @param handle the device handle
 * @param acceleration_raw pointer to where acceleration raw data write to
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Get_AxesRaw( DrvContextTypeDef *handle, SensorAxesRaw_t *value )
{
  uint8_t regValue[6] = { 0, 0, 0, 0, 0, 0 };
  int16_t dataRaw[3];
  LIS2DW12_ACC_MODE_t mode;
  LIS2DW12_ACC_LP_MODE_t lp_mode;

  /* Read raw data from LIS2DW12 output register. */
  if ( LIS2DW12_ACC_Get_Acceleration( handle, regValue ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  dataRaw[0] = ( ( ( ( int16_t )regValue[1] ) << 8 ) + ( int16_t )regValue[0] );
  dataRaw[1] = ( ( ( ( int16_t )regValue[3] ) << 8 ) + ( int16_t )regValue[2] );
  dataRaw[2] = ( ( ( ( int16_t )regValue[5] ) << 8 ) + ( int16_t )regValue[4] );

  /* Read actual power mode selection from sensor. */
  if ( LIS2DW12_ACC_R_ModeSelection( ( void * )handle, &mode ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }
  if ( LIS2DW12_ACC_R_LowPowerModeSelection( ( void * )handle, &lp_mode ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Select raw data format according to power mode selected. */
  if ( (mode == LIS2DW12_ACC_MODE_LOW_POWER_STD || mode == LIS2DW12_ACC_MODE_LOW_POWER_SINGLE)
      && lp_mode == LIS2DW12_ACC_LP_MODE1_12bit )
  {
    /* 12 bits left justified */
    dataRaw[0] >>= 4;
    dataRaw[1] >>= 4;
    dataRaw[2] >>= 4;
  }
  else
  {
    /* 14 bits left justified */
    dataRaw[0] >>= 2;
    dataRaw[1] >>= 2;
    dataRaw[2] >>= 2;
  }

  /* Set the raw data. */
  value->AXIS_X = dataRaw[0];
  value->AXIS_Y = dataRaw[1];
  value->AXIS_Z = dataRaw[2];

  return COMPONENT_OK;
}

/**
 * @brief Get the LIS2DW12 accelerometer sensor sensitivity
 * @param handle the device handle
 * @param sensitivity pointer to where sensitivity write to
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Get_Sensitivity( DrvContextTypeDef *handle, float *sensitivity )
{
  LIS2DW12_ACC_MODE_t mode;
  LIS2DW12_ACC_LP_MODE_t lp_mode;
  LIS2DW12_ACC_FS_t fullScale;

  /* Read actual power mode selection from sensor. */
  if ( LIS2DW12_ACC_R_ModeSelection( ( void * )handle, &mode ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }
  if ( LIS2DW12_ACC_R_LowPowerModeSelection( ( void * )handle, &lp_mode ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Read actual full scale selection from sensor. */
  if ( LIS2DW12_ACC_R_FullScaleSelection( ( void * )handle, &fullScale ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Store the sensitivity based on actual operating mode and full scale. */
  if ( (mode == LIS2DW12_ACC_MODE_LOW_POWER_STD || mode == LIS2DW12_ACC_MODE_LOW_POWER_SINGLE)
      && lp_mode == LIS2DW12_ACC_LP_MODE1_12bit )
  {
    switch ( fullScale )
    {
      case LIS2DW12_ACC_FS_2g:
        *sensitivity = ( float )LIS2DW12_ACC_SENSITIVITY_FOR_FS_2G_LOPOW1_MODE;
        break;
      case LIS2DW12_ACC_FS_4g:
        *sensitivity = ( float )LIS2DW12_ACC_SENSITIVITY_FOR_FS_4G_LOPOW1_MODE;
        break;
      case LIS2DW12_ACC_FS_8g:
        *sensitivity = ( float )LIS2DW12_ACC_SENSITIVITY_FOR_FS_8G_LOPOW1_MODE;
        break;
      case LIS2DW12_ACC_FS_16g:
        *sensitivity = ( float )LIS2DW12_ACC_SENSITIVITY_FOR_FS_16G_LOPOW1_MODE;
        break;
      default:
        *sensitivity = -1.0f;
        return COMPONENT_ERROR;
    }
  }
  else
  {
    switch ( fullScale )
    {
      case LIS2DW12_ACC_FS_2g:
        *sensitivity = ( float )LIS2DW12_ACC_SENSITIVITY_FOR_FS_2G_OTHER_MODES;
        break;
      case LIS2DW12_ACC_FS_4g:
        *sensitivity = ( float )LIS2DW12_ACC_SENSITIVITY_FOR_FS_4G_OTHER_MODES;
        break;
      case LIS2DW12_ACC_FS_8g:
        *sensitivity = ( float )LIS2DW12_ACC_SENSITIVITY_FOR_FS_8G_OTHER_MODES;
        break;
      case LIS2DW12_ACC_FS_16g:
        *sensitivity = ( float )LIS2DW12_ACC_SENSITIVITY_FOR_FS_16G_OTHER_MODES;
        break;
      default:
        *sensitivity = -1.0f;
        return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the LIS2DW12 accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr pointer to where output data rate write to
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Get_ODR( DrvContextTypeDef *handle, float *odr )
{
  LIS2DW12_ACC_MODE_t mode;
  LIS2DW12_ACC_ODR_t odr_low_level;

  /* Read actual power mode selection from sensor. */
  if ( LIS2DW12_ACC_R_ModeSelection( ( void * )handle, &mode ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Read actual output data rate selection from sensor. */
  if ( LIS2DW12_ACC_R_OutputDataRate( ( void * )handle, &odr_low_level ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch ( odr_low_level )
  {
  case LIS2DW12_ACC_ODR_POWER_DOWN:
    *odr = 0.0f;
    break;
  case LIS2DW12_ACC_ODR_LP_1Hz6_HP_12Hz5:
    switch ( mode )
    {
    case LIS2DW12_ACC_MODE_LOW_POWER_STD:
    case LIS2DW12_ACC_MODE_LOW_POWER_SINGLE:
      *odr = 1.6f;
      break;
    case LIS2DW12_ACC_MODE_HIGH_PERFORMANCE:
      *odr = 12.5f;
      break;
    default:
      *odr = -1.0f;
      return COMPONENT_ERROR;
    }
  case LIS2DW12_ACC_ODR_LP_12Hz5_HP_12Hz5:
    *odr = 12.5f;
    break;
  case LIS2DW12_ACC_ODR_LP_25Hz_HP_25Hz:
    *odr = 25.0f;
    break;
  case LIS2DW12_ACC_ODR_LP_50Hz_HP_50Hz:
    *odr = 50.0f;
    break;
  case LIS2DW12_ACC_ODR_LP_100Hz_HP_100Hz:
    *odr = 100.0f;
    break;
  case LIS2DW12_ACC_ODR_LP_200Hz_HP_200Hz:
    *odr = 200.0f;
    break;
  case LIS2DW12_ACC_ODR_LP_200Hz_HP_400Hz:
    switch ( mode )
    {
    case LIS2DW12_ACC_MODE_LOW_POWER_STD:
    case LIS2DW12_ACC_MODE_LOW_POWER_SINGLE:
      *odr = 200.0f;
      break;
    case LIS2DW12_ACC_MODE_HIGH_PERFORMANCE:
      *odr = 400.0f;
      break;
    default:
      *odr = -1.0f;
      return COMPONENT_ERROR;
    }
  case LIS2DW12_ACC_ODR_LP_200Hz_HP_800Hz:
    switch ( mode )
    {
    case LIS2DW12_ACC_MODE_LOW_POWER_STD:
    case LIS2DW12_ACC_MODE_LOW_POWER_SINGLE:
      *odr = 200.0f;
      break;
    case LIS2DW12_ACC_MODE_HIGH_PERFORMANCE:
      *odr = 800.0f;
      break;
    default:
      *odr = -1.0f;
      return COMPONENT_ERROR;
    }
  case LIS2DW12_ACC_ODR_LP_200Hz_HP_1600Hz:
    switch ( mode )
    {
    case LIS2DW12_ACC_MODE_LOW_POWER_STD:
    case LIS2DW12_ACC_MODE_LOW_POWER_SINGLE:
      *odr = 200.0f;
      break;
    case LIS2DW12_ACC_MODE_HIGH_PERFORMANCE:
      *odr = 1600.0f;
      break;
    default:
      *odr = -1.0f;
      return COMPONENT_ERROR;
    }
  default:
    *odr = -1.0f;
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LIS2DW12 accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr )
{
  if ( handle->isEnabled == 1 )
  {
    if ( LIS2DW12_Set_ODR_When_Enabled( handle, odr ) == COMPONENT_ERROR )
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if ( LIS2DW12_Set_ODR_When_Disabled( handle, odr ) == COMPONENT_ERROR )
    {
      return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LIS2DW12 accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Set_ODR_Value( DrvContextTypeDef *handle, float odr )
{
  if ( handle->isEnabled == 1 )
  {
    if( LIS2DW12_Set_ODR_Value_When_Enabled( handle, odr ) == COMPONENT_ERROR )
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if ( LIS2DW12_Set_ODR_Value_When_Disabled( handle, odr ) == COMPONENT_ERROR )
    {
      return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the LIS2DW12 accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale pointer to where full scale write to
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Get_FS( DrvContextTypeDef *handle, float *fullScale )
{
  LIS2DW12_ACC_FS_t fs_low_level;

  if ( LIS2DW12_ACC_R_FullScaleSelection( ( void* )handle, &fs_low_level ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( fs_low_level )
  {
    case LIS2DW12_ACC_FS_2g:
      *fullScale = 2.0f;
      break;
    case LIS2DW12_ACC_FS_4g:
      *fullScale = 4.0f;
      break;
    case LIS2DW12_ACC_FS_8g:
      *fullScale = 8.0f;
      break;
    case LIS2DW12_ACC_FS_16g:
      *fullScale = 16.0f;
      break;
    default:
      *fullScale = -1.0f;
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LIS2DW12 accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale the functional full scale to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Set_FS( DrvContextTypeDef *handle, SensorFs_t fullScale )
{
  LIS2DW12_ACC_FS_t new_fs;

  switch( fullScale )
  {

  case FS_LOW:
    new_fs = LIS2DW12_ACC_FS_2g;
    break;

  case FS_MID_LOW:
  case FS_MID:
    new_fs = LIS2DW12_ACC_FS_4g;
    break;

  case FS_MID_HIGH:
    new_fs = LIS2DW12_ACC_FS_8g;
    break;

  case FS_HIGH:
    new_fs = LIS2DW12_ACC_FS_16g;
    break;

  default:
    return COMPONENT_ERROR;
  }

  if ( LIS2DW12_ACC_W_FullScaleSelection( ( void* )handle, new_fs ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LIS2DW12 accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale the full scale value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Set_FS_Value( DrvContextTypeDef *handle, float fullScale )
{
  LIS2DW12_ACC_FS_t new_fs;

  new_fs = ( fullScale <= 2.0f ) ?  LIS2DW12_ACC_FS_2g
         : ( fullScale <= 4.0f ) ?  LIS2DW12_ACC_FS_4g
         : ( fullScale <= 8.0f ) ?  LIS2DW12_ACC_FS_8g
         :                          LIS2DW12_ACC_FS_16g;

  if ( LIS2DW12_ACC_W_FullScaleSelection( ( void* )handle, new_fs ) == MEMS_ERROR )
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
static DrvStatusTypeDef LIS2DW12_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{

  if ( LIS2DW12_ACC_ReadReg( (void *)handle, reg, data, 1 ) == MEMS_ERROR )
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
static DrvStatusTypeDef LIS2DW12_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( LIS2DW12_ACC_WriteReg( (void *)handle, reg, &data, 1 ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get data ready status
 * @param handle the device handle
 * @param status the data ready status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status )
{
  LIS2DW12_ACC_DRDY_t status_raw = LIS2DW12_ACC_DRDY_NOT_READY;

  if ( LIS2DW12_ACC_R_DataStatus( (void *)handle, &status_raw ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *status = (uint8_t)status_raw;

  return COMPONENT_OK;
}

/**
 * @}
 */

/** @addtogroup LIS2DW12_Private_Functions Private functions
 * @{
 */

/**
 * @brief Set the LIS2DW12 accelerometer sensor output data rate when enabled
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Set_ODR_When_Enabled( DrvContextTypeDef *handle, SensorOdr_t odr )
{
  LIS2DW12_ACC_ODR_t new_odr;

  switch( odr )
  {

  case ODR_LOW:
    new_odr = LIS2DW12_ACC_ODR_LP_1Hz6_HP_12Hz5;
    break;

  case ODR_MID_LOW:
    new_odr = LIS2DW12_ACC_ODR_LP_25Hz_HP_25Hz;
    break;

  case ODR_MID:
    new_odr = LIS2DW12_ACC_ODR_LP_50Hz_HP_50Hz;
    break;

  case ODR_MID_HIGH:
    new_odr = LIS2DW12_ACC_ODR_LP_100Hz_HP_100Hz;
    break;

  case ODR_HIGH:
    new_odr = LIS2DW12_ACC_ODR_LP_200Hz_HP_200Hz;
    break;

  default:
    return COMPONENT_ERROR;
  }

  if ( LIS2DW12_ACC_W_OutputDataRate( ( void * )handle, new_odr ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LIS2DW12 accelerometer sensor output data rate when disabled
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Set_ODR_When_Disabled( DrvContextTypeDef *handle, SensorOdr_t odr )
{
  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  LIS2DW12_Data_t *pComponentData = ( LIS2DW12_Data_t * )pData->pComponentData;

  switch( odr )
  {

  case ODR_LOW:
    pComponentData->Previous_ODR =  12.5f;
    break;

  case ODR_MID_LOW:
    pComponentData->Previous_ODR =  25.0f;
    break;

  case ODR_MID:
    pComponentData->Previous_ODR =  50.0f;
    break;

  case ODR_MID_HIGH:
    pComponentData->Previous_ODR = 100.0f;
    break;

  case ODR_HIGH:
    pComponentData->Previous_ODR = 200.0f;
    break;

  default:
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LIS2DW12 accelerometer sensor output data rate when enabled
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Set_ODR_Value_When_Enabled( DrvContextTypeDef *handle, float odr )
{
  LIS2DW12_ACC_ODR_t new_odr;

  new_odr = ( odr <=   1.6f ) ? LIS2DW12_ACC_ODR_LP_1Hz6_HP_12Hz5
          : ( odr <=  12.5f ) ? LIS2DW12_ACC_ODR_LP_12Hz5_HP_12Hz5
          : ( odr <=  25.0f ) ? LIS2DW12_ACC_ODR_LP_25Hz_HP_25Hz
          : ( odr <=  50.0f ) ? LIS2DW12_ACC_ODR_LP_50Hz_HP_50Hz
          : ( odr <= 100.0f ) ? LIS2DW12_ACC_ODR_LP_100Hz_HP_100Hz
          : ( odr <= 200.0f ) ? LIS2DW12_ACC_ODR_LP_200Hz_HP_200Hz
          : ( odr <= 400.0f ) ? LIS2DW12_ACC_ODR_LP_200Hz_HP_400Hz
          : ( odr <= 800.0f ) ? LIS2DW12_ACC_ODR_LP_200Hz_HP_800Hz
          :                     LIS2DW12_ACC_ODR_LP_200Hz_HP_1600Hz;

  if ( LIS2DW12_ACC_W_OutputDataRate( ( void* )handle, new_odr ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if ( odr <= 1.6f )
  {
    /* Set low-power mode for 1.6 Hz ODR */
    if ( LIS2DW12_ACC_W_ModeSelection( ( void * )handle, LIS2DW12_ACC_MODE_LOW_POWER_STD ) == MEMS_ERROR )
    {
      return COMPONENT_ERROR;
    }
  }

  if ( odr > 200.0f )
  {
    /* Set high-performance mode for ODR higher then 200 Hz */
    if ( LIS2DW12_ACC_W_ModeSelection( ( void * )handle, LIS2DW12_ACC_MODE_HIGH_PERFORMANCE ) == MEMS_ERROR )
    {
      return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LIS2DW12 accelerometer sensor output data rate when disabled
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Set_ODR_Value_When_Disabled( DrvContextTypeDef *handle, float odr )
{
  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  LIS2DW12_Data_t *pComponentData = ( LIS2DW12_Data_t * )pData->pComponentData;

  pComponentData->Previous_ODR = ( odr <=   1.6f ) ?    1.6f
                               : ( odr <=  12.5f ) ?   12.5f
                               : ( odr <=  25.0f ) ?   25.0f
                               : ( odr <=  50.0f ) ?   50.0f
                               : ( odr <= 100.0f ) ?  100.0f
                               : ( odr <= 200.0f ) ?  200.0f
                               : ( odr <= 400.0f ) ?  400.0f
                               : ( odr <= 800.0f ) ?  800.0f
                               :                     1600.0f;

  return COMPONENT_OK;
}




/**
 * @brief Read the ALL_INT_SRC register in LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @param value the pointer where to save the ALL_INT_SRC register value
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_ReadAllInterruptSrcRegister(DrvContextTypeDef *handle, uint8_t *value)
{
    if(LIS2DW12_ACC_R_All_Int_Src((void*)handle, value) == MEMS_ERROR){
        return COMPONENT_ERROR;
    }
    return COMPONENT_OK;
}




/**
 * @brief Enable the free fall detection for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @note  This function sets the LIS2DW12 accelerometer ODR to 50Hz and the LIS2DW12 accelerometer full scale to 2g
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Enable_Free_Fall_Detection(DrvContextTypeDef *handle)
{
  /* Output Data Rate selection */
  if(LIS2DW12_Set_ODR_Value(handle, 50.0f) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Set LOW_POWER mode. */
  if(LIS2DW12_ACC_W_ModeSelection((void*)handle, LIS2DW12_ACC_MODE_LOW_POWER_STD) == MEMS_ERROR){
    return COMPONENT_ERROR;
  }
  
  /* Full scale selection */
  if (LIS2DW12_ACC_W_FullScaleSelection((void*)handle, LIS2DW12_ACC_FS_2g) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Set event duration (FF_DUR[5] = 0) */
  if (LIS2DW12_ACC_W_FreeFallDuration((void*)handle, 0x00) == MEMS_ERROR)
  {
    return COMPONENT_ERROR;
  }
  
  /*  Set event duration (FF_DUR[4:0] = 01111b) */
  if (LIS2DW12_ACC_W_LIS2DW12_ACC_W_FreeFallDuration((void *)handle, 0x08) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* FF_THS setting */
  if (LIS2DW12_Set_Free_Fall_Threshold(handle, LIS2DW12_ACC_FF_THS_7) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* INT1_FF setting */
  if(LIS2DW12_ACC_W_PinFunction_INT1((void *)handle, LIS2DW12_ACC_INT1_MODE_FREE_FALL, 1) == MEMS_ERROR)
  {
    return COMPONENT_ERROR;
  }
  
  /* @Note The code does not work properly in multiple event mode if latching interrupt. */
//  // Latch interrupt
//  if(LIS2DW12_ACC_W_LatchIntteruptRq((void *)handle, LIS2DW12_ACC_LIR_ENABLE) == MEMS_ERROR){
//    return COMPONENT_ERROR;
//  }
  
  // Enable interrupts
  if(LIS2DW12_ACC_W_HardwarePin((void *)handle, LIS2DW12_ACC_INTERRUPTS_ENABLE_ENABLE) == MEMS_ERROR){
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Disable the free fall detection for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Disable_Free_Fall_Detection(DrvContextTypeDef *handle)
{
  /* INT1_FF setting */
  if(LIS2DW12_ACC_W_PinFunction_INT1((void *)handle, LIS2DW12_ACC_INT1_MODE_FREE_FALL, 0) == MEMS_ERROR)
  {
    return COMPONENT_ERROR;
  }
  
  /* FF_DUR setting */
  if(LIS2DW12_ACC_W_FreeFallDuration((void*)handle, 0x00) == MEMS_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* FF_THS setting */
  if(LIS2DW12_ACC_W_FreeFallThreshold((void*)handle, LIS2DW12_ACC_FF_THS_5) == MEMS_ERROR)
  {
    return COMPONENT_ERROR;
  }

  // Disable interrupts
  if(LIS2DW12_ACC_W_HardwarePin((void *)handle, LIS2DW12_ACC_INTERRUPTS_ENABLE_DISABLE) == MEMS_ERROR){
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}

/**
 * @brief Get the status of the free fall detection for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @param status the pointer to the status of free fall detection: 0 means no detection, 1 means detection happened
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Get_Free_Fall_Detection_Status(DrvContextTypeDef *handle, uint8_t *status)
{

  LIS2DW12_ACC_FF_IA_t free_fall_status;

  if (LIS2DW12_ACC_R_FreeFall_Event((void*)handle, &free_fall_status) == MEMS_ERROR)
  {
    return COMPONENT_ERROR;
  }

  switch(free_fall_status)
  {
    case LIS2DW12_ACC_FF_IA_DETECTED:
      *status = 1;
      break;
    case LIS2DW12_ACC_FF_IA_NOT_DETECTED:
      *status = 0;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the free fall detection threshold for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @param thr the threshold to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Set_Free_Fall_Threshold( DrvContextTypeDef *handle, uint8_t thr )
{
    if(LIS2DW12_ACC_W_FreeFallThreshold((void *)handle, thr) == MEMS_ERROR)
    {
        return COMPONENT_ERROR;
    }
    return COMPONENT_OK;
}




/**
 * @brief Enable the wake up detection for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @note  This function sets the LIS2DW12 accelerometer ODR to 50Hz and the LIS2DW12 accelerometer full scale to 2g
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Enable_Wake_Up_Detection(DrvContextTypeDef *handle)
{
    /* Output Data Rate selection */
    if(LIS2DW12_Set_ODR_Value(handle, 50.0f) == COMPONENT_ERROR){
        goto fail;
    }
    
    /* Set LOW_POWER mode. */
    if(LIS2DW12_ACC_W_ModeSelection((void*)handle, LIS2DW12_ACC_MODE_LOW_POWER_STD) == MEMS_ERROR){
        goto fail;
    }

    /* Full scale selection */
    if(LIS2DW12_ACC_W_FullScaleSelection((void*)handle, LIS2DW12_ACC_FS_2g) == MEMS_ERROR){
        goto fail;
    }
    
    /* Set Wake Up Duration */
    if(LIS2DW12_ACC_W_WakeUpDuration((void*)handle, 0x01) == MEMS_ERROR){
        goto fail;
    }
    
    /* Set Wake Up Threshold */
    if(LIS2DW12_ACC_W_WakeUpThreshold((void*)handle, 0x0F) == MEMS_ERROR){
        goto fail;
    }
    
    /* Wake-up interrupt driven to INT1 pin. */
    if(LIS2DW12_ACC_W_PinFunction_INT1((void*)handle, LIS2DW12_ACC_INT1_MODE_WAKE_UP, 1) == MEMS_ERROR){
        goto fail;
    }
    
    /* @Note The code does not work properly in multiple event mode if latching interrupt. */
//    /* Latch interrupt. */
//    if(LIS2DW12_ACC_W_LatchIntteruptRq((void *)handle, LIS2DW12_ACC_LIR_ENABLE) == MEMS_ERROR){
//        goto fail;
//    }

    /* Enable interrupts. */
    if(LIS2DW12_ACC_W_HardwarePin((void *)handle, LIS2DW12_ACC_INTERRUPTS_ENABLE_ENABLE) == MEMS_ERROR){
        goto fail;
    }
    
    return COMPONENT_OK;
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Disable the wake up detection for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Disable_Wake_Up_Detection(DrvContextTypeDef *handle)
{
    /* Disable Wake-up interrupt on INT1 pin. */
    if(LIS2DW12_ACC_W_PinFunction_INT1((void*)handle, LIS2DW12_ACC_INT1_MODE_WAKE_UP, 0) == MEMS_ERROR){
        goto fail;
    }
    
    /* Reset Wake Up Duration */
    if(LIS2DW12_ACC_W_WakeUpDuration((void*)handle, 0x00) == MEMS_ERROR){
        goto fail;
    }
    
    /* Reset Wake Up Threshold */
    if(LIS2DW12_ACC_W_WakeUpThreshold((void*)handle, 0x00) == MEMS_ERROR){
        goto fail;
    }
    
    // Disable interrupts
    if(LIS2DW12_ACC_W_HardwarePin((void *)handle, LIS2DW12_ACC_INTERRUPTS_ENABLE_DISABLE) == MEMS_ERROR){
        return COMPONENT_ERROR;
    }
    
    return COMPONENT_OK;
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Get the status of the wake up detection for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @param status the pointer to the status of the wake up detection: 0 means no detection, 1 means detection happened
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Get_Wake_Up_Detection_Status(DrvContextTypeDef *handle, uint8_t *status)
{
    LIS2DW12_ACC_WU_IA_t wakeUpStatus;

    if (LIS2DW12_ACC_R_WakeUp_Event((void*)handle, &wakeUpStatus) == MEMS_ERROR){
        goto fail;
    }

    switch(wakeUpStatus){
    case LIS2DW12_ACC_WU_IA_DETECTED:
        *status = 1;
        break;
    case LIS2DW12_ACC_WU_IA_NOT_DETECTED:
        *status = 0;
        break;
    default:
        goto fail;
    }

    return COMPONENT_OK;
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Set the wake up threshold for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @param thr the threshold to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Set_Wake_Up_Threshold(DrvContextTypeDef *handle, uint8_t thr)
{
    if(LIS2DW12_ACC_W_WakeUpThreshold((void*)handle, thr) == MEMS_ERROR){
        return COMPONENT_ERROR;
    }
    
    return COMPONENT_OK;
}




/**
 * @brief Enable the single tap detection for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @note  This function sets the LIS2DW12 accelerometer ODR to 50Hz and the LIS2DW12 accelerometer full scale to 2g
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Enable_Single_Tap_Detection(DrvContextTypeDef *handle)
{
    /* Output Data Rate selection */
    if(LIS2DW12_Set_ODR_Value(handle, 50.0f) == COMPONENT_ERROR){
        goto fail;
    }
    
    /* Set LOW_POWER mode. */
    if(LIS2DW12_ACC_W_ModeSelection((void*)handle, LIS2DW12_ACC_MODE_LOW_POWER_STD) == MEMS_ERROR){
        goto fail;
    }

    /* Full scale selection */
    if(LIS2DW12_ACC_W_FullScaleSelection((void*)handle, LIS2DW12_ACC_FS_2g) == MEMS_ERROR){
        goto fail;
    }
    
    /* Set tap threshold for X axis. */
    if(LIS2DW12_ACC_W_TAP_X_Threshold((void*)handle, 0x0C) == MEMS_ERROR){
        goto fail;
    }
    
    /* Set tap threshold for Y axis. */
    if(LIS2DW12_ACC_W_TAP_Y_Threshold((void*)handle, 0x0C) == MEMS_ERROR){
        goto fail;
    }
    
    /* Set TAP priority Z-Y-X. */
    if(LIS2DW12_ACC_W_TAP_PriorityAxisTreshold((void*)handle, LIS2DW12_ACC_TAP_PRIOR_ZYX) == MEMS_ERROR){
        goto fail;
    }
    
    /* Set tap threshold for Z-axis. */
    if(LIS2DW12_ACC_W_TAP_Z_Threshold((void*)handle, 0x0C) == MEMS_ERROR){
        goto fail;
    }
    
    /* Enable tap detection on Z-axis. */
    if(LIS2DW12_ACC_W_DetectTAP_on_Z_Axis((void*)handle, LIS2DW12_ACC_TAP_Z_EN_ENABLE) == MEMS_ERROR){
        goto fail;
    }
    
    /* Enable tap detection on Y-axis. */
    if(LIS2DW12_ACC_W_DetectTAP_on_Y_Axis((void*)handle, LIS2DW12_ACC_TAP_Y_EN_ENABLE) == MEMS_ERROR){
        goto fail;
    }
    
    /* Enable tap detection on X-axis. */
    if(LIS2DW12_ACC_W_DetectTAP_on_X_Axis((void*)handle, LIS2DW12_ACC_TAP_X_EN_ENABLE) == MEMS_ERROR){
        goto fail;
    }
    
    /* Enable Single Tap  */
    if(LIS2DW12_ACC_W_TAP_mode((void*)handle, LIS2DW12_ACC_SINGLE_DOUBLE_TAP_SINGLE) == MEMS_ERROR){
        goto fail;
    }
    
    /* Single-tap interrupt driven to INT1 pin. */
    if(LIS2DW12_ACC_W_PinFunction_INT1((void*)handle, LIS2DW12_ACC_INT1_MODE_SINGLE_TAP, 1) == MEMS_ERROR){
        goto fail;
    }
    
    /* @Note The code does not work properly in multiple event mode if latching interrupt. */
//    /* Latch interrupt. */
//    if(LIS2DW12_ACC_W_LatchIntteruptRq((void *)handle, LIS2DW12_ACC_LIR_ENABLE) == MEMS_ERROR){
//        goto fail;
//    }

    /* Enable interrupts. */
    if(LIS2DW12_ACC_W_HardwarePin((void *)handle, LIS2DW12_ACC_INTERRUPTS_ENABLE_ENABLE) == MEMS_ERROR){
        goto fail;
    }
    
    return COMPONENT_OK;
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Disable the single tap detection for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Disable_Single_Tap_Detection( DrvContextTypeDef *handle )
{
    /* Disable Single-tap interrupt on INT1 pin. */
    if(LIS2DW12_ACC_W_PinFunction_INT1((void*)handle, LIS2DW12_ACC_INT1_MODE_SINGLE_TAP, 0) == MEMS_ERROR){
        goto fail;
    }
    
    /* Reset tap threshold for X axis. */
    if(LIS2DW12_ACC_W_TAP_X_Threshold((void*)handle, 0x00) == MEMS_ERROR){
        goto fail;
    }

    /* Reset tap threshold for Y axis. */
    if(LIS2DW12_ACC_W_TAP_Y_Threshold((void*)handle, 0x00) == MEMS_ERROR){
        goto fail;
    }
    
    /* Reset tap threshold for Z-axis. */
    if(LIS2DW12_ACC_W_TAP_Z_Threshold((void*)handle, 0x00) == MEMS_ERROR){
        goto fail;
    }

    /* Disable tap detection on Z-axis. */
    if(LIS2DW12_ACC_W_DetectTAP_on_Z_Axis((void*)handle, LIS2DW12_ACC_TAP_Z_EN_DISABLE) == MEMS_ERROR){
        goto fail;
    }
    
    /* Disable tap detection on Y-axis. */
    if(LIS2DW12_ACC_W_DetectTAP_on_Y_Axis((void*)handle, LIS2DW12_ACC_TAP_Y_EN_DISABLE) == MEMS_ERROR){
        goto fail;
    }
    
    /* Disable tap detection on X-axis. */
    if(LIS2DW12_ACC_W_DetectTAP_on_X_Axis((void*)handle, LIS2DW12_ACC_TAP_X_EN_DISABLE) == MEMS_ERROR){
        goto fail;
    }
    
    // Disable interrupts
    if(LIS2DW12_ACC_W_HardwarePin((void *)handle, LIS2DW12_ACC_INTERRUPTS_ENABLE_DISABLE) == MEMS_ERROR){
        return COMPONENT_ERROR;
    }
    
    return COMPONENT_OK;
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Get the single tap detection status for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @param status the pointer to the single tap detection status: 0 means no single tap detected, 1 means single tap detected
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Get_Single_Tap_Detection_Status( DrvContextTypeDef *handle, uint8_t *status )
{
    LIS2DW12_ACC_SINGLE_TAP_t tapStatus;

    if (LIS2DW12_ACC_R_SingleTap_Event((void*)handle, &tapStatus) == MEMS_ERROR){
        goto fail;
    }

    switch(tapStatus){
    case LIS2DW12_ACC_SINGLE_TAP_DETECTED:
        *status = 1;
        break;
    case LIS2DW12_ACC_SINGLE_TAP_NOT_DETECTED:
        *status = 0;
        break;
    default:
        goto fail;
    }

    return COMPONENT_OK;
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Enable the double tap detection for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @note  This function sets the LIS2DW12 accelerometer ODR to 50Hz and the LIS2DW12 accelerometer full scale to 2g
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Enable_Double_Tap_Detection( DrvContextTypeDef *handle )
{
    /* Output Data Rate selection */
    if(LIS2DW12_Set_ODR_Value(handle, 50.0f) == COMPONENT_ERROR){
        goto fail;
    }

    /* Set LOW_POWER mode. */
    if(LIS2DW12_ACC_W_ModeSelection((void*)handle, LIS2DW12_ACC_MODE_LOW_POWER_STD) == MEMS_ERROR){
        goto fail;
    }
    
    /* Full scale selection */
    if(LIS2DW12_ACC_W_FullScaleSelection((void*)handle, LIS2DW12_ACC_FS_2g) == MEMS_ERROR){
        goto fail;
    }
    
    /* Set tap threshold for X axis. */
    if(LIS2DW12_ACC_W_TAP_X_Threshold((void*)handle, 0x0C) == MEMS_ERROR){
        goto fail;
    }
    
    /* Set tap threshold for Y axis. */
    if(LIS2DW12_ACC_W_TAP_Y_Threshold((void*)handle, 0x0C) == MEMS_ERROR){
        goto fail;
    }
    
    /* Set TAP priority Z-Y-X. */
    if(LIS2DW12_ACC_W_TAP_PriorityAxisTreshold((void*)handle, LIS2DW12_ACC_TAP_PRIOR_ZYX) == MEMS_ERROR){
        goto fail;
    }
    
    /* Set tap threshold for Z-axis. */
    if(LIS2DW12_ACC_W_TAP_Z_Threshold((void*)handle, 0x0C) == MEMS_ERROR){
        goto fail;
    }
    
    /* Enable tap detection on Z-axis. */
    if(LIS2DW12_ACC_W_DetectTAP_on_Z_Axis((void*)handle, LIS2DW12_ACC_TAP_Z_EN_ENABLE) == MEMS_ERROR){
        goto fail;
    }
    
    /* Enable tap detection on Y-axis. */
    if(LIS2DW12_ACC_W_DetectTAP_on_Y_Axis((void*)handle, LIS2DW12_ACC_TAP_Y_EN_ENABLE) == MEMS_ERROR){
        goto fail;
    }
    
    /* Enable tap detection on X-axis. */
    if(LIS2DW12_ACC_W_DetectTAP_on_X_Axis((void*)handle, LIS2DW12_ACC_TAP_X_EN_ENABLE) == MEMS_ERROR){
        goto fail;
    }
    
    /* Set shock time window. */
    if(LIS2DW12_ACC_W_MaxDurationOverThreshold((void*)handle, 0x01) == MEMS_ERROR){
        goto fail;
    }
    
    /* Set quiet time window. */
    if(LIS2DW12_ACC_W_QuietTimeAfterTAP((void*)handle, 0x01) == MEMS_ERROR){
        goto fail;
    }
    
    /* Set duration time window. */
    if(LIS2DW12_ACC_W_DoubleTAP_Latency((void*)handle, 0x02) == MEMS_ERROR){
        goto fail;
    }
    
    /* Enable Double Tap  */
    if(LIS2DW12_ACC_W_TAP_mode((void*)handle, LIS2DW12_ACC_SINGLE_DOUBLE_TAP_DOUBLE) == MEMS_ERROR){
        goto fail;
    }
    
    /* Double-tap interrupt driven to INT1 pin. */
    if(LIS2DW12_ACC_W_PinFunction_INT1((void*)handle, LIS2DW12_ACC_INT1_MODE_DOUBLE_TAP, 1) == MEMS_ERROR){
        goto fail;
    }
    
    /* @Note The code does not work properly in multiple event mode if latching interrupt. */
//    /* Latch interrupt. */
//    if(LIS2DW12_ACC_W_LatchIntteruptRq((void *)handle, LIS2DW12_ACC_LIR_ENABLE) == MEMS_ERROR){
//        goto fail;
//    }

    /* Enable interrupts. */
    if(LIS2DW12_ACC_W_HardwarePin((void *)handle, LIS2DW12_ACC_INTERRUPTS_ENABLE_ENABLE) == MEMS_ERROR){
        goto fail;
    }
    
    return COMPONENT_OK;
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Disable the double tap detection for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Disable_Double_Tap_Detection( DrvContextTypeDef *handle )
{
    /* Disable Double-tap interrupt on INT1 pin. */
    if(LIS2DW12_ACC_W_PinFunction_INT1((void*)handle, LIS2DW12_ACC_INT1_MODE_DOUBLE_TAP, 0) == MEMS_ERROR){
        goto fail;
    }
    
    /* Enable only Single Tap  */
    if(LIS2DW12_ACC_W_TAP_mode((void*)handle, LIS2DW12_ACC_SINGLE_DOUBLE_TAP_SINGLE) == MEMS_ERROR){
        goto fail;
    }
    
    /* Reset tap threshold for X axis. */
    if(LIS2DW12_ACC_W_TAP_X_Threshold((void*)handle, 0x00) == MEMS_ERROR){
        goto fail;
    }

    /* Reset tap threshold for Y axis. */
    if(LIS2DW12_ACC_W_TAP_Y_Threshold((void*)handle, 0x00) == MEMS_ERROR){
        goto fail;
    }
    
    /* Reset tap threshold for Z-axis. */
    if(LIS2DW12_ACC_W_TAP_Z_Threshold((void*)handle, 0x00) == MEMS_ERROR){
        goto fail;
    }
    
    /* Reset shock time window. */
    if(LIS2DW12_ACC_W_MaxDurationOverThreshold((void*)handle, 0x00) == MEMS_ERROR){
        goto fail;
    }
    
    /* Reset quiet time window. */
    if(LIS2DW12_ACC_W_QuietTimeAfterTAP((void*)handle, 0x00) == MEMS_ERROR){
        goto fail;
    }
    
    /* Reset duration time window. */
    if(LIS2DW12_ACC_W_DoubleTAP_Latency((void*)handle, 0x00) == MEMS_ERROR){
        goto fail;
    }

    /* Disable tap detection on Z-axis. */
    if(LIS2DW12_ACC_W_DetectTAP_on_Z_Axis((void*)handle, LIS2DW12_ACC_TAP_Z_EN_DISABLE) == MEMS_ERROR){
        goto fail;
    }
    
    /* Disable tap detection on Y-axis. */
    if(LIS2DW12_ACC_W_DetectTAP_on_Y_Axis((void*)handle, LIS2DW12_ACC_TAP_Y_EN_DISABLE) == MEMS_ERROR){
        goto fail;
    }
    
    /* Disable tap detection on X-axis. */
    if(LIS2DW12_ACC_W_DetectTAP_on_X_Axis((void*)handle, LIS2DW12_ACC_TAP_X_EN_DISABLE) == MEMS_ERROR){
        goto fail;
    }
    
    // Disable interrupts
    if(LIS2DW12_ACC_W_HardwarePin((void *)handle, LIS2DW12_ACC_INTERRUPTS_ENABLE_DISABLE) == MEMS_ERROR){
        return COMPONENT_ERROR;
    }
    
    return COMPONENT_OK;
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Get the double tap detection status for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @param status the pointer to the double tap detection status: 0 means no double tap detected, 1 means double tap detected
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Get_Double_Tap_Detection_Status( DrvContextTypeDef *handle, uint8_t *status )
{
    LIS2DW12_ACC_DOUBLE_TAP_t tapStatus;

    if (LIS2DW12_ACC_R_DoubleTap_Event((void*)handle, &tapStatus) == MEMS_ERROR){
        goto fail;
    }

    switch(tapStatus){
    case LIS2DW12_ACC_DOUBLE_TAP_DETECTED:
        *status = 1;
        break;
    case LIS2DW12_ACC_DOUBLE_TAP_NOT_DETECTED:
        *status = 0;
        break;
    default:
        goto fail;
    }

    return COMPONENT_OK;
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Set the tap threshold for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @param thr the threshold to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Set_Tap_Threshold(DrvContextTypeDef *handle, uint8_t thr)
{
    /* Set tap threshold for X axis. */
    if(LIS2DW12_ACC_W_TAP_X_Threshold((void*)handle, thr) == MEMS_ERROR){
        goto fail;
    }
    
    /* Set tap threshold for Y axis. */
    if(LIS2DW12_ACC_W_TAP_Y_Threshold((void*)handle, thr) == MEMS_ERROR){
        goto fail;
    }
    
    /* Set tap threshold for Z-axis. */
    if(LIS2DW12_ACC_W_TAP_Z_Threshold((void*)handle, thr) == MEMS_ERROR){
        goto fail;
    }
    
    return COMPONENT_OK;
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Set the tap shock time window for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @param time the shock time window to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Set_Tap_Shock_Time(DrvContextTypeDef *handle, uint8_t time)
{
    if(LIS2DW12_ACC_W_MaxDurationOverThreshold((void*)handle, time) == MEMS_ERROR){
        return COMPONENT_ERROR;
    }
    return COMPONENT_OK;
}

/**
 * @brief Set the tap quiet time window for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @param time the quiet time window to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Set_Tap_Quiet_Time(DrvContextTypeDef *handle, uint8_t time)
{
    if(LIS2DW12_ACC_W_QuietTimeAfterTAP((void*)handle, time) == MEMS_ERROR){
        return COMPONENT_ERROR;
    }
    return COMPONENT_OK;
}

/**
 * @brief Set the tap duration of the time window for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @param time the duration of the time window to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Set_Tap_Duration_Time(DrvContextTypeDef *handle, uint8_t time)
{
    if(LIS2DW12_ACC_W_DoubleTAP_Latency((void*)handle, time) == MEMS_ERROR){
        return COMPONENT_ERROR;
    }
    return COMPONENT_OK;
}




/**
 * @brief Enable the 6D orientation detection for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @note  This function sets the LIS2DW12 accelerometer ODR to 50Hz and the LIS2DW12 accelerometer full scale to 2g
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Enable_6D_Orientation(DrvContextTypeDef *handle)
{
    /* Output Data Rate selection. */
    if(LIS2DW12_Set_ODR_Value(handle, 50.0f) == COMPONENT_ERROR){
        goto fail;
    }

    /* Set LOW_POWER mode. */
    if(LIS2DW12_ACC_W_ModeSelection((void*)handle, LIS2DW12_ACC_MODE_LOW_POWER_STD) == MEMS_ERROR){
        goto fail;
    }
    
    /* Full scale selection. */
    if(LIS2DW12_ACC_W_FullScaleSelection((void*)handle, LIS2DW12_ACC_FS_2g) == MEMS_ERROR){
        goto fail;
    }
    
    /* Set 6D threshold. */
    if(LIS2DW12_ACC_W_6D_Threshold((void*)handle, LIS2DW12_ACC_6D_THS_50deg) == MEMS_ERROR){
        goto fail;
    }
    
    /* D6D interrupt driven to INT1 pin. */
    if(LIS2DW12_ACC_W_PinFunction_INT1((void*)handle, LIS2DW12_ACC_INT1_MODE_6D, 1) == MEMS_ERROR){
        goto fail;
    }
    
    /* @Note The code does not work properly in multiple event mode if latching interrupt. */
//    /* Latch interrupt. */
//    if(LIS2DW12_ACC_W_LatchIntteruptRq((void *)handle, LIS2DW12_ACC_LIR_ENABLE) == MEMS_ERROR){
//        goto fail;
//    }

    /* Enable interrupts. */
    if(LIS2DW12_ACC_W_HardwarePin((void *)handle, LIS2DW12_ACC_INTERRUPTS_ENABLE_ENABLE) == MEMS_ERROR){
        goto fail;
    }
    
    return COMPONENT_OK;
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Disable the 6D orientation detection for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Disable_6D_Orientation(DrvContextTypeDef *handle)
{
    /* Disable 6D interrupt on INT1 pin. */
    if(LIS2DW12_ACC_W_PinFunction_INT1((void*)handle, LIS2DW12_ACC_INT1_MODE_6D, 0) == MEMS_ERROR){
        goto fail;
    }
    
    /* Reset 6D threshold. */
    if(LIS2DW12_ACC_W_6D_Threshold((void*)handle, LIS2DW12_ACC_6D_THS_80deg) == MEMS_ERROR){
        goto fail;
    }
    
    // Disable interrupts
    if(LIS2DW12_ACC_W_HardwarePin((void *)handle, LIS2DW12_ACC_INTERRUPTS_ENABLE_DISABLE) == MEMS_ERROR){
        return COMPONENT_ERROR;
    }
    
    return COMPONENT_OK;
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Get the status of the 6D orientation detection for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @param status the pointer to the status of the 6D orientation detection: 0 means no detection, 1 means detection happened
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Get_6D_Orientation_Status(DrvContextTypeDef *handle, uint8_t *status)
{
    LIS2DW12_ACC_6D_IA_t dimentionStatus;

    if (LIS2DW12_ACC_R_6D_Event((void*)handle, &dimentionStatus) == MEMS_ERROR){
        goto fail;
    }

    switch(dimentionStatus){
    case LIS2DW12_ACC_6D_IA_DETECTED:
        *status = 1;
        break;
    case LIS2DW12_ACC_6D_IA_NOT_DETECTED:
        *status = 0;
        break;
    default:
        goto fail;
    }

    return COMPONENT_OK;
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Get the 6D orientation XL axis for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @param xl the pointer to the 6D orientation XL axis
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Get_6D_Orientation_XL(DrvContextTypeDef *handle, uint8_t *xl)
{
    LIS2DW12_ACC_XL_t axisStatus;

    if (LIS2DW12_ACC_R_6D_X_Low((void*)handle, &axisStatus) == MEMS_ERROR){
        goto fail;
    }

    switch(axisStatus){
    case LIS2DW12_ACC_XL_OVER_THRESHOLD:
        *xl = 1;
        break;
    case LIS2DW12_ACC_XL_UNDER_THRESHOLD:
        *xl = 0;
        break;
    default:
        goto fail;
    }

    return COMPONENT_OK;
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Get the 6D orientation XH axis for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @param xh the pointer to the 6D orientation XH axis
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Get_6D_Orientation_XH(DrvContextTypeDef *handle, uint8_t *xh)
{
    LIS2DW12_ACC_XH_t axisStatus;

    if (LIS2DW12_ACC_R_6D_X_High((void*)handle, &axisStatus) == MEMS_ERROR){
        goto fail;
    }

    switch(axisStatus){
    case LIS2DW12_ACC_XH_OVER_THRESHOLD:
        *xh = 1;
        break;
    case LIS2DW12_ACC_XH_UNDER_THRESHOLD:
        *xh = 0;
        break;
    default:
        goto fail;
    }

    return COMPONENT_OK;
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Get the 6D orientation YL axis for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @param yl the pointer to the 6D orientation YL axis
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Get_6D_Orientation_YL(DrvContextTypeDef *handle, uint8_t *yl)
{
    LIS2DW12_ACC_YL_t axisStatus;

    if (LIS2DW12_ACC_R_6D_Y_Low((void*)handle, &axisStatus) == MEMS_ERROR){
        goto fail;
    }

    switch(axisStatus){
    case LIS2DW12_ACC_YL_OVER_THRESHOLD:
        *yl = 1;
        break;
    case LIS2DW12_ACC_YL_UNDER_THRESHOLD:
        *yl = 0;
        break;
    default:
        goto fail;
    }

    return COMPONENT_OK;
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Get the 6D orientation YH axis for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @param yh the pointer to the 6D orientation YH axis
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Get_6D_Orientation_YH(DrvContextTypeDef *handle, uint8_t *yh)
{
    LIS2DW12_ACC_YH_t axisStatus;

    if (LIS2DW12_ACC_R_6D_Y_High((void*)handle, &axisStatus) == MEMS_ERROR){
        goto fail;
    }

    switch(axisStatus){
    case LIS2DW12_ACC_YH_OVER_THRESHOLD:
        *yh = 1;
        break;
    case LIS2DW12_ACC_YH_UNDER_THRESHOLD:
        *yh = 0;
        break;
    default:
        goto fail;
    }

    return COMPONENT_OK;
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Get the 6D orientation ZL axis for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @param zl the pointer to the 6D orientation ZL axis
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Get_6D_Orientation_ZL(DrvContextTypeDef *handle, uint8_t *zl)
{
    LIS2DW12_ACC_ZL_t axisStatus;

    if (LIS2DW12_ACC_R_6D_Z_Low((void*)handle, &axisStatus) == MEMS_ERROR){
        goto fail;
    }

    switch(axisStatus){
    case LIS2DW12_ACC_ZL_OVER_THRESHOLD:
        *zl = 1;
        break;
    case LIS2DW12_ACC_ZL_UNDER_THRESHOLD:
        *zl = 0;
        break;
    default:
        goto fail;
    }

    return COMPONENT_OK;
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Get the 6D orientation ZH axis for LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @param zh the pointer to the 6D orientation ZH axis
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LIS2DW12_Get_6D_Orientation_ZH(DrvContextTypeDef *handle, uint8_t *zh)
{
    LIS2DW12_ACC_ZH_t axisStatus;

    if (LIS2DW12_ACC_R_6D_Z_High((void*)handle, &axisStatus) == MEMS_ERROR){
        goto fail;
    }

    switch(axisStatus){
    case LIS2DW12_ACC_ZH_OVER_THRESHOLD:
        *zh = 1;
        break;
    case LIS2DW12_ACC_ZH_UNDER_THRESHOLD:
        *zh = 0;
        break;
    default:
        goto fail;
    }

    return COMPONENT_OK;
    
fail: return COMPONENT_ERROR;
}




/**
 * @brief Set FIFO output data rate
 * @param handle the device handle
 * @param odr Output data rate
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LIS2DW12_FIFO_Set_ODR_Value(DrvContextTypeDef*handle, float odr)
{
    /* Output Data Rate selection. */
    return LIS2DW12_Set_ODR_Value(handle, odr);
}

/**
 * @brief Get status of FIFO full flag
 * @param handle the device handle
 * @param *status Pointer where the status is stored. Values:
 *        0 ... no detection
 *        1 ... detection happened
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LIS2DW12_FIFO_Get_Full_Status(DrvContextTypeDef*handle, uint8_t *status)
{
    LIS2DW12_ACC_FIFO_THS_t tapStatus;

    if (LIS2DW12_ACC_R_FIFO_ThresholdStatus((void*)handle, &tapStatus) == MEMS_ERROR){
        goto fail;
    }

    switch(tapStatus){
    case LIS2DW12_ACC_FIFO_THS_LEVEL_REACHED:
        *status = 1;
        break;
    case LIS2DW12_ACC_FIFO_THS_BELOW_LEVEL:
        *status = 0;
        break;
    default:
        goto fail;
    }

    return COMPONENT_OK;
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Get status of FIFO empty flag
 * @param handle the device handle
 * @param *status The pointer where the status of FIFO is stored. Values:
 *        0 ... FIFO is not empty
 *        1 ... FIFO is empty
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LIS2DW12_FIFO_Get_Empty_Status(DrvContextTypeDef *handle, uint8_t *status)
{
    uint8_t fifoLevel = 0;
    if(LIS2DW12_ACC_R_FIFO_level((void*)handle, &fifoLevel) == MEMS_ERROR){
        goto fail;
    }
    
    if(fifoLevel == 0){
        *status = 1;
    } else {
        *status = 0;
    }
    
    return COMPONENT_OK;
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Get status of FIFO_OVR flag
 * @param handle the device handle
 * @param *status The pointer where the status of FIFO is stored. Values:
 *        0 ... no sample overrun
 *        1 ... at least 1 sample overrun
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LIS2DW12_FIFO_Get_Overrun_Status(DrvContextTypeDef *handle, uint8_t *status)
{
    LIS2DW12_ACC_FIFO_OVR_t tapStatus;

    if (LIS2DW12_ACC_R_FIFO_OverrunStatus((void*)handle, &tapStatus) == MEMS_ERROR){
        goto fail;
    }

    switch(tapStatus){
    case LIS2DW12_ACC_FIFO_OVR_OCCURRED:
        *status = 1;
        break;
    case LIS2DW12_ACC_FIFO_OVR_NOT_OCCURRED:
        *status = 0;
        break;
    default:
        goto fail;
    }

    return COMPONENT_OK;
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Get FIFO data
 * @param handle the device handle
 * @param *aData Pointer to the array where the data are stored
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LIS2DW12_FIFO_Get_Data(DrvContextTypeDef *handle, SensorAxes_t* aData, uint8_t size)
{
    uint8_t errorOccurred = 0;
    
    if(aData == NULL){
        goto fail;
    }
    
    for(uint8_t i = 0; i < size; i++){
        if(LIS2DW12_Get_Axes(handle, &aData[i]) != COMPONENT_OK){
            errorOccurred = 1;
        }
    }
    
    if(errorOccurred){
        goto fail;
    }
    
    return COMPONENT_OK;
        
fail: return COMPONENT_ERROR;
}

/**
 * @brief Get number of unread FIFO samples
 * @param handle the device handle
 * @param *nSamples Number of unread FIFO samples
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LIS2DW12_FIFO_Get_Num_Of_Samples(DrvContextTypeDef *handle, uint8_t *nSamples)
{
    if(LIS2DW12_ACC_R_FIFO_level((void*)handle, nSamples) == MEMS_ERROR){
        return COMPONENT_ERROR;
    }
    return COMPONENT_OK;
}

/**
 * @brief Set FIFO mode
 * @param handle the device handle
 * @param mode FIFO mode
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LIS2DW12_FIFO_Set_Mode(DrvContextTypeDef *handle, uint8_t mode)
{
    /* Verify that the passed parameter contains one of the valid values. */
    switch((LIS2DW12_ACC_FMODE_t)mode)
    {
    case LIS2DW12_ACC_FMODE_BYPASS:           /* Bypass mode.          0x00 */
    case LIS2DW12_ACC_FMODE_STOP_WHEN_FULL:   /* FIFO mode.            0x20 */
    case LIS2DW12_ACC_FMODE_STREAM_TO_FIFO:   /* Continuous-to-FIFO.   0x60 */
    case LIS2DW12_ACC_FMODE_BYPASS_TO_STREAM: /* Bypass-to-Continuous. 0x80 */
    case LIS2DW12_ACC_FMODE_STREAM:           /* Continuous mode.      0xC0 */
        break;
    default:
        goto fail;
    }
    
    if(LIS2DW12_ACC_W_FIFO_mode((void*)handle, (LIS2DW12_ACC_FMODE_t)mode) == MEMS_ERROR){
        goto fail;
    }
    
    return COMPONENT_OK;
        
fail: return COMPONENT_ERROR;
}

/**
 * @brief Set FIFO_FULL interrupt on INT1 pin
 * @param handle the device handle
 * @param status FIFO_FULL interrupt on INT1 pin enable/disable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LIS2DW12_FIFO_Set_INT1_FIFO_Full(DrvContextTypeDef *handle, uint8_t status)
{
    /* FIFO Full interrupt driven to INT1 pin. */
    if(LIS2DW12_ACC_W_PinFunction_INT1((void*)handle, LIS2DW12_ACC_INT1_MODE_FIFO_FULL, !!status) == MEMS_ERROR){
        goto fail;
    }
    
    /* @Note The code does not work properly in multiple event mode if latching interrupt. */
//    /* Latch interrupt. */
//    if(LIS2DW12_ACC_W_LatchIntteruptRq((void *)handle, LIS2DW12_ACC_LIR_ENABLE) == MEMS_ERROR){
//        goto fail;
//    }

    /* Enable interrupts. */
    if(LIS2DW12_ACC_W_HardwarePin((void *)handle, LIS2DW12_ACC_INTERRUPTS_ENABLE_ENABLE) == MEMS_ERROR){
        goto fail;
    }
    
    return COMPONENT_OK;
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Set FIFO_TRESHOLD interrupt on INT1 pin
 * @param handle the device handle
 * @param status FIFO_FULL interrupt on INT1 pin enable/disable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LIS2DW12_FIFO_Set_INT1_FIFO_Treshold(DrvContextTypeDef *handle, uint8_t status)
{
    /* FIFO Treshold interrupt driven to INT1 pin. */
    if(LIS2DW12_ACC_W_PinFunction_INT1((void*)handle, LIS2DW12_ACC_INT1_MODE_FIFO_TRESHOLD, !!status) == MEMS_ERROR){
        goto fail;
    }
    
    /* @Note The code does not work properly in multiple event mode if latching interrupt. */
//    /* Latch interrupt. */
//    if(LIS2DW12_ACC_W_LatchIntteruptRq((void *)handle, LIS2DW12_ACC_LIR_ENABLE) == MEMS_ERROR){
//        goto fail;
//    }

    /* Enable interrupts. */
    if(LIS2DW12_ACC_W_HardwarePin((void *)handle, LIS2DW12_ACC_INTERRUPTS_ENABLE_ENABLE) == MEMS_ERROR){
        goto fail;
    }
    
    return COMPONENT_OK;
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Set FIFO_TRESHOLD interrupt on INT2 pin
 * @param handle the device handle
 * @param status FIFO_FULL interrupt on INT1 pin enable/disable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LIS2DW12_FIFO_Set_INT2_FIFO_Treshold(DrvContextTypeDef *handle, uint8_t status)
{
    /* FIFO Treshold interrupt driven to INT1 pin. */
    if(LIS2DW12_ACC_W_PinFunction_INT2((void*)handle, LIS2DW12_ACC_INT2_MODE_FIFO_TRESHOLD, !!status) == MEMS_ERROR){
        goto fail;
    }
    
    /* @Note The code does not work properly in multiple event mode if latching interrupt. */
//    /* Latch interrupt. */
//    if(LIS2DW12_ACC_W_LatchIntteruptRq((void *)handle, LIS2DW12_ACC_LIR_ENABLE) == MEMS_ERROR){
//        goto fail;
//    }

    /* Enable interrupts. */
    if(LIS2DW12_ACC_W_HardwarePin((void *)handle, LIS2DW12_ACC_INTERRUPTS_ENABLE_ENABLE) == MEMS_ERROR){
        goto fail;
    }
    
    return COMPONENT_OK;
    
fail: return COMPONENT_ERROR;
}

/**
 * @brief Set FIFO watermark level
 * @param handle the device handle
 * @param watermark FIFO watermark level
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LIS2DW12_FIFO_Set_Watermark_Level(DrvContextTypeDef *handle, uint8_t watermark)
{
    if(LIS2DW12_ACC_W_FIFO_Threshold((void*)handle, watermark) == MEMS_ERROR){
        return COMPONENT_ERROR;
    }
    return COMPONENT_OK;
}




/**
 * @brief Set accelero interrupt latch
 * @param handle the device handle
 * @param status interrupt latch enable/disable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LIS2DW12_Set_Interrupt_Latch(DrvContextTypeDef *handle, uint8_t status)
{
    LIS2DW12_ACC_LIR_t stat;
    if(status == 0){
        stat = LIS2DW12_ACC_LIR_DISABLE;
    } else {
        stat = LIS2DW12_ACC_LIR_ENABLE;
    }
    
    if(LIS2DW12_ACC_W_LatchIntteruptRq((void *)handle, stat) == MEMS_ERROR){
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
