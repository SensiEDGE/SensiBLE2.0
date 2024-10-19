/**
 ******************************************************************************
 * @file    VEML6075_Driver.c
 * @version V1.0
 * @date    09-October-2024
 * @brief   VEML6075 driver file
 ******************************************************************************
 *
 * COPYRIGHT(c) 2024 SensiEDGE
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
#include "VEML6075_Driver.h"

#ifdef  USE_FULL_ASSERT_VEML6075
#include <stdio.h>
#endif

// Calibration constants:
// Four gain calibration constants -- alpha, beta, gamma, delta -- can be used to correct the output in
// reference to a GOLDEN sample. The golden sample should be calibrated under a solar simulator.
// Setting these to 1.0 essentialy eliminates the "golden"-sample calibration
const float CALIBRATION_ALPHA_VIS = 1.0; // UVA / UVAgolden
const float CALIBRATION_BETA_VIS = 1.0;  // UVB / UVBgolden
const float CALIBRATION_GAMMA_IR = 1.0;  // UVcomp1 / UVcomp1golden
const float CALIBRATION_DELTA_IR = 1.0;  // UVcomp2 / UVcomp2golden

// These values are recommended by the "Designing the VEML6075 into an application" app note for 100ms IT
const float UVA_RESPONSIVITY = 0.001461; // UVAresponsivity
const float UVB_RESPONSIVITY = 0.002591; // UVBresponsivity

// UV coefficients:
// These values are recommended by the "Designing the VEML6075 into an application" app note
const float UVA_VIS_COEF_A = 2.22;
const float UVA_IR_COEF_B = 1.33;
const float UVB_VIS_COEF_C = 2.95;
const float UVB_IR_COEF_D = 1.74;

extern uint8_t Sensor_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
extern uint8_t Sensor_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );

/*******************************************************************************
* Function Name : VEML6075_ReadReg
* Description   : Generic Reading function. It must be fullfilled with
*               : I2C reading functions
* Input         : Register Address
* Output        : Data Read
* Return        : None
*******************************************************************************/
VEML6075_Error_et VEML6075_ReadReg( void *handle, uint8_t RegAddr, 
                                    uint16_t NumByteToRead, uint8_t *Data )
{

  if ( Sensor_IO_Read( handle, RegAddr, Data, NumByteToRead ) )
    return VEML6075_ERROR;
  else
    return VEML6075_OK;
}


/*******************************************************************************
* Function Name : VEML6075_WriteReg
* Description   : Generic Writing function. It must be fullfilled with either
*               : I2C or SPI writing function
* Input         : Register Address, Data to be written
* Output        : None
* Return        : None
*******************************************************************************/
VEML6075_Error_et VEML6075_WriteReg( void *handle, uint8_t RegAddr, 
                                    uint16_t NumByteToWrite, uint8_t *Data )
{
  if ( Sensor_IO_Write( handle, RegAddr, Data, NumByteToWrite ) )
    return VEML6075_ERROR;
  else
    return VEML6075_OK;
}


/**
* @brief  Get the version of this driver.
* @param  pxVersion pointer to a VEML6075_DriverVersion_st structure that 
*         contains the version information.
*         This parameter is a pointer to @ref VEML6075_DriverVersion_st.
* @retval Error code [VEML6075_OK, VEML6075_ERROR].
*/
VEML6075_Error_et VEML6075_Get_DriverVersion(VEML6075_DriverVersion_st* version)
{
  version->Major = VEML6075_DRIVER_VERSION_MAJOR;
  version->Minor = VEML6075_DRIVER_VERSION_MINOR;
  version->Point = VEML6075_DRIVER_VERSION_POINT;

  return VEML6075_OK;
}


/**
* @brief  Get device type ID.
* @param  *handle Device handle.
* @param  deviceid pointer to the returned device type ID.
* @retval Error code [VEML6075_OK, VEML6075_ERROR].
*/
VEML6075_Error_et VEML6075_Get_DeviceID(void *handle, uint8_t* deviceid)
{
  if(VEML6075_ReadReg(handle, VEML6075_WHO_AM_I_REG, 2, deviceid))
    return VEML6075_ERROR;

  return VEML6075_OK;
}


/**
* @brief  De initialization function for VEML6075.
*         This function put the VEML6075 in power down, make a memory boot and clear the data output flags.
* @param  *handle Device handle.
* @retval Error code [VEML6075_OK, VEML6075_ERROR].
*/
VEML6075_Error_et VEML6075_DeInit(void *handle)
{
  VEML6075_DeActivate(handle);

  return VEML6075_OK;
}


/**
* @brief  Read VEML6075 output registers, and calculate ultraviolet.
* @param  *handle Device handle.
* @param  ultraviolet pointer to the returned ultraviolet value in units
* @retval Error code [VEML6075_OK, VEML6075_ERROR].
*/
VEML6075_Error_et VEML6075_Get_Measurement(void *handle, int16_t* ultraviolet)
{
  if ( VEML6075_Get_UltravioletIndex( handle, ultraviolet ) == VEML6075_ERROR ) return VEML6075_ERROR;

  return VEML6075_OK;
}


/**
* @brief  Read VEML6075 Ultraviolet output registers
* @param  *handle Device handle.
* @param  Pointer to the returned ultraviolet value
* @retval Error code [VEML6075_OK, VEML6075_ERROR].
*/
VEML6075_Error_et VEML6075_Get_UltravioletIndex(void *handle, int16_t* value)
{
  uint16_t uva_data, uvb_data, visibleComp, irComp;
  float uvia, uvib, uviaCalc, uvibCalc;
  uint8_t buffer[2] = {0};

  if(VEML6075_ReadReg(handle, VEML6075_UVA_DATA_REG, 2, buffer))return VEML6075_ERROR;
  uva_data = (buffer[0]) | ( buffer[1] << 8);

  if(VEML6075_ReadReg(handle, VEML6075_UVB_DATA_REG, 2, buffer)) return VEML6075_ERROR;
  uvb_data = (buffer[0]) | (buffer[1] << 8);

  if (VEML6075_Get_UvComp1(handle, &visibleComp) != VEML6075_OK) return VEML6075_ERROR;
  if (VEML6075_Get_UvComp2(handle, &irComp) != VEML6075_OK) return VEML6075_ERROR;

  // Calculate the simple UVIA and UVIB.
  uviaCalc = (float)uva_data - ((UVA_VIS_COEF_A * CALIBRATION_ALPHA_VIS * visibleComp) / CALIBRATION_GAMMA_IR) - ((UVA_IR_COEF_B * CALIBRATION_ALPHA_VIS * irComp) / CALIBRATION_DELTA_IR);
  uvibCalc = (float)uvb_data - ((UVB_VIS_COEF_C * CALIBRATION_BETA_VIS * visibleComp) / CALIBRATION_GAMMA_IR) - ((UVB_IR_COEF_D * CALIBRATION_BETA_VIS * irComp) / CALIBRATION_DELTA_IR);

  // Convert raw UVIA and UVIB to values scaled by the sensor responsivity
  uvia = uviaCalc * (1.0 / CALIBRATION_ALPHA_VIS) * UVA_RESPONSIVITY;
  uvib = uvibCalc * (1.0 / CALIBRATION_BETA_VIS) * UVB_RESPONSIVITY;

  /* Calculate average value */
  *value = (uvia + uvib) / 2;

  return VEML6075_OK;
}


/**
* @brief  Exit from power down mode.
* @param  *handle Device handle.
* @param  void.
* @retval Error code [VEML6075_OK, VEML6075_ERROR].
*/
VEML6075_Error_et VEML6075_Activate(void *handle)
{
  uint8_t tmp[2] = {0};

  if(VEML6075_ReadReg(handle, VEML6075_UV_CONF_REG1, 2, tmp))
    return VEML6075_ERROR;

  tmp[0] &= ~VEML6075_SD_MASK;
  tmp[0] |= VEML6075_UV_IT_100MS;

  if(VEML6075_WriteReg(handle, VEML6075_UV_CONF_REG1, 2, tmp))
    return VEML6075_ERROR;

  return VEML6075_OK;
}

/**
* @brief  Put the sensor in power down mode.
* @param  *handle Device handle.
* @retval Error code [VEML6075_OK, VEML6075_ERROR].
*/
VEML6075_Error_et VEML6075_DeActivate(void *handle)
{
  uint8_t tmp[2] = {0};

  if(VEML6075_ReadReg(handle, VEML6075_UV_CONF_REG1, 2, tmp))
    return VEML6075_ERROR;

  tmp[0] |= VEML6075_SD_MASK;

  if(VEML6075_WriteReg(handle, VEML6075_UV_CONF_REG1, 2, tmp))
    return VEML6075_ERROR;

  return VEML6075_OK;
}


/**
* @brief  Enter or exit from power down mode.
* @param  *handle Device handle.
* @param  status can be VEML6075_SET: VEML6075 in power down mode.
* @param  status can be VEML6075_REET: VEML6075 in active mode.
*         This parameter is a @ref VEML6075_BitStatus_et.
* @retval Error code [VEML6075_OK, VEML6075_ERROR].
*/
VEML6075_Error_et VEML6075_Set_PowerDownMode(void *handle, VEML6075_BitStatus_et status)
{
  uint8_t tmp[2]= {0};

  VEML6075_assert_param(IS_VEML6075_BitStatus(status));

  if(VEML6075_ReadReg(handle, VEML6075_UV_CONF_REG1, 2, tmp))
    return VEML6075_ERROR;

  tmp[0] |= VEML6075_SD_MASK;

  if(VEML6075_WriteReg(handle, VEML6075_UV_CONF_REG1, 2, tmp))
    return VEML6075_ERROR;

  return VEML6075_OK;
}

/**
* @brief  Get if VEML6075 is in active mode or in power down mode.
* @param  *handle Device handle.
* @param  Pointer to the returned value with VEML6075 status.
* @retval Error code [VEML6075_OK, VEML6075_ERROR].
*/
VEML6075_Error_et VEML6075_Get_PowerDownMode(void *handle, VEML6075_BitStatus_et* status)
{
  uint8_t tmp[2]= {0};

  if(VEML6075_ReadReg(handle, VEML6075_UV_CONF_REG1, 2, tmp))
    return VEML6075_ERROR;

  *status = (VEML6075_BitStatus_et)((tmp[0] & VEML6075_SD_MASK));

  return VEML6075_OK;
}


VEML6075_Error_et VEML6075_Get_UvComp1(void *handle, uint16_t *uvComp1)
{
    uint8_t raw[2] = {0};

    if(VEML6075_ReadReg(handle, VEML6075_UVCOMP1_DATA_REG, 2, raw))
        return VEML6075_ERROR;

    *uvComp1 = (raw[0]) | (raw[1] << 8);

    return VEML6075_OK;
}

VEML6075_Error_et VEML6075_Get_UvComp2(void *handle, uint16_t *uvComp2)
{
    uint8_t raw[2] = {0};

    if(VEML6075_ReadReg(handle, VEML6075_UVCOMP2_DATA_REG, 2, raw))
        return VEML6075_ERROR;

    *uvComp2 = (raw[0]) | (raw[1] << 8);

    return VEML6075_OK;
}

#ifdef  USE_FULL_ASSERT_VEML6075
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval : None
*/
void VEML6075_assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number */
  printf("Wrong parameters value: file %s on line %d\r\n", file, (int)line);

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT SensiEdge *****************************/
