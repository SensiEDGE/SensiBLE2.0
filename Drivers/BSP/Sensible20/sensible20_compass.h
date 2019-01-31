/**
 ******************************************************************************
 * @file    sensible20_compass.h
 * @date    20-December-2018
 * @brief   This is a header file for compass feature.
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
#ifndef __SENSIBLE_2_0_COMPASS_H
#define __SENSIBLE_2_0_COMPASS_H

/* Includes ------------------------------------------------------------------*/
#include "steval_bluemic1.h"
#include "sensible_sensors.h"

/* Public defines ------------------------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/

/**
  * @brief  Start eCompass. 
  *         Set Accelero and Magneto Enable
  * @param  none
  * @retval none
  */
void BSP_Compass_Start(void);

/**
  * @brief  Stop eCompass. 
  *         Set Accelero and Magneto Disable
  * @param  none
  * @retval none
  */
void BSP_Compass_Stop(void);

/**
  * @brief  Run compass algorithm or calibration
  * @param  none
  * @retval SENSIBLE_SENS_OK  in case of success
  * @retval SENSIBLE_SENS_ERR in case of error
  */
SensibleResult_t BSP_Compass_Run(void);

/**
  * @brief  This function is used for applying calibration coefficients 
  *         to measured magneto values.
  * @param  magAxis  the pointer where measured magneto values are saved
  * @retval none
  */
void BSP_Compass_ApplyCalibrationCoefficients(SensorAxes_t* magAxis);

/**
  * @brief  Check if magneto was calibrated
  * @param  none
  * @retval none
  */
void BSP_Compass_Check_Calibration(void);

/**
  * @brief  Inform about compass state (active or not)
  * @param  none
  * @retval BOOL
  */
BOOL BSP_Compass_Get_Active(void);

/**
  * @brief  Inform if compass is calibrated
  * @param  none
  * @retval BOOL
  */
BOOL BSP_Compass_Get_Calibrated(void);

/**
  * @brief  Increase compass counter
  * @param  none
  * @retval none
  */
void BSP_Compass_IncTick(void);

/**
  * @brief  Inform if it is necessity to calibrate magneto sensor
  * @param  state TRUE if it is necessity to calibrate magneto sensor,
  *               FALSE in other way.
  * @retval none
  */
void BSP_Compass_Set_Need_Calibration(BOOL state);

/**
  * @brief  Inform if it is time to update compass information
  * @param  none
  * @retval BOOL
  */
BOOL BSP_Compass_Need_Update(void);

 #endif //__SENSIBLE_2_0_BAT_H

/************************ (C) COPYRIGHT SensiEdge *****************************/
