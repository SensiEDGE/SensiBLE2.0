/**
******************************************************************************
* @file    peripheral_mngr_app.h
* @author  Central Labs
* @version V 1.0.0
* @date    May-2017
* @brief   Header for peripheral_mngr_app.c file.
*******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        http://www.st.com/software_license_agreement_liberty_v2
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
********************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PERIPHERAL_MNGR_APP_H
#define __PERIPHERAL_MNGR_APP_H

/* Includes ------------------------------------------------------------------*/

#include "bluenrg1_stack.h"
#include "bluevoice_adpcm_bnrg1.h"
#include "bluevoice_app.h"
#include "inertial_app.h"

/* Masks for features used in manufacturer data */
#define SENSI_UUID_MASK_PEDOMETER       (1 << 0)         // Pedometer                     // 0
#define SENSI_UUID_MASK_GESTURE         (1 << 1)         // MemsGesture                   // 1
#define SENSI_UUID_MASK_PROX_GETURE     (1 << 2)         // ProximityGesture              // 2
#define SENSI_UUID_MASK_CARRY_POS       (1 << 3)         // Carry Position                // 3
#define SENSI_UUID_MASK_ACTIVITY        (1 << 4)         // Activity                      // 4
#define SENSI_UUID_MASK_COMPASS         (1 << 6)         // Compass                       // 5
#define SENSI_UUID_MASK_MOTION_INTENS   (1 << 5)         // Motion intensity              // 6
#define SENSI_UUID_MASK_FUSION          (1 << 7)         // Sensor Fusion                 // 7
#define SENSI_UUID_MASK_FUSION_COMPACT  (1 << 8)         // Sensor Fusion Compact         // 8
#define SENSI_UUID_MASK_FREEFALL        (1 << 9)         // FreeFall                      // 9
#define SENSI_UUID_MASK_ACC_EVENT       (1 << 10)        // AccEvent                      // 10
#define SENSI_UUID_MASK_BEAMFORMING     (1 << 11)        // Beam forming                  // 11
#define SENSI_UUID_MASK_SD_LOG          (1 << 12)        // SD Logging                    // 12
#define SENSI_UUID_MASK_OTA_REBOOT      (1 << 13)        // STM32WB OTA Reboot bit        // 13
#define SENSI_UUID_MASK_THREAD_REBOOT   (1 << 14)        // STM32WB Thread Reboot bit     // 14
#define SENSI_UUID_MASK_CO              (1 << 15)        // CO Sensor                     // 15
#define SENSI_UUID_MASK_TMP2            (1 << 16)        // Second Temperature            // 16
#define SENSI_UUID_MASK_BAT             (1 << 17)        // Battery                       // 17
#define SENSI_UUID_MASK_TMP1            (1 << 18)        // Temperature                   // 18
#define SENSI_UUID_MASK_HUM             (1 << 19)        // Humidity                      // 19
#define SENSI_UUID_MASK_PRES            (1 << 20)        // Pressure                      // 20
#define SENSI_UUID_MASK_AMG             (1 << 21)        // Mag                           // 21
#define SENSI_UUID_MASK_GYRO            (1 << 22)        // Gyro                          // 22
#define SENSI_UUID_MASK_ACC             (1 << 23)        // Acc                           // 23
#define SENSI_UUID_MASK_LUX             (1 << 24)        // Lux                           // 24
#define SENSI_UUID_MASK_PROX            (1 << 25)        // Proximity                     // 25
#define SENSI_UUID_MASK_MIC_LEVEL       (1 << 26)        // MicLevel                      // 26
#define SENSI_UUID_MASK_ADPC_AUDIO      (1 << 27)        // ADPC Audio                    // 27
#define SENSI_UUID_MASK_DIR_OF_ARRIVAL  (1 << 28)        // Direction of arrival          // 28
#define SENSI_UUID_MASK_SWITCH          (1 << 29)        // Switch                        // 29
#define SENSI_UUID_MASK_ADPCM_SYNC      (1 << 30)        // ADPCM Sync                    // 30
#define SENSI_UUID_MASK_RFU             (1 << 31)        // RFU                           // 31
#define SENSI_UUID_MASK_COUNT           (1 << 32)        // SENSI_FEATURES_COUNT          // 32

/** @addtogroup BLUEMIC_1_APP BLUEMIC_1_APP
 * @{
 */

/** @addtogroup BLUEMIC_1_APP_MNGR BLUEMIC_1_APP_MNGR
 * @{
 */

/** @defgroup APP_MNGR_Exported_Types APP_MNGR_Exported_Types
  * @{
  */

/**
 * @brief BlueMic-1 application status
 */
typedef enum 
{
  APP_SUCCESS = 0x00, /*!< APP Success.*/
  APP_ERROR = 0x10    /*!< APP Error.*/
} APP_Status;

/**
  * @}
  */

/** @defgroup APP_MNGR_Exported_Defines APP_MNGR_Exported_Defines
  * @{
  */
typedef enum
{
  APP_STATUS_ADVERTISEMENT = (0x10),      /*!< BlueVoice Peripheral device is in advertisement mode.*/
  APP_STATUS_CONNECTED     = (0x20)       /*!< BlueVoice device is connected.*/
} APP_State;

/**
  * @}
  */

#include <stdbool.h>

struct app_per_enabled_s
{
//  bool APP_READY;
  bool APP_BLUEVOICE_ENABLE;
  bool APP_INERTIAL_ENABLE;
  bool APP_ENV_ENABLE;
  bool APP_LUX_ENABLE;
  bool APP_LED_ENABLE;
  bool APP_BAT_ENABLE;
  bool APP_MIC_LEVEL_ENABLE;
  bool APP_COLOR_AMBIENT;
};

/* Exported variables --------------------------------------------------------*/
extern volatile uint8_t AccGryro_DataReady;
extern volatile APP_State APP_PER_state;
extern volatile uint8_t ButtonState;
extern volatile struct app_per_enabled_s APP_PER_enabled;

/* Exported functions ------------------------------------------------------- */

/** @defgroup APP_MNGR_Functions_Prototype APP_MNGR_Functions_Prototype
  * @{
  */
  
/**
 * @brief  Process user input.
 * @param  None.
 * @retval None.
 */
void APP_Tick(void);

/**
 * @brief  BlueNRG-1 Initialization.
 * @param  None.
 * @retval APP_Status: APP_SUCCESS if the configuration is ok, APP_ERROR otherwise.
 */
APP_Status PER_APP_Init_BLE(void);

/**
 * @brief  BLE service and characteristics initialization.
 * @param  None.
 * @retval APP_Status: APP_SUCCESS if the configuration is ok, APP_ERROR otherwise.
 */
APP_Status PER_APP_Service_Init(void);

void PER_APP_Stop_Advertise(void);
/**
 * @brief  This function is called from the server in order set advertisement mode.
 * @param  None
 * @retval APP_Status: APP_SUCCESS if the configuration is ok, APP_ERROR otherwise.
 */
APP_Status PER_APP_Advertise(void);

void setIbeacon(void);

/**
 * @brief  Error handler.
 * @param  None.
 * @retval None.
 */
void PER_APP_Error_Handler(void);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __PERIPHERAL_MNGR_APP_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
