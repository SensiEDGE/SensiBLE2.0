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
  bool APP_CO_LUX_ENABLE;
  bool APP_LED_ENABLE;
  bool APP_BAT_ENABLE;
  bool APP_MIC_LEVEL_ENABLE;
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
