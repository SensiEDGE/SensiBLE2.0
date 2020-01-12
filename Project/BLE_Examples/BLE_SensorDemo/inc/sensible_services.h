/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSIBLE_SERVICES_H
#define __SENSIBLE_SERVICES_H

/* Includes ------------------------------------------------------------------*/

/* Feature mask for Sensor fusion short precision */
#define FEATURE_MASK_SENSORFUSION_SHORT 0x00000100

/* Feature mask for e-compass */
#define FEATURE_MASK_ECOMPASS    0x00000040

/* W2ST command for asking the calibration status */
#define W2ST_COMMAND_CAL_STATUS 0xFF
/* W2ST command for resetting the calibration */
#define W2ST_COMMAND_CAL_RESET  0x00
/* W2ST command for stopping the calibration process */
#define W2ST_COMMAND_CAL_STOP   0x01
/* W2ST message about calibration ok */
#define W2ST_COMPASS_CAL_OK     0x64
/* W2ST message about calibration error */
#define W2ST_COMPASS_CAL_ERROR  0x00

#include "bluenrg1_stack.h"
#include "bluevoice_adpcm_bnrg1.h"
#include "bluevoice_app.h"
#include "inertial_app.h"

void sensible_aci_gatt_attribute_modified_event(uint16_t Connection_Handle,
                                                uint16_t Attr_Handle,
                                                uint16_t Offset,
                                                uint8_t Attr_Data_Length,
                                                uint8_t Attr_Data[]);

tBleStatus Add_Console_Service(void);

tBleStatus Add_Environmental_Sensor_Service(void);
tBleStatus Add_HW_SW_ServW2ST_Service(void);

tBleStatus EnvironmentalUpdate(void);
//tBleStatus LuxUpdate(void);
tBleStatus CoLuxUpdate(void);

tBleStatus AccEvent_Notify(uint16_t Command, uint8_t dimByte);
tBleStatus Config_Notify(uint32_t Feature, uint8_t Command, uint8_t val);
tBleStatus ECompass_Update(uint16_t Angle);


void LedUpdate(Led_TypeDef led);
void BatUpdate();

/**
* @brief  User function that is called when 1 ms of PDM data is available.
* @param  none
* @retval None
*/
void AudioProcess(uint16_t* PCM_Buffer);

void MicLevelUpdate();
                                                
void UpdateAll(void);

#endif /* __SENSIBLE_SERVICES_H */


