/**
 ******************************************************************************
 * @file    peripheral_mngr_app.c
 * @author  Central Labs
 * @version V 1.0.0
 * @date    May-2017
 * @brief   This file contains definitions for the application manager.
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

/* Includes ------------------------------------------------------------------*/
#include "peripheral_mngr_app.h"
#include "steval_bluemic1.h"
#include "sensible_services.h"
#include "steval_bluemic1_audio_in.h"
#include "sensible_sensors.h"
#include "sensible20_compass.h"

#include "sleep.h"
#include "main.h"
#include "clock.h"
#include <inttypes.h>

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
   
/** @addtogroup BLUEMIC_1_APP BLUEMIC_1_APP
 * @{
 */

/** @defgroup BLUEMIC_1_APP_MNGR BLUEMIC_1_APP_MNGR
 * @{
 */

/** @defgroup APP_MNGR_Private_Defines APP_MNGR_Private_Defines
 * @{
 */

#ifdef SENSIBLE_2_0
#define NAME_WEAR 'S', 'e', 'n', 's', 'i', 'B', 'L', 'E', '2', '.', '0'
#elif defined SENSIBLE_2_1
#define NAME_WEAR 'S', 'e', 'n', 's', 'i', 'B', 'L', 'E', '2', '.', '1'
#else
#define NAME_WEAR 'B', 'V', 'B', 'N', 'R', 'G', '1'
#endif // SENSIBLE_2_0
   
#if (defined SENSIBLE_2_0) || (defined SENSIBLE_2_1)

#define APP_UPDATE_ALL_PERIOD                           500 //ms
#define APP_UPDATE_MIC_LEVEL_PERIOD                     50 //ms
#define LED_ON_TIME                                     50  //ms
#define LED_OFF_TIME                                    950 //ms

#else // SENSIBLE_2_0

#define LED_TOGGLE_ADVERTISEMENT                        10000
#define LED_TOGGLE_READY                                3000
#define LED_TOGGLE_INERTIAL                             800
#define LED_TOGGLE_BLUEVOICE                            800

#endif // SENSIBLE_2_0

/**
 * @}
 */
 
/** @defgroup APP_MNGR_Private_Variables APP_MNGR_Private_Variables
 * @{
 */
extern uint32_t FirstConnectionConfig;

extern void DeinitAll(void);
extern void EnableAll(void);

volatile uint16_t ServiceHandle;
volatile uint16_t conn_handle;
volatile struct app_per_enabled_s APP_PER_enabled = {0};

uint8_t PERIPHERAL_BDADDR[] = { 0x55, 0x11, 0x07, 0x01, 0x16, 0xE2 };

static uint32_t updateAllTime = 0;
static uint32_t micLevelUpdateTime = 0;
static uint32_t ledSwitchTime = 0;
static BOOL ledActive = FALSE;// Variable used for holding LED state;

/**
 * @}
 */

/** @defgroup APP_MNGR_Exported_Variables APP_MNGR_Exported_Variables
 * @{
 */
volatile APP_State APP_PER_state = APP_STATUS_ADVERTISEMENT;
volatile uint8_t AccGryro_DataReady = 0;
volatile uint8_t ButtonState = 0;
/**
 * @}
 */

/** @defgroup APP_MNGR_Private_Functions APP_MNGR_Private_Functions
 * @{
 */

static void ledBlink(Led_TypeDef led);

/**
 * @brief  BlueNRG-1 Initialization.
 * @param  None.
 * @retval APP_Status: APP_SUCCESS if the configuration is ok, APP_ERROR otherwise.
 */
APP_Status PER_APP_Init_BLE(void)
{
  uint8_t ret = 0;
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
#if (defined SENSIBLE_2_0) || (defined SENSIBLE_2_1)
  // Try to generate unique MAC
  uint8_t *p = (uint8_t*)0x100007F4;
  for(int i = 0; i < 6; i++) {
      PERIPHERAL_BDADDR[i] = *p++;
  }
#endif
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, PERIPHERAL_BDADDR);
  
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("aci_hal_write_config_data() failed: 0x%02x\r\n", ret);
    return APP_ERROR;
  }
  
  aci_hal_set_tx_power_level(1, 4);
  
  /* GATT Init */
  ret = aci_gatt_init();
  if (ret != BLE_STATUS_SUCCESS) 
  { 
    PRINTF("aci_gatt_init() failed: 0x%02x\r\n", ret);
    return APP_ERROR;
  }
 
  /* GAP Init */
  ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  if (ret != BLE_STATUS_SUCCESS) 
  {
    PRINTF("aci_gap_init() failed: 0x%02x\r\n", ret);
    return APP_ERROR;
  }

  /* Set auth requirement*/
  aci_gap_set_authentication_requirement(
      NO_BONDING, //BONDING,
      MITM_PROTECTION_NOT_REQUIRED, //MITM_PROTECTION_REQUIRED,
      SC_IS_NOT_SUPPORTED, //OOB_AUTH_DATA_ABSENT,
      KEYPRESS_IS_NOT_SUPPORTED, //NULL,
      7,
      16,
      DONOT_USE_FIXED_PIN_FOR_PAIRING, //USE_FIXED_PIN_FOR_PAIRING,
      123456,
      0x00//BONDING
      );

  PRINTF("BLE Stack Initialized with SUCCESS\n");

  return APP_SUCCESS;
}

/**
 * @brief  BLE service and characteristics initialization.
 * @param  None.
 * @retval APP_Status: APP_SUCCESS if the configuration is ok, APP_ERROR otherwise.
 */

APP_Status PER_APP_Service_Init(void)
{
  /* Console service and characteristics initialization. (for OTA)*/
	tBleStatus ret = Add_Console_Service();
  
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Error while adding Add_Console_Service().\n");
    return APP_ERROR;
  }

  ret = Add_HW_SW_ServW2ST_Service();
  
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Error while adding Add_HW_SW_ServW2ST_Service().\n");
    return APP_ERROR;
  }

  ServiceHandle = GetHWServW2STHandle();
  return APP_SUCCESS;
}

void PER_APP_Stop_Advertise(void)
{
    hci_le_set_advertise_enable(0);
}

/**
 * @brief  This function is called from the server in order set advertisement mode.
 * @param  None
 * @retval APP_Status: APP_SUCCESS if the configuration is ok, APP_ERROR otherwise.
 */
APP_Status PER_APP_Advertise(void)
{
  uint8_t ret = 0;
  
  uint8_t local_name[] =
  {
    AD_TYPE_COMPLETE_LOCAL_NAME, NAME_WEAR
  };

  uint32_t feature = SENSI_UUID_MASK_PEDOMETER;
  feature |= SENSI_UUID_MASK_COMPASS;
  feature |= SENSI_UUID_MASK_ACC_EVENT;
  feature |= SENSI_UUID_MASK_BAT;
  feature |= SENSI_UUID_MASK_TMP1;
  feature |= SENSI_UUID_MASK_HUM;
  feature |= SENSI_UUID_MASK_PRES;
  feature |= SENSI_UUID_MASK_AMG;
  feature |= SENSI_UUID_MASK_ACC;
  feature |= SENSI_UUID_MASK_LUX;
  feature |= SENSI_UUID_MASK_MIC_LEVEL;
  feature |= SENSI_UUID_MASK_ADPC_AUDIO;
  feature |= SENSI_UUID_MASK_SWITCH;
  feature |= SENSI_UUID_MASK_ADPCM_SYNC;


#if (defined SENSIBLE_2_0) || (defined SENSIBLE_2_1)
  uint8_t manuf_data[24] = {

    2,0x0A,0x00,

    // 8,0x09,NAME_WEAR, /* Complete Name */
    12,0x09,NAME_WEAR, /* Complete Name */
    7,0xFF,0x01,      /* SKD version */
         // The device types:
         // 0x00 for a generic device
         // 0x01 is reserved for the STEVAL-WESU1 board
         // 0x02 is reserved for the STEVAL-STLKT01V1 (SensorTile) board
         // 0x03 is reserved for the STEVAL-BCNKT01V1 (BlueCoin) board
         // 0x80 for a generic Nucleo board
         // 0x81 for a Nucleo board exporting remote feature
         0x80,// 0x00, // The device type

		 //(removed second temperature) 0xBF,//(1 ) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (0 << 5) | (0 << 7), /*Second Temp, Bat, Temp, Hum, Pres, Mag, Acc */
//         #warning "CO sensor here"
		 /* Accelerometer Events | CO sensor*/
		 (feature & 0xff000000) >> 24,
		 (feature & 0x00ff0000) >> 16,
		 (feature & 0x0000ff00) >> 8,
		 (feature & 0x000000ff)
  };

#else // For ST board
  uint8_t manuf_data[24] = {
  2,0x0A,0x00,
  12,0x09,NAME_WEAR, /* Complete Name */
  7,0xFF,0x01,      /* SKD version */
         0x00,
         0x48,      /* AudioSync+AudioData */
         0xC0,      /* Acc+Gyr */
         0x04,
         0x00 
  };
#endif // SENSIBLE_2_0

  /* disable scan response */
  ret = hci_le_set_scan_response_data(0, NULL);

  ret = aci_gap_set_discoverable(ADV_IND,
                                 (300 * 1000) / 625, (300 * 1000) / 625,
//                                 (100 * 1000) / 625, (100 * 1000) / 625,
                                 PUBLIC_ADDR, NO_WHITE_LIST_USE,
                           sizeof(local_name), local_name, 0, NULL, 0, 0);

  /* Send Advertising data */
#if (defined SENSIBLE_2_0) || (defined SENSIBLE_2_1)
  ret = aci_gap_update_adv_data(24, manuf_data);
#else
  ret = aci_gap_update_adv_data(20, manuf_data);
#endif // SENSIBLE_2_0
  
  if (ret != BLE_STATUS_SUCCESS)
  {
    return APP_ERROR;
  }
  
  return APP_SUCCESS;
}

void setIbeacon(void)
{
 
  uint16_t major = 0;
  uint16_t minor = 0;
  
  
/*
 * minor and major values represent the ibeacon information.
 * User can change these values in order to use beacon for something.
 * In this demo, major codes SensiEDGE board ( 1 for SensiBLE1)
 * Minor is generated from MCU UUID to distinguish the boards.
 */
  
  major = 2; // 2 is for SensiBLE 2  
  minor = (PERIPHERAL_BDADDR[4] << 8) | PERIPHERAL_BDADDR[5];//STM32_UUID[1] & 0x0000FFFF;
  minor <<= 8;
  minor ^= (PERIPHERAL_BDADDR[2] << 8) | PERIPHERAL_BDADDR[3];//(STM32_UUID[1] & 0xFFFF0000) >> 16;
  minor <<= 8;
  minor ^= (PERIPHERAL_BDADDR[0] << 8) | PERIPHERAL_BDADDR[1];
  
  static uint8_t ibeacon[] = 
    {
        0x02, 0x01, 0x06, 0x1A, 0xFF, 0x4C, 0x00, 0x02, 0x15,
        0xC2, 0x64, 0x25, 0x63, 0xEC, 0x5E, 0x49, 0xCA, 0x95, 0x38, 0xF2, 0x9A, 0x6A, 0xA7, 0xE4, 0x07,
        0x00, 0x00, // major
        0x00, 0x00, // minor
        0xBA // -70dB@1m
    };
    
  ibeacon[25] = (major & 0xFF00) >> 8;
  ibeacon[26] = major & 0x00FF;
  ibeacon[27] = minor & 0x00FF;
  ibeacon[28] = (minor & 0xFF00) >> 8;
        
    volatile tBleStatus stat;
    
    /* disable scan response */
    
  hci_le_set_scan_response_data(0, NULL);
  stat = aci_gap_set_discoverable(ADV_IND,
//                           (100 * 1000) / 625, (100 * 1000) / 625,
                           (300 * 1000) / 625, (300 * 1000) / 625,
                           RANDOM_ADDR, // PUBLIC_ADDR,
                           NO_WHITE_LIST_USE,
                           0 , NULL, 0, NULL, 0, 0);

  if(stat != 0) {
    stat = 0;
  }
  
    stat = aci_gap_delete_ad_type(AD_TYPE_TX_POWER_LEVEL);
    if(stat != 0) {
      stat = 0;
    }
    stat = aci_gap_delete_ad_type(AD_TYPE_COMPLETE_LOCAL_NAME);
    if(stat != 0) {
      stat = 0;
    }
    stat = aci_gap_delete_ad_type(AD_TYPE_MANUFACTURER_SPECIFIC_DATA);
    if(stat != 0) {
      stat = 0;
    }
    
    stat = aci_gap_update_adv_data(sizeof(ibeacon), ibeacon);
    if(stat != 0)
        stat = 0;
}

/**
 * @brief  Process user input.
 * @param  None.
 * @retval None.
 */
void APP_Tick(void)
{
    //Blink LED
    if (tick_ms() >= ledSwitchTime){
        if(APP_PER_state != APP_STATUS_ADVERTISEMENT){
            //green led blink
            ledBlink(LED1);
        }
    }

    switch (APP_PER_state)
    {
      case APP_STATUS_ADVERTISEMENT:
      {
          static uint32_t advChangeTime = 0;

          if(advChangeTime == 0)
          {
              static uint8_t advState = 0;
              advChangeTime = 15;//15 is the number of cycles
              if(advState == 0)
              {
                  advState = 1;
                  PER_APP_Advertise();
              }
              else
              {
                  advState = 0;
                  setIbeacon();
              }
          }
          advChangeTime -= 1;

          static uint8_t periodCounter = 0;//for red led blink

          periodCounter += 1;
          if(periodCounter == 1){
              BSP_LED_On(LED2);
          } else if(periodCounter == 15){//15 is the number of cycles
              BSP_LED_Off(LED2);
          } else if(periodCounter >= 60){//60 is the number of cycles
              periodCounter = 0;
          }
      }
      break;

      case APP_STATUS_CONNECTED:
      {
          if (APP_PER_enabled.APP_BLUEVOICE_ENABLE != 0)
          {
              if(audio_streaming_active)
              {
                  /* Updated Audio data */
                  BV_APP_DataUpdate();
              }
          }

          if (APP_PER_enabled.APP_INERTIAL_ENABLE != 0)
          {
              if(AccGryro_DataReady)
              {
                  AccGryro_DataReady = 0;
                  /* Updated Acc and Gyro data */
                  INERTIAL_APP_DataUpdate(ServiceHandle);
              }
          }

          if (APP_PER_enabled.APP_MIC_LEVEL_ENABLE != 0)
          {
              if (tick_ms() >= micLevelUpdateTime)
              {
                  MicLevelUpdate();
                  micLevelUpdateTime = tick_ms() + APP_UPDATE_MIC_LEVEL_PERIOD;
              }
          }

          if(tick_ms() >= updateAllTime)
          {
              if (APP_PER_enabled.APP_ENV_ENABLE != 0)
              {
                  EnvironmentalUpdate();
              }

              if (APP_PER_enabled.APP_LUX_ENABLE != 0)
              {
                  LuxUpdate();
              }

              if (APP_PER_enabled.APP_LED_ENABLE != 0)
              {
                  LedUpdate(LED2);
              }

              if (APP_PER_enabled.APP_BAT_ENABLE != 0)
              {
                  BatUpdate();
              }

              if (APP_PER_enabled.APP_COLOR_AMBIENT != 0)
              {
            	  ColorAmbientLightUpdate();
              }

              updateAllTime = tick_ms() + APP_UPDATE_ALL_PERIOD;
          }

      }
      break;
    }
}

static void ledBlink(Led_TypeDef led)
{
    if(ledActive){
        ledActive = FALSE;
        BSP_LED_Off(led);
        ledSwitchTime = tick_ms() + LED_OFF_TIME;
    } else {
        ledActive = TRUE;
        BSP_LED_On(led);
        ledSwitchTime = tick_ms() + LED_ON_TIME;
    }
}

/**
 * @brief  Error handler.
 * @param  None.
 * @retval None.
 */
void PER_APP_Error_Handler(void)
{
  BSP_LED_On(LED1);
  BSP_LED_Off(LED2);
  while(1)
  {
      BSP_LED_Toggle(LED1);
      DelayMs(500);
      BSP_LED_Toggle(LED2);
      DelayMs(500);

#if 0
      BlueNRG_Sleep(SLEEPMODE_WAKETIMER, 0, 0);
#endif
  }
}    

/**
 * @}
 */

/** @defgroup APP_MNGR_BlueNRG_Events APP_MNGR_BlueNRG_Events
 * @{
 */

/*******************************************************************************
 * Function Name  : hci_le_connection_complete_event.
 * Description    : This event indicates that a new connection has been created.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)

{
  /* Connection completed */
  BluevoiceADPCM_BNRG1_ConnectionComplete_CB(Connection_Handle);
  
  FirstConnectionConfig = 0;
  
#if (!defined SENSIBLE_2_0) && (!defined SENSIBLE_2_1)
  BSP_LED_On(LED1);
#endif // SENSIBLE_2_0

  conn_handle = Connection_Handle;
  int ret = aci_l2cap_connection_parameter_update_req(conn_handle,
                                                9 /* interval_min*/,
                                                9 /* interval_max */,
                                                0   /* slave_latency */,
                                                400 /*timeout_multiplier*/);
  
#if 0
  /* In order to use an iOS device as receiver, with the ST BlueMS app, please substitute the following function with the previous. */
  /* With iOS only the 8kHz (as audio sampling frequency) version is available */
  int ret = aci_l2cap_connection_parameter_update_req(conn_handle,
                                                8 /* interval_min*/,
                                                17 /* interval_max */,
                                                0   /* slave_latency */,
                                                400 /*timeout_multiplier*/);
#endif

#if 0
  /* In order to use an Android device version 4 as receiver, with audio sampling frequancy @16kHz, */
  /* please substitute the following function with the previous. */
  int ret = aci_l2cap_connection_parameter_update_req(conn_handle,
                                                8 /* interval_min*/,
                                                8 /* interval_max */,
                                                0   /* slave_latency */,
                                                400 /*timeout_multiplier*/);
#endif
  
  if (ret != BLE_STATUS_SUCCESS)
  {
      while (1) {
          PER_APP_Error_Handler();
      };
  }
  
  APP_PER_state = APP_STATUS_CONNECTED;
  
  EnableAll();
  
  BSP_LED_Off(LED2);
  ledActive = FALSE;
}/* end hci_le_connection_complete_event() */


/*******************************************************************************
 * Function Name  : hci_disconnection_complete_event_isr.
 * Description    : This event occurs when a connection is terminated.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_disconnection_complete_event(uint8_t Status,
                                          uint16_t Connection_Handle,
                                          uint8_t Reason)
{
  if(audio_streaming_active)
    BV_APP_StartStop_ctrl();
  audio_streaming_active = 0;
  
  /* Make the device connectable again. */
  BluevoiceADPCM_BNRG1_DisconnectionComplete_CB();
  
  FirstConnectionConfig = 0;
  
  APP_PER_state = APP_STATUS_ADVERTISEMENT;
#if (defined SENSIBLE_2_0) || (defined SENSIBLE_2_1)
  BSP_LED_Off(LED1);
#endif // SENSIBLE_2_0
  
  /*Set module in advertise mode*/
  APP_Status status = PER_APP_Advertise(); 
  if(status != APP_SUCCESS)
  {
    PER_APP_Error_Handler();
  }
  
  DeinitAll();
}/* end hci_disconnection_complete_event_isr() */


/*******************************************************************************
 * Function Name  : aci_gatt_attribute_modified_event.
 * Description    : This event occurs when an attribute is modified.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_attribute_modified_event(uint16_t Connection_Handle,
                                       uint16_t Attr_Handle,
                                       uint16_t Offset,
                                       uint16_t Attr_Data_Length,
                                       uint8_t Attr_Data[])
{
  if((Attr_Handle == tx_handle.CharAudioHandle+2) || (Attr_Handle == tx_handle.CharAudioSyncHandle+2))
  {
    if(Attr_Data[0] == 0x01)
    {
      if(!audio_streaming_active)
      {        
        BV_APP_StartStop_ctrl();
      }
      
      APP_PER_enabled.APP_BLUEVOICE_ENABLE = 1;
      BluevoiceADPCM_BNRG1_AttributeModified_CB(Attr_Handle, Attr_Data_Length, Attr_Data);
    }
    else if(Attr_Data[0] == 0x00)
    {
      if(APP_PER_enabled.APP_BLUEVOICE_ENABLE != 0)
      {
        APP_PER_enabled.APP_BLUEVOICE_ENABLE = 0;//APP_READY;
        if(audio_streaming_active)
        {
          BV_APP_StartStop_ctrl();
        }
      }
    }
  }
  
  if(Attr_Handle == AccHandle+2)
  {
    if(Attr_Data[0] == 0x01)
    {
      APP_PER_enabled.APP_INERTIAL_ENABLE = 1;

      INERTIAL_StartTimer();
    }
    else if(Attr_Data[0] == 0x00)
    {
      if(APP_PER_enabled.APP_INERTIAL_ENABLE != 0)
      {
        APP_PER_enabled.APP_INERTIAL_ENABLE = 0;//APP_READY;
      }
      INERTIAL_StopTimer();
    }
  }
  
#if (defined SENSIBLE_2_0) || (defined SENSIBLE_2_1)
  sensible_aci_gatt_attribute_modified_event(Connection_Handle,
                                             Attr_Handle,
                                             Offset,
                                             Attr_Data_Length,
                                             Attr_Data);
#endif // SENSIBLE_2_0
}

/*******************************************************************************
 * Function Name  : aci_gatt_tx_pool_available_event.
 * Description    : This event occurs when a TX pool available is received.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_tx_pool_available_event(uint16_t Connection_Handle,
                                      uint16_t Available_Buffers)
{       
  /* It allows to notify when at least 2 GATT TX buffers are available */
  tx_buffer_full = 0;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
