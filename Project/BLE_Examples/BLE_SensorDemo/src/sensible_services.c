/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "peripheral_mngr_app.h"
#include "steval_bluemic1.h" 
#include "sensible_services.h"
#include "sensible_sensors.h"
#include "steval_bluemic1_audio_in.h"
#include "sensible20_bat.h"
#include "HWAdvanceFeatures.h"
#include "BlueNRG1_flash.h"
#include "OTA.h"
#include "sensible20_compass.h"
#include "calculate_cct.h"
#include "main.h"

/* External symbols ----------------------------------------------------------*/
extern uint32_t FirstConnectionConfig;

volatile BOOL ForceReCalibration = FALSE;

extern volatile uint8_t audio_streaming_active;

/* Local symbols -------------------------------------------------------------*/
static uint16_t consoleHandle = 0;
static uint16_t termCharHandle = 0;
static uint16_t stdErrCharHandle = 0;
// static Service_UUID_t service_uuid;
static uint16_t EnvironmentalCharSize;
static uint16_t EnvironmentalCharHandle;
static uint16_t LedCharHandle;
static uint16_t LuxCharHandle;
static uint16_t VbatHandle;
static uint16_t AudioLevelCharHandle;
static uint16_t ColorAmbientLightHandle;

static uint16_t ConfigServW2STHandle;
static uint16_t ConfigCharHandle;

static uint16_t HWServW2STHandle;
//statis uint1
static uint16_t AccEventCharHandle;
static uint16_t ECompassCharHandle;

static uint8_t VbatState = 0;

/* Define the Max dimesion of the Bluetooth characteristics
for each packet used for Console Service */
#define W2ST_CONSOLE_MAX_CHAR_LEN 20

static uint8_t LastStderrBuffer[W2ST_CONSOLE_MAX_CHAR_LEN];
static uint8_t LastStderrLen;
static uint8_t LastTermBuffer[W2ST_CONSOLE_MAX_CHAR_LEN];
static uint8_t LastTermLen;
static uint32_t SizeOfUpdateBlueFW = 0;
static uint32_t bytesToWrite = 0;

static uint8_t BufferToWrite[256];
static uint32_t ConnectionBleStatus;

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#ifdef SENSIBLE_2_0
#define MCU_TYPE                "BlueNRG_1"
#define PACKAGE_NAME            "SensiBLE-2.0"
#define VERSION                 '2','3','1'
#elif defined SENSIBLE_2_1
#define MCU_TYPE                "BlueNRG_2"
#define PACKAGE_NAME            "SensiBLE-2.1"
#define VERSION                 '3','1','2'
#else
#error "SENSIBLE_2_0 or SENSIBLE_2_1 must be defined!"
#endif

/* Global symbols ------------------------------------------------------------*/

/* Macros --------------------------------------------------------------------*/
#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};

/* UUDIs ---------------------------------------------------------------------*/
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
    uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
    uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
    uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

/* Console Service */
#define COPY_CONSOLE_SERVICE_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,  0x00,0x0E,  0x11,0xe1,  0x9a,0xb4,  0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_TERM_CHAR_UUID(uuid_struct)               COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x01,  0x00,0x0E,  0x11,0xe1,  0xac,0x36,  0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_STDERR_CHAR_UUID(uuid_struct)             COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,  0x00,0x0E,  0x11,0xe1,  0xac,0x36,  0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_HW_SENS_W2ST_SERVICE_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ECOMPASS_W2ST_CHAR_UUID(uuid_struct)      COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x40,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ENVIRONMENTAL_W2ST_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_MIC_W2ST_CHAR_UUID(uuid_struct)           COPY_UUID_128(uuid_struct,0x04,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_COLOR_AMBIENT_LIGHT_CHAR_UUID(uuid_struct)COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x15,0x00,0x02,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_LED_W2ST_CHAR_UUID(uuid_struct)           COPY_UUID_128(uuid_struct,0x20,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_LUX_W2ST_CHAR_UUID(uuid_struct)           COPY_UUID_128(uuid_struct,0x01,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_BAT_W2ST_CHAR_UUID(uuid_struct)           COPY_UUID_128(uuid_struct,0x00,0x02,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ACC_EVENT_W2ST_CHAR_UUID(uuid_struct)     COPY_UUID_128(uuid_struct,0x00,0x00,0x04,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

/* Configuration Service */
#define COPY_CONFIG_SERVICE_UUID(uuid_struct)     COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x0F,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_CONFIG_W2ST_CHAR_UUID(uuid_struct)   COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0x00,0x0F,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

#define STORE_LE_16(buf, val)    ( ((buf)[0] =  (uint8_t) (val)    ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8) ) )

#define STORE_LE_32(buf, val)    ( ((buf)[0] =  (uint8_t) (val)     ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8)  ) , \
                                   ((buf)[2] =  (uint8_t) (val>>16) ) , \
                                   ((buf)[3] =  (uint8_t) (val>>24) ) )

#define STORE_BE_32(buf, val)    ( ((buf)[3] =  (uint8_t) (val)     ) , \
                                   ((buf)[2] =  (uint8_t) (val>>8)  ) , \
                                   ((buf)[1] =  (uint8_t) (val>>16) ) , \
                                   ((buf)[0] =  (uint8_t) (val>>24) ) )

//OTA defines begin
/*************** Don't Change the following defines *************/
/* Define the symbol used for defining each termination string
used in Console service */
#define W2ST_CONSOLE_END_STRING "\0"

/* Standard Terminal */
#define W2ST_CONNECT_STD_TERM      (1<<10)
/* Standard Error */
#define W2ST_CONNECT_STD_ERR       (1<<11)
/* HW Advance Features */
#define W2ST_CONNECT_ACC_EVENT     (1<<12)

/* W2ST command - BF type */
#define W2ST_COMMAND_BF_TYPE       0xCC
/* W2ST command - BF ASR_READY */
#define W2ST_COMMAND_BF_ASR_READY   0x00
/* W2ST command - BF Strong */
#define W2ST_COMMAND_BF_STRONG  0x01

/* W2ST command - BF toggle */
#define W2ST_COMMAND_BF_TOGGLE 0xAA
/* W2ST command - toggle BF On */
#define W2ST_COMMAND_BF_OFF   0x00
/* W2ST command - toggle BF Off */
#define W2ST_COMMAND_BF_ON  0x01

/* W2ST command - BF direction */
#define W2ST_COMMAND_BF_CHANGEDIR 0xBB
/* W2ST command - BF direction 1 */
#define W2ST_COMMAND_BF_DIR1  (uint8_t) 1
/* W2ST command - BF direction 2 */
#define W2ST_COMMAND_BF_DIR2  (uint8_t) 2
/* W2ST command - BF direction 3 */
#define W2ST_COMMAND_BF_DIR3  (uint8_t) 3
/* W2ST command - BF direction 4 */
#define W2ST_COMMAND_BF_DIR4  (uint8_t) 4
/* W2ST command - BF direction 5 */
#define W2ST_COMMAND_BF_DIR5  (uint8_t) 5
/* W2ST command - BF direction 6 */
#define W2ST_COMMAND_BF_DIR6  (uint8_t) 6
/* W2ST command - BF direction 7 */
#define W2ST_COMMAND_BF_DIR7  (uint8_t) 7
/* W2ST command - BF direction 8 */
#define W2ST_COMMAND_BF_DIR8  (uint8_t) 8

/* BLE Characteristic connection control */
/* Environmental Data */
#define W2ST_CONNECT_ENV           (1   )

////////////////////////////////////////////////////////////////////////////////
#define W2ST_CHECK_CONNECTION(BleChar) ((ConnectionBleStatus&(BleChar)) ? 1 : 0)
#define W2ST_ON_CONNECTION(BleChar)    (ConnectionBleStatus|=(BleChar))
#define W2ST_OFF_CONNECTION(BleChar)   (ConnectionBleStatus&=(~BleChar))
//OTA defines end

#define AUDIO_VOLUME_VALUE       64
#define AUDIO_CHANNELS           1
/* Define The transmission interval in Multiple of 10ms for Microphones dB Values */
#define MICS_DB_UPDATE_MUL_10MS 5

volatile float RMS_Ch[AUDIO_CHANNELS];
float DBNOISE_Value_Old_Ch[AUDIO_CHANNELS];

/* Local functions prototypes ------------------------------------------------*/
static tBleStatus Environmental_Update(int32_t Press,uint16_t Hum,int16_t Temp1);//,int16_t Temp2);
static tBleStatus Lux_Update(uint16_t LuxValue);
static tBleStatus ColorAmbient_Update(uint32_t lux, uint16_t cct, uint16_t uv);

static uint8_t DebugConsoleCommandParsing(uint8_t * att_data, uint8_t data_length);
static void processTermWrite(uint8_t * att_data, uint8_t data_length);
static void processUpgradeFw(uint8_t * att_data);
static tBleStatus Stderr_Update(uint8_t *data,uint8_t length);
static tBleStatus Term_Update_AfterRead(void);
static tBleStatus Stderr_Update_AfterRead(void);
static tBleStatus Term_Update(uint8_t *data, uint8_t length);
static uint32_t ConfigCommandParsing(uint8_t * att_data, uint8_t data_length);

/**
 * @brief  This event is given when a read request is received 
 *         by the server from the client.
 */
void aci_gatt_read_permit_req_event(uint16_t Connection_Handle,
                                    uint16_t Attribute_Handle,
                                    uint16_t Offset)
{
    if (Attribute_Handle == termCharHandle + 1) {
        /* Send again the last packet for Terminal */
        Term_Update_AfterRead();
    } else if (Attribute_Handle == stdErrCharHandle + 1) {
        /* Send again the last packet for StdError */
        Stderr_Update_AfterRead();
    } else if(Attribute_Handle == AccEventCharHandle + 1) {
        /* Read Request for Pedometer */
        uint16_t StepCount;
        if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER)) {
            StepCount = GetStepSWPedometer();
        } else {
            StepCount = 0;
        }
        AccEventSteps_Notify(ACC_PEDOMETER, StepCount);
        //Only for ST BLE Sensor Classic
        AccEventSteps_Notifi(StepCount);
    } else if (Attribute_Handle == EnvironmentalCharHandle + 1) {
    	EnvironmentalUpdate();
    } else if(Attribute_Handle == LedCharHandle + 1){
        /* Read Request for Led Status */
        LedUpdate(LED1);
    } else if(Attribute_Handle == LuxCharHandle + 1){
    	LuxUpdate();
    } else if(Attribute_Handle == VbatHandle + 1){
    	BatUpdate();
    } else if(Attribute_Handle == ColorAmbientLightHandle + 1){
    	ColorAmbientLightUpdate();
    }

    //EXIT:
    if(Connection_Handle != 0) {
    	aci_gatt_allow_read(Connection_Handle);
	}
}

/**
 * @brief This function handles attribure modified ebvetns for sensible 2.0
 *        characteristics
 */
void sensible_aci_gatt_attribute_modified_event(uint16_t Connection_Handle,
                                                uint16_t Attr_Handle,
                                                uint16_t Offset,
                                                uint8_t Attr_Data_Length,
                                                uint8_t Attr_Data[])
{

    /* ---------------------------------------------------------------------- */
    if(Attr_Handle == LedCharHandle + 2) {
    /* Switch fucntion on/off */
        if(Attr_Data[0] == 0x01) {
            /* Switch on */
            APP_PER_enabled.APP_LED_ENABLE = 1;
            //BSP_LED_On(LED1);
        } else if(Attr_Data[0] == 0x00) {
            /* Switch off */
            APP_PER_enabled.APP_LED_ENABLE = 0;
            //BSP_LED_Off(LED1);
        }
    } else if(Attr_Handle == VbatHandle + 2) {
    /* ---------------------------------------------------------------------- */
    /* Environmental sensors enable / disable command */
        if(Attr_Data[0] == 0x01) {
            VbatState = 1;
            APP_PER_enabled.APP_BAT_ENABLE = 1;
            BSP_Audio_IN_DeInit();
            BSP_BatLevel_IN_Init();
        } else if(Attr_Data[0] == 0x00) {
            VbatState = 0;
            APP_PER_enabled.APP_BAT_ENABLE = 0;
            BSP_BatLevel_IN_DeInit();
            BSP_AUDIO_IN_Init(AUDIO_SAMPLING_FREQUENCY);
        }
    } else if (Attr_Handle == AudioLevelCharHandle + 2) {
    /* ---------------------------------------------------------------------- */
    /* Mic level enable / disable command */
        if (Attr_Data[0] == 01)
        {
            APP_PER_enabled.APP_MIC_LEVEL_ENABLE = 1;
            for(int32_t Count=0;Count<AUDIO_CHANNELS;Count++)
            {
                RMS_Ch[Count]=0;
                DBNOISE_Value_Old_Ch[Count] =0;
            }

            if(!audio_streaming_active)
            {        
                BV_APP_StartStop_ctrl();
            }
            
            BluevoiceADPCM_BNRG1_AttributeModified_CB(Attr_Handle, Attr_Data_Length, Attr_Data);

#if 0
            /* Start the TIM Base generation in interrupt mode */
            if(HAL_TIM_Base_Start_IT(&TimAudioDataHandle) != HAL_OK)
            {
                /* Starting Error */
                Error_Handler();
            }
#endif
        } else if (Attr_Data[0] == 0)
        {
            APP_PER_enabled.APP_MIC_LEVEL_ENABLE = 0;
            
            if(audio_streaming_active)
            {
                BV_APP_StartStop_ctrl();
            }

#if 0
            /* Stop the TIM Base generation in interrupt mode */
            if(HAL_TIM_Base_Stop_IT(&TimAudioDataHandle) != HAL_OK)
            {
                /* Stopping Error */
                Error_Handler();
            }
#endif
        }
    } else if(Attr_Handle == EnvironmentalCharHandle + 2) {
    /* ---------------------------------------------------------------------- */
    /* Environmental sensors enable / disable command */
        if(Attr_Data[0] == 0x01) {
            APP_PER_enabled.APP_ENV_ENABLE = 1;// SensorsEnable();
        } else if(Attr_Data[0] == 0x00) {
            APP_PER_enabled.APP_ENV_ENABLE = 0;// SensorsDisable();
        }
    } else if(Attr_Handle == ColorAmbientLightHandle + 2) {
    	 /* APP_COLOR_AMBIENT sensors enable / disable command */
        if(Attr_Data[0] == 0x01) {
            APP_PER_enabled.APP_COLOR_AMBIENT = 1;
        } else if(Attr_Data[0] == 0x00) {
            APP_PER_enabled.APP_COLOR_AMBIENT = 0;
        }
    } else if(Attr_Handle == LuxCharHandle + 2) {
	/* ---------------------------------------------------------------------- */
	/* LUX sensors enable / disable command */
		if(Attr_Data[0] == 0x01) {
			APP_PER_enabled.APP_LUX_ENABLE = 1;
		} else if(Attr_Data[0] == 0x00) {
			APP_PER_enabled.APP_LUX_ENABLE = 0;
		}
    } else if(Attr_Handle == ConfigCharHandle + 1) {
    /* ---------------------------------------------------------------------- */
    /* Configuration received */
		ConfigCommandParsing(Attr_Data, Attr_Data_Length);
    } else if(Attr_Handle == ConfigCharHandle + 2) {
        if (Attr_Data[0] == 01) {
            FirstConnectionConfig = 1;
        } else if (Attr_Data[0] == 0){
            FirstConnectionConfig = 0;
        }
    } else if(Attr_Handle == termCharHandle + 2){
    /* ---------------------------------------------------------------------- */
    /* OTA */
        //It is used for OTA
        if (Attr_Data[0] == 01) {
            W2ST_ON_CONNECTION(W2ST_CONNECT_STD_TERM);
        } else if (Attr_Data[0] == 0){
            W2ST_OFF_CONNECTION(W2ST_CONNECT_STD_TERM);
        }
    }  else if (Attr_Handle == termCharHandle + 1){
        //It is used for OTA
        processTermWrite(Attr_Data, Attr_Data_Length);
    } else if(Attr_Handle == stdErrCharHandle + 2){
        if (Attr_Data[0] == 01) {
            W2ST_ON_CONNECTION(W2ST_CONNECT_STD_ERR);
        } else if (Attr_Data[0] == 0){
            W2ST_OFF_CONNECTION(W2ST_CONNECT_STD_ERR);
        }
    } else if(Attr_Handle == AccEventCharHandle + 2){
    /* ---------------------------------------------------------------------- */
		if (Attr_Data[0] == 01) {
			ResetSWPedometer();
			Config_Notify(FEATURE_MASK_ACC_EVENTS, 'o', 1);
		} else if (Attr_Data[0] == 0) {
			DisableHWMultipleEvents();
		}
    } else if(Attr_Handle == ECompassCharHandle + 2){
    /* ---------------------------------------------------------------------- */
    /* eCompass */
        if (Attr_Data[0] == 0x01) {
            BSP_Compass_Start();
        } else if (Attr_Data[0] == 0x00){
            BSP_Compass_Stop();
        }
    }
}

/**
* @brief  User function that is called when 1 ms of PDM data is available.
* @param  none
* @retval None
*/
void AudioProcess(uint16_t* PCM_Buffer)
{
    if (APP_PER_enabled.APP_MIC_LEVEL_ENABLE != 0)
    {
        for(int32_t i = 0; i < PCM_BUFFER_SIZE/2; i++)
        {
            for(int32_t NumberMic=0;NumberMic<AUDIO_CHANNELS;NumberMic++)
            {
                RMS_Ch[NumberMic] += (float)((int16_t)PCM_Buffer[i*AUDIO_CHANNELS+NumberMic]
                                             * ((int16_t)PCM_Buffer[i*AUDIO_CHANNELS+NumberMic]));
            }
        }
    }
}

void MicLevelUpdate(void)
{
    uint16_t DBNOISE_Value_Ch[AUDIO_CHANNELS];
    uint8_t buff[2+1*(AUDIO_CHANNELS)];
    
    for(int32_t NumberMic=0;NumberMic<(AUDIO_CHANNELS);NumberMic++) {
        DBNOISE_Value_Ch[NumberMic] = 0;
        
        RMS_Ch[NumberMic] /= (16.0f*MICS_DB_UPDATE_MUL_10MS*10);
        
        DBNOISE_Value_Ch[NumberMic] = (uint16_t)((120.0f - 20 * log10f(32768 * (1 + 0.25f * (AUDIO_VOLUME_VALUE /*AudioInVolume*/ - 4))) + 10.0f * log10f(RMS_Ch[NumberMic])) * 0.3f + DBNOISE_Value_Old_Ch[NumberMic] * 0.7f);
        DBNOISE_Value_Old_Ch[NumberMic] = DBNOISE_Value_Ch[NumberMic];
        RMS_Ch[NumberMic] = 0.0f;
    }
    
    STORE_LE_16(buff,(tick_ms()>>3));
    for(uint16_t Counter=0;Counter<(AUDIO_CHANNELS);Counter++)
    {
        buff[2+Counter]= DBNOISE_Value_Ch[Counter]&0xFF;
    }
    aci_gatt_update_char_value(HWServW2STHandle, AudioLevelCharHandle, 0, sizeof(buff), buff);
}

/**
 * @brief Update all charachteristics
 */
void UpdateAll(void)
{
    if(VbatState == 0) {
        EnvironmentalUpdate();
        LuxUpdate();
        LedUpdate(LED2);
    } else {
    	BatUpdate();
    }
}

/**
 * @brief  Add service and charachteristic for OTA
 * @param  
 * @retval tBleStatus Status
 */
tBleStatus Add_Console_Service(void)
{
    volatile tBleStatus ret;
    uint8_t uuid[16];

    COPY_CONSOLE_SERVICE_UUID(uuid);
    ret = aci_gatt_add_service(UUID_TYPE_128,       //UUID type
                            (Service_UUID_t*)&uuid, //Service_UUID_t
                            PRIMARY_SERVICE,        //Service type
                            1+3*2,                  //Maximum number of attribute records that can 
                                                    //be added to this service.
                                                    //1 + if(read|write) + 2 for every char,
                                                    //if(notify|indicate) + 3 for every char
                            &consoleHandle);        //Handle of the Service
    
    if (ret != BLE_STATUS_SUCCESS) {
        goto fail;
    }

    COPY_TERM_CHAR_UUID(uuid);
    ret = aci_gatt_add_char(consoleHandle,              //Service_Handle
                            UUID_TYPE_128,              //UUID type
                            (Char_UUID_t*)&uuid,        //Char_UUID_t
                            W2ST_CONSOLE_MAX_CHAR_LEN,  //Maximum length of the characteristic value
                            CHAR_PROP_NOTIFY | 
                            CHAR_PROP_WRITE_WITHOUT_RESP |
                            CHAR_PROP_WRITE |
                            CHAR_PROP_READ,             //Char_Properties
                            ATTR_PERMISSION_NONE,       //Security permission flags
                            GATT_NOTIFY_ATTRIBUTE_WRITE | 
                            GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,//event mask
                            0x10,                       //Enc_Key_Size
                            1,                          //Is_Variable length, 0 - Fixed length, 1 - Variable length
                            &termCharHandle);           //Char_Handle
    
    if (ret != BLE_STATUS_SUCCESS) {
        goto fail;
    }

    COPY_STDERR_CHAR_UUID(uuid);
    ret = aci_gatt_add_char(consoleHandle,              //Service_Handle
                            UUID_TYPE_128,              //UUID type
                            (Char_UUID_t*)&uuid,        //Char_UUID_t
                            W2ST_CONSOLE_MAX_CHAR_LEN,  //Maximum length of the characteristic value
                            CHAR_PROP_NOTIFY | 
                            CHAR_PROP_READ,             //Char_Properties
                            ATTR_PERMISSION_NONE,       //Security permission flags
                            GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,//event mask
                            0x10,                       //Enc_Key_Size
                            1,                          //Is_Variable length, 0 - Fixed length, 1 - Variable length
                            &stdErrCharHandle);         //Char_Handle

    if (ret != BLE_STATUS_SUCCESS){
        goto fail;
    }

    return BLE_STATUS_SUCCESS;

fail:
    return BLE_STATUS_ERROR;
}

/**
 * @brief  Add the HW Features service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_HW_SW_ServW2ST_Service(void)
{
  tBleStatus ret;
  uint8_t uuid[16];

  COPY_HW_SENS_W2ST_SERVICE_UUID(uuid);

  ret = aci_gatt_add_service(UUID_TYPE_128,           //Service_UUID_Type
							(Service_UUID_t*)uuid,    //Service_UUID
							PRIMARY_SERVICE,          //Service_Type
							1+3*11,                    //Max_Attribute_Records that can be added
													  // to this service.
													  //1 + if(read|write) + 2 for every char,
													  //if(notify|indicate) + 3 for every char
							&HWServW2STHandle);      //Service_Handle

  if (ret != BLE_STATUS_SUCCESS){
	  goto fail;
  }
  
  /* Fill the Environmental BLE Characteristc */
  /* ************************ */
  COPY_ENVIRONMENTAL_W2ST_CHAR_UUID(uuid);
  
  uuid[14] |= 0x04; /* One Temperature value*/ //5
  uuid[14] |= 0x08; /* Humidity */
  uuid[14] |= 0x10; /* Pressure value*/

  EnvironmentalCharSize = 10;// if second temperature - 12;

  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, (Char_UUID_t*)uuid, EnvironmentalCharSize,
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &EnvironmentalCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  /* ************************ */
  COPY_LUX_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, (Char_UUID_t*)uuid, 2+2,
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &LuxCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  /* ************************ */
  COPY_LED_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, (Char_UUID_t*)uuid, 2+1,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &LedCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  /* ************************ */
  COPY_BAT_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, (Char_UUID_t*)uuid, 2+2+2+2+1,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &VbatHandle);
    
  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  /* ************************ */
  /* Mic level characteristic */
  COPY_MIC_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, (Char_UUID_t*)uuid,2+AUDIO_CHANNELS,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &AudioLevelCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  /* ************************ */
  COPY_ACC_EVENT_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle,          //Service_Handle
							UUID_TYPE_128,              //UUID type
							(Char_UUID_t*)uuid,         //Char_UUID_t
							2+3,                        //Maximum length of the characteristic value
							CHAR_PROP_NOTIFY |
							CHAR_PROP_READ,             //Char_Properties
							ATTR_PERMISSION_NONE,       //Security permission flags
							GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,//event mask
							16,                         //Enc_Key_Size
							1,                          //Is_Variable length, 0 - Fixed length, 1 - Variable length
							&AccEventCharHandle);       //Char_Handle

  if (ret != BLE_STATUS_SUCCESS){
	  goto fail;
  }

  /* ************************ */
  COPY_ECOMPASS_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle,          //Service_Handle
							UUID_TYPE_128,              //UUID type
							(Char_UUID_t*)uuid,         //Char_UUID_t
							2+2,                        //Maximum length of the characteristic value
							CHAR_PROP_NOTIFY,           //Char_Properties
							ATTR_PERMISSION_NONE,       //Security permission flags
							GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,//event mask
							16,                         //Enc_Key_Size
							0,                          //Is_Variable length, 0 - Fixed length, 1 - Variable length
							&ECompassCharHandle);       //Char_Handle

  if (ret != BLE_STATUS_SUCCESS) {
	  goto fail;
  }

  /* ************************ */
  COPY_COLOR_AMBIENT_LIGHT_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, (Char_UUID_t*)uuid,2+4+2+2,
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &ColorAmbientLightHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  BV_APP_Status BV_status = BV_APP_add_char(HWServW2STHandle);

  if(BV_status != BV_APP_SUCCESS)
  {
	  PRINTF("Error while adding BV_APP_add_char(ServiceHandle).\n");
	  return APP_ERROR;
  }

  INERTIAL_APP_Status IN_status = INERTIAL_APP_add_char(HWServW2STHandle);

  if(IN_status != INERTIAL_APP_SUCCESS)
  {
	  PRINTF("Error while adding char INERTIAL_APP_add_char(ServiceHandle).\n");
	  return APP_ERROR;
  }

  /* ************************ */
  COPY_CONFIG_SERVICE_UUID(uuid);
  ret = aci_gatt_add_service(UUID_TYPE_128,  (Service_UUID_t*)uuid, PRIMARY_SERVICE, 1+3,&ConfigServW2STHandle);
  
  if (ret != BLE_STATUS_SUCCESS)
    goto fail;

  COPY_CONFIG_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(ConfigServW2STHandle, UUID_TYPE_128, (Char_UUID_t*)uuid, 20 /* Max Dimension */,
                           CHAR_PROP_NOTIFY| CHAR_PROP_WRITE_WITHOUT_RESP,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &ConfigCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  /* ************************ */
                           
  return BLE_STATUS_SUCCESS;

fail:
  return BLE_STATUS_ERROR ;

}

/**
 * @brief Get value HWServW2STHandle
 */
uint16_t GetHWServW2STHandle(void)
{
	return HWServW2STHandle;
}

/**
 * @brief Update battery level to the application
 */
tBleStatus BatUpdate(void)
{
    uint32_t soc = 0;
    uint32_t voltage = 0;
    int32_t current = 0;
    uint8_t buff[2+2+2+2+1];
    
    BSP_BatLevel_GetValues(&soc, &voltage, &current);

    STORE_LE_16(buff, tick_ms() >> 3);
    STORE_LE_16(buff+2, soc);
    STORE_LE_16(buff+4, voltage);
    STORE_LE_16(buff+6, current);
    // Make % from, i.e. 15 instead 153 (which means 15.3%)
    soc /= 10;
    if(soc<15) {
        /* if it's < 15% Low Battery*/
        buff[8] = 0x00; /* Low Battery */
    } else {
        static uint32_t PreVoltage = 0;
        static uint32_t Status = 0x04; /* Unknown */
        if(PreVoltage != 0) {
            if(PreVoltage > voltage){
                PreVoltage = voltage;
                Status = 0x01; /* Discharging */
            } else if(PreVoltage < voltage){
                PreVoltage = voltage;
                Status = 0x03; /* Charging */
            }
        } else {
            PreVoltage = voltage;
        }

        buff[8] = Status;
    }

    return aci_gatt_update_char_value(HWServW2STHandle, VbatHandle, 0, sizeof(buff),buff);
}

/**
 * @brief Update led state to application
 */
tBleStatus LedUpdate(Led_TypeDef led)
{
	uint8_t buff[2+1];
	STORE_LE_16(buff  ,3);
	buff[2] = BSP_LED_IsOn(led);
	return aci_gatt_update_char_value(HWServW2STHandle, LedCharHandle, 0, 2+1,buff);
}

/**
 * @brief  Read light sensor value and send via BLE
 * @param
 * @retval tBleStatus Status
 */
tBleStatus LuxUpdate(void)
{
    uint16_t lux = 0;

    SensorsReadLux(&lux);
    return Lux_Update(lux);
}

tBleStatus ColorAmbientLightUpdate(void)
{
    uint16_t lux = 0;
    uint16_t cct = 0;
    int16_t uv = 0;

    SensorsReadLux(&lux);
    cct = get_cct();
    SensorsReadUv(&uv);

    if (uv < 0) uv = 0;

    return ColorAmbient_Update(lux, cct, uv);
}

/**
 * @brief  Send a notification When the DS3 detects one Acceleration event
 * @param  Acceleration event and steps to Send
 * @retval tBleStatus Status
 */
tBleStatus AccEventSteps_Notify(uint8_t event, uint16_t steps)
{
    tBleStatus ret= BLE_STATUS_SUCCESS;
    uint8_t buff[2+3] = {0};

	STORE_LE_16(buff, (tick_ms()>>3));
	buff[2]= event;
	STORE_LE_16(buff+3,steps);
	ret = aci_gatt_update_char_value(HWServW2STHandle, AccEventCharHandle, 0, sizeof(buff),buff);

    return ret;
}

/**
 * @brief  Send a steps When the DS3 detects
 * @param  Steps to Send
 * @retval tBleStatus Status
 */
tBleStatus AccEvent_Notifi(uint8_t event)
{
	tBleStatus ret= BLE_STATUS_SUCCESS;
	uint8_t buff[2+1] = {0};

	STORE_LE_16(buff, (tick_ms()>>3));
	buff[2]= event;
	ret = aci_gatt_update_char_value(HWServW2STHandle, AccEventCharHandle, 0, sizeof(buff),buff);

	return ret;
}

/**
 * @brief  Send a steps When the DS3 detects
 * @param  Steps to Send
 * @retval tBleStatus Status
 */
tBleStatus AccEventSteps_Notifi(uint16_t steps)
{
	tBleStatus ret= BLE_STATUS_SUCCESS;
	uint8_t buff[2+2] = {0};

	STORE_LE_16(buff, (tick_ms()>>3));
	STORE_LE_16(buff+2,steps);
	ret = aci_gatt_update_char_value(HWServW2STHandle, AccEventCharHandle, 0, sizeof(buff),buff);

	return ret;
}

/* @brief  Send a notification for answering to a configuration command for Accelerometer events
 * @param  uint32_t Feature Feature calibrated
 * @param  uint8_t Command Replay to this Command
 * @param  uint8_t data result to send back
 * @retval tBleStatus Status
 */
tBleStatus Config_Notify(uint32_t Feature, uint8_t Command, uint8_t data)
{
    uint8_t buff[2+4+1+1];

    STORE_LE_16(buff, (tick_ms()>>3));
    STORE_BE_32(buff+2, Feature);
    buff[6] = Command;
    buff[7] = data;

    volatile tBleStatus ret= aci_gatt_update_char_value(ConfigServW2STHandle, ConfigCharHandle, 0, 8,buff);
    
    if(ret == BLE_STATUS_SUCCESS){
        return BLE_STATUS_SUCCESS;
    } else {
        return BLE_STATUS_ERROR;
    }
    
}

/**
 * @brief  Update E-Compass characteristic value
 * @param  uint16_t Angle To Magnetic North in cents of degree [0 -> 35999]
 * @retval tBleStatus      Status
 */
tBleStatus ECompass_Update(uint16_t Angle)
{
    uint8_t buff[2+ 2];
    
    if(Angle > 35999){
        return BLE_STATUS_ERROR;
    }

    STORE_LE_16(buff, (tick_ms()>>3));
    STORE_LE_16(buff+2,Angle);

    return aci_gatt_update_char_value(HWServW2STHandle, ECompassCharHandle, 0, 2+2,buff);
}

/**
 * @brief  Read environment sensors values and send them via BLE
 * @param  
 * @retval tBleStatus Status
 */
tBleStatus EnvironmentalUpdate(void)
{
    int32_t decPart, intPart;
    float press;
    float temp1;
    float hum;
//    float temp2;
    
    SesnsorsReadTemp1(&temp1);
//    SensorsReadTemp2(&temp2);
    SesnorsReadHumidity(&hum);
    SensorsReadPressure(&press);
      
    MCR_BLUEMS_F2I_2D(press, intPart, decPart);
    int32_t PressToSend=intPart*100+decPart;
    MCR_BLUEMS_F2I_1D(hum, intPart, decPart);
    uint16_t HumToSend = intPart*10+decPart;
    MCR_BLUEMS_F2I_1D(temp1, intPart, decPart);
    int16_t Temp1ToSend = intPart*10+decPart;
//    MCR_BLUEMS_F2I_1D(temp2, intPart, decPart);
//    int16_t Temp2ToSend = intPart*10+decPart;
        
    return Environmental_Update(PressToSend,
                                HumToSend,
                                Temp1ToSend);//, Temp2ToSend);
}

/**
 * @brief  Update environmental data
 * @param  
 * @retval tBleStatus Status
 */
static tBleStatus Environmental_Update(int32_t Press,uint16_t Hum,int16_t Temp1)//,int16_t Temp2)
{
  tBleStatus ret;
  uint8_t BuffPos;

  uint8_t buff[2+4/*Press*/+2/*Hum*/+2/*Temp1*/];//+2/*Temp2*/];
  
  static uint16_t time = 0;
  
  time += 3;
  STORE_LE_16(buff  , time);
  BuffPos=2;

  STORE_LE_32(buff+BuffPos,Press);
  BuffPos+=4;

  STORE_LE_16(buff+BuffPos,Hum);
  BuffPos+=2;
  
//  STORE_LE_16(buff+BuffPos,Temp2);
//  BuffPos+=2;
  
  STORE_LE_16(buff+BuffPos,Temp1);
  BuffPos+=2;
  
  ret = aci_gatt_update_char_value(HWServW2STHandle, EnvironmentalCharHandle, 0, EnvironmentalCharSize,buff);

  if (ret != BLE_STATUS_SUCCESS){
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update LLux characteristic value
 * @param  uint16_t Value in Lux
 * @retval tBleStatus   Status
 */
static tBleStatus Lux_Update(uint16_t LuxValue)
{
  tBleStatus ret;
  uint8_t buff[2+8] = {0};

  static uint16_t time = 0;

  time += 3;
  STORE_LE_16(buff  ,(time));
  STORE_LE_16(buff+2,LuxValue);

  ret = aci_gatt_update_char_value(HWServW2STHandle, LuxCharHandle, 0, 2+2,buff);

  if (ret != BLE_STATUS_SUCCESS){
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update CO characteristic value
 * @param  uint16_t Value in Lux
 * @retval tBleStatus   Status
 */
tBleStatus ColorAmbient_Update(uint32_t lux, uint16_t cct, uint16_t uv)
{
  tBleStatus ret;
  uint8_t buff[2+4+2+2];
  uint8_t pos = 0;
  static uint16_t time = 0;

  time += 3;

  STORE_LE_16(&buff[pos], time);
  pos += 2;
  STORE_LE_32(&buff[pos], lux);
  pos += 4;
  STORE_LE_16(&buff[pos], cct);
  pos += 2;
  STORE_LE_16(&buff[pos], uv);

  ret = aci_gatt_update_char_value(HWServW2STHandle, ColorAmbientLightHandle, 0, sizeof(buff),buff);

  if (ret != BLE_STATUS_SUCCESS){
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}


/**
 * @brief  Update Terminal characteristic value
 * @param  data  string to write
 * @param  length length of string to write
 * @retval tBleStatus      Status
 */
tBleStatus Term_Update(uint8_t *data, uint8_t length)
{
    tBleStatus ret;
    uint8_t Offset;
    uint8_t DataToSend;

    /* Split the code in packages */
    for(Offset =0; Offset<length; Offset +=W2ST_CONSOLE_MAX_CHAR_LEN){
        DataToSend = (length-Offset);
        DataToSend = (DataToSend>W2ST_CONSOLE_MAX_CHAR_LEN) ?  W2ST_CONSOLE_MAX_CHAR_LEN : DataToSend;

        /* keep a copy */
        memcpy(LastTermBuffer,data+Offset,DataToSend);
        LastTermLen = DataToSend;

        ret = aci_gatt_update_char_value(
            consoleHandle, 
            termCharHandle, 
            0, 
            DataToSend, 
            data+Offset);
        if (ret != BLE_STATUS_SUCCESS) {
            //Error Updating Stdout Char
            return BLE_STATUS_ERROR;
        }
        
        DelayMs(20);
    }

    return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Terminal characteristic value after a read request
 * @param  None
 * @retval tBleStatus      Status
 */
static tBleStatus Term_Update_AfterRead(void)
{
    tBleStatus ret;

    ret = aci_gatt_update_char_value(
        consoleHandle, 
        termCharHandle, 
        0, 
        LastTermLen,
        LastTermBuffer);
    if (ret != BLE_STATUS_SUCCESS) {
        if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
            bytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Stdout Char\n");
            Stderr_Update(BufferToWrite, bytesToWrite);
        } else {
            //Error Updating Stdout Char
        }
        return BLE_STATUS_ERROR;
    }

    return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Stderr characteristic value
 * @param  data  string to write
 * @param  length length of string to write
 * @retval tBleStatus      Status
 */
static tBleStatus Stderr_Update(uint8_t *data,uint8_t length)
{
    tBleStatus ret;
    uint8_t Offset;
    uint8_t DataToSend;

    /* Split the code in packages*/
    for(Offset =0; Offset < length; Offset += W2ST_CONSOLE_MAX_CHAR_LEN){
        DataToSend = (length-Offset);
        DataToSend = (DataToSend>W2ST_CONSOLE_MAX_CHAR_LEN) ?  W2ST_CONSOLE_MAX_CHAR_LEN : DataToSend;

        /* keep a copy */
        memcpy(LastStderrBuffer,data+Offset,DataToSend);
        LastStderrLen = DataToSend;

        ret = aci_gatt_update_char_value(
            consoleHandle, 
            stdErrCharHandle, 
            0, 
            DataToSend, 
            data+Offset);
        if (ret != BLE_STATUS_SUCCESS) {
            return BLE_STATUS_ERROR;
        }
        DelayMs(10);
    }

    return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Stderr characteristic value after a read request
 * @param  None
 * @retval tBleStatus      Status
 */
static tBleStatus Stderr_Update_AfterRead(void)
{
    tBleStatus ret;

    ret = aci_gatt_update_char_value(
        consoleHandle, 
        stdErrCharHandle, 
        0, 
        LastStderrLen, 
        LastStderrBuffer);
    if (ret != BLE_STATUS_SUCCESS) {
        return BLE_STATUS_ERROR;
    }

    return BLE_STATUS_SUCCESS;
}

/**
 * @brief  This function makes the parsing of the Debug Console Commands
 * @param  att_data attribute data
 * @param  data_length length of the data
 * @retval SendItBack true/false
 */
static uint8_t DebugConsoleCommandParsing(uint8_t * att_data, uint8_t data_length)
{
    uint8_t SendBackData = 1;
    
    if(!strncmp("versionFw",(char *)(att_data),9)) {
        bytesToWrite = sprintf((char *)BufferToWrite,"%s_%s_%c.%c.%c\r\n",
                            MCU_TYPE, PACKAGE_NAME, VERSION);
        Term_Update(BufferToWrite,bytesToWrite);
        SendBackData=0;
        
    } else if(!strncmp("upgradeFw",(char *)(att_data),9)) {
        processUpgradeFw(att_data);
        SendBackData=0;
    
    } else if((att_data[0]=='u') & (att_data[1]=='i') & (att_data[2]=='d')) {
        //The address of Unique device serial number in BlueNRG-1. It contains 
        //only 6 bytes of unique ID but we need 12 bytes.
        uint8_t *uid = (uint8_t*)0x100007F4;
        //This is ID from STM32L476
        uint32_t MCU_ID = 0x415;
        bytesToWrite = sprintf((char *)BufferToWrite,"%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X_%.3lX\r\n",
                            uid[3], uid[2], uid[1], uid[0],
                            uid[1], uid[0], uid[5], uid[4],
                            uid[5], uid[4], uid[3], uid[2],
                            MCU_ID);
        Term_Update(BufferToWrite, bytesToWrite);
        SendBackData=0;
    }

    return SendBackData;
}

/**
 * @brief  Process Terminal write request
 * @param  data  string to write
 * @param  length length of string to write
 * @retval none
 */
static void processTermWrite(uint8_t * att_data, uint8_t data_length)
{
    /* By default Answer with the same message received */
    uint8_t SendBackData = 1;
    if(SizeOfUpdateBlueFW != 0) {
        /* FP-SNS-ALLMEMS1 firwmare update */
        int8_t RetValue = UpdateFWBlueMS(&SizeOfUpdateBlueFW, att_data, data_length);
        if(RetValue != 0) {
            aci_gatt_update_char_value(consoleHandle, termCharHandle, 0, 1, ((uint8_t *)&RetValue));

            if(RetValue == 1) {
                /* if OTA checked */
                bytesToWrite = sprintf((char *)BufferToWrite,"Success. Restarting..\n");
                Term_Update(BufferToWrite,bytesToWrite);
                for(uint8_t i = 0; i < 100; i++){
                    BTLE_StackTick();
                }
                
                //Write command for bootloader for the beginning programming operation
                FLASH_ErasePage(BOOTLOADER_CONFIG_PAGE_NUM);
                FLASH_ProgramWord(BOOTLOADER_CONFIG_PAGE_ADDR, BOOTLOADER_START_UPDATE_COMMAND);
                //restart in 5 seconds
                DelayMs(5000);
                NVIC_SystemReset();
            }
        }
        SendBackData=0;
    } else {
        /* Received one write from Client on Terminal characteristc */
        SendBackData = DebugConsoleCommandParsing(att_data, data_length);
    }
    /* Send it back for testing */
    if(SendBackData) {
        Term_Update(att_data,data_length);
    }
}

/**
 * @brief  Process Update firmware request
 * @param  att_data  string to write
 * @retval none
 */
static void processUpgradeFw(uint8_t* att_data)
{
    uint32_t uwCRCValue = 0;
    uint8_t *pointerByte = 0;
    
    //check size
    pointerByte = (uint8_t*) &SizeOfUpdateBlueFW;
    pointerByte[0]=att_data[ 9];
    pointerByte[1]=att_data[10];
    pointerByte[2]=att_data[11];
    pointerByte[3]=att_data[12];

    /* Check the Maximum Possible OTA size */
    if(SizeOfUpdateBlueFW > (OTA_MAX_PROG_SIZE - 8)) { // 8 bytes for size and crc
        goto error;
    }
    
    //check crc
    pointerByte = (uint8_t*) &uwCRCValue;
    pointerByte[0]=att_data[13];
    pointerByte[1]=att_data[14];
    pointerByte[2]=att_data[15];
    pointerByte[3]=att_data[16];

    //Prepare Flash
    if(!StartUpdateFWBlueMS(SizeOfUpdateBlueFW, uwCRCValue)){
        goto error;
    }

    /* Signal that we are ready sending back the crc value*/
    BufferToWrite[0] = pointerByte[0];
    BufferToWrite[1] = pointerByte[1];
    BufferToWrite[2] = pointerByte[2];
    BufferToWrite[3] = pointerByte[3];
    bytesToWrite = 4;
    Term_Update(BufferToWrite,bytesToWrite);
    //success
    return;
    
    error:
    /* Answer with a wrong CRC value for signaling the problem to BlueMS application */
    pointerByte[0] = att_data[13];
    pointerByte[1] = (att_data[14]!=0) ? 0 : 1;/* In order to be sure to have a wrong CRC */
    pointerByte[2] = att_data[15];
    pointerByte[3] = att_data[16];
    bytesToWrite = 4;
    Term_Update(BufferToWrite,bytesToWrite);
}


/**
 * @brief  This function makes the parsing of the Configuration Commands
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval uint32_t SendItBack true/false
 */
uint32_t ConfigCommandParsing(uint8_t * att_data, uint8_t data_length)
{
	uint32_t FeatureMask = (att_data[3]) | (att_data[2] << 8) | (att_data[1] << 16) | (att_data[0] << 24);
	uint8_t Command = att_data[4];
	uint8_t Data = att_data[5];
	uint32_t SendItBack = 1;

	switch (FeatureMask) {

		case FEATURE_MASK_ACC_EVENTS:
			/* Acc events */
			switch (Command) {

			case 'm':
				/* Multiple */
				switch (Data) {
				case 1:
					Config_Notify(FEATURE_MASK_ACC_EVENTS, Command, Data);
					EnableHWMultipleEvents();
					W2ST_ON_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS);
					break;
				case 0:
					Config_Notify(FEATURE_MASK_ACC_EVENTS, Command, Data);
					DisableHWMultipleEvents();
					break;
				}
				break;
			case 'o':
				switch (Data) {
				case 1:
					Config_Notify(FEATURE_MASK_ACC_EVENTS, Command, Data);
					EnableHWOrientation6D();
					W2ST_ON_HW_FEATURE(W2ST_HWF_6DORIENTATION);
					break;
				case 0:
					Config_Notify(FEATURE_MASK_ACC_EVENTS, Command, Data);
					DisableHWOrientation6D();
					break;
				}
				break;
				/* Orientation */
				break;
			case 'd':
				/* Double Tap */
				switch (Data) {
				case 1:
					Config_Notify(FEATURE_MASK_ACC_EVENTS, Command, Data);
					EnableHWDoubleTap();
					W2ST_ON_HW_FEATURE(W2ST_HWF_DOUBLE_TAP);
					break;
				case 0:
					Config_Notify(FEATURE_MASK_ACC_EVENTS, Command, Data);
					DisableHWDoubleTap();
					break;
				}
				break;
			case 'f':
				/* FreeFall */
				switch (Data) {
				case 1:
					Config_Notify(FEATURE_MASK_ACC_EVENTS, Command, Data);
					EnableHWFreeFall();
					W2ST_ON_HW_FEATURE(W2ST_HWF_FREE_FALL);
					break;
				case 0:
					Config_Notify(FEATURE_MASK_ACC_EVENTS, Command, Data);
					DisableHWFreeFall();
					break;
				}
				break;

			case 's':
				/* Single Tap */
				switch (Data) {
				case 1:
					Config_Notify(FEATURE_MASK_ACC_EVENTS, Command, Data);
					EnableHWSingleTap();
					W2ST_ON_HW_FEATURE(W2ST_HWF_SINGLE_TAP);
					break;
				case 0:
					Config_Notify(FEATURE_MASK_ACC_EVENTS, Command, Data);
					DisableHWSingleTap();
					break;
				}
				break;
			case 'p':
				/* Pedometer */
				switch (Data) {
				case 1:
					Config_Notify(FEATURE_MASK_ACC_EVENTS, Command, Data);
					ResetSWPedometer();
					EnableSWPedometer();
					W2ST_ON_HW_FEATURE(W2ST_HWF_PEDOMETER);
					break;
				case 0:
					Config_Notify(FEATURE_MASK_ACC_EVENTS, Command, Data);
					DisableSWPedometer();
					break;
				}
				break;
			case 'w':
				/* Wake UP */
				switch (Data) {
				case 1:
					Config_Notify(FEATURE_MASK_ACC_EVENTS, Command, Data);
					EnableHWWakeUp();
					W2ST_ON_HW_FEATURE(W2ST_HWF_WAKE_UP);
					break;
				case 0:
					Config_Notify(FEATURE_MASK_ACC_EVENTS, Command, Data);
					DisableHWWakeUp();
					break;
				}
				break;
			case 't':
				/* Tilt */
				switch (Data) {
				case 1:
					Config_Notify(FEATURE_MASK_ACC_EVENTS, Command, Data);
					EnableSWTilt();
					W2ST_ON_HW_FEATURE(W2ST_HWF_TILT);
					break;
				case 0:
					Config_Notify(FEATURE_MASK_ACC_EVENTS, Command, Data);
					DisableSWTilt();
					break;
				}
				break;

			}
			break;

		case FEATURE_MASK_LED:
			switch (Command) {
			case 1:
				BSP_LED_On(LED2);
				Config_Notify(FEATURE_MASK_LED, Command, Data);
				LedUpdate(LED2);
				break;
			case 0:
				BSP_LED_Off(LED2);
				Config_Notify(FEATURE_MASK_LED, Command, Data);
				LedUpdate(LED2);
				break;
			}
			break;
		case FEATURE_MASK_ECOMPASS:
			/* Sensor Fusion and e-compass */
			switch (Command) {
			case W2ST_COMMAND_CAL_STATUS: {
				 /* e-compass */
				switch (Command) {
					case W2ST_COMMAND_CAL_STATUS:
						{
						/* Replay with the calibration status for the feature */
						/* Control the calibration status */
						uint8_t tmp = BSP_Compass_Get_Calibrated() ? W2ST_COMPASS_CAL_OK : W2ST_COMPASS_CAL_ERROR;
						Config_Notify(FeatureMask,Command, tmp);
						}
						break;
					case W2ST_COMMAND_CAL_RESET:
						/* Reset the calibration */
						BSP_Compass_Set_Need_Calibration(TRUE);
						break;
					case W2ST_COMMAND_CAL_STOP:
						/* Do nothing in this case */
						break;
					default:
						/* Do nothing in this case */
						break;
				}
				break;
			}
		break;
		}
	}

	return SendItBack;
}
