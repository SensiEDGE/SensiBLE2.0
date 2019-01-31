/* Includes ------------------------------------------------------------------*/

#include "steval_bluemic1.h" 
#include "sensible_sensors.h"

#include "steval_bluemic1_uv.h"
#include "steval_bluemic1_temperature.h"
#include "steval_bluemic1_humidity.h"
#include "steval_bluemic1_pressure.h"
#include "steval_bluemic1_lux.h"
#include "sensible20_magneto.h"
#include "peripheral_mngr_app.h"
#include "sensible20_port_exp.h"

static void *HandleUvSensor;
static void *HandleTempHts221;
static void *HandleHumHts221;
static void *HandleTempLps25Hb;
static void *HandlePresLps25Hb;

static void *MAGNETO_handle;

static void SensibleErrorHandler(void);


SensibleResult_t SensorsInit(void)
{
    BSP_ULTRAVIOLET_Init(VEML6075_0, &HandleUvSensor);
    
    BSP_TEMPERATURE_Init(HTS221_T_0, &HandleTempHts221);
    
    BSP_HUMIDITY_Init(HTS221_H_0, &HandleHumHts221);
    
    BSP_TEMPERATURE_Init(LPS25HB_T_0, &HandleTempLps25Hb);
    
    BSP_PRESSURE_Init(LPS25HB_P_0, &HandlePresLps25Hb);
    
    BSP_LUX_Init();
    
    BSP_MAGNETO_Init(LIS2MDL_M_0, &MAGNETO_handle);
    
    return SENSIBLE_SENS_OK;
}
    
void SensorsEnable(void)
{
    uint8_t status = 0;
    
    if(COMPONENT_OK == BSP_ULTRAVIOLET_IsInitialized(HandleUvSensor, &status) &&
       status != 0) {
        if(COMPONENT_OK != BSP_ULTRAVIOLET_Sensor_Enable(HandleUvSensor)) {
            SensibleErrorHandler();
        }
    }
    if(COMPONENT_OK == BSP_TEMPERATURE_IsInitialized(HandleTempHts221, &status) &&
       status != 0) {
        if(COMPONENT_OK != BSP_TEMPERATURE_Sensor_Enable(HandleTempHts221)) {
            SensibleErrorHandler();
        }
    }
    if(COMPONENT_OK == BSP_HUMIDITY_IsInitialized(HandleHumHts221, &status) &&
       status != 0) {    
        if(COMPONENT_OK != BSP_HUMIDITY_Sensor_Enable(HandleHumHts221)) {
            SensibleErrorHandler();
        }
    }
    if(COMPONENT_OK == BSP_TEMPERATURE_IsInitialized(HandleTempLps25Hb, &status) &&
       status != 0) {    
        if(COMPONENT_OK != BSP_TEMPERATURE_Sensor_Enable(HandleTempLps25Hb)) {
            SensibleErrorHandler();
        }
    }
    if(COMPONENT_OK == BSP_PRESSURE_IsInitialized(HandlePresLps25Hb, &status) &&
       status != 0) {
        if(COMPONENT_OK != BSP_PRESSURE_Sensor_Enable(HandlePresLps25Hb)) {
            SensibleErrorHandler();
        }
    }
    if(0 != BSP_LUX_IsInitalized()) {
        if(LUX_OK != BSP_LUX_PowerON()) {
            SensibleErrorHandler();
        }
    }

    if(COMPONENT_OK == BSP_MAGNETO_IsInitialized(MAGNETO_handle, &status) &&
       status != 0) {
        if(COMPONENT_OK != BSP_MAGNETO_Sensor_Enable(MAGNETO_handle)) {
            SensibleErrorHandler();
        }
        if(COMPONENT_OK != BSP_MAGNETO_Set_ODR_Value(MAGNETO_handle, 50.f)){
            SensibleErrorHandler();
        }
    }
}

void SensorsDisable(void)
{
    uint8_t status = 0;
    
    if(COMPONENT_OK == BSP_ULTRAVIOLET_IsInitialized(HandleUvSensor, &status) &&
       status != 0) {
        if(COMPONENT_OK != BSP_ULTRAVIOLET_Sensor_Disable(HandleUvSensor)) {
            SensibleErrorHandler();
        }
    }
    if(COMPONENT_OK == BSP_TEMPERATURE_IsInitialized(HandleTempHts221, &status) &&
       status != 0) {
        if(COMPONENT_OK != BSP_TEMPERATURE_Sensor_Disable(HandleTempHts221)) {
            SensibleErrorHandler();
        }
    }
    if(COMPONENT_OK == BSP_HUMIDITY_IsInitialized(HandleHumHts221, &status) &&
       status != 0) {    
        if(COMPONENT_OK != BSP_HUMIDITY_Sensor_Disable(HandleHumHts221)) {
            SensibleErrorHandler();
        }
    }
    if(COMPONENT_OK == BSP_TEMPERATURE_IsInitialized(HandleTempLps25Hb, &status) &&
       status != 0) {    
        if(COMPONENT_OK != BSP_TEMPERATURE_Sensor_Disable(HandleTempLps25Hb)) {
            SensibleErrorHandler();
        }
    }
    if(COMPONENT_OK == BSP_PRESSURE_IsInitialized(HandlePresLps25Hb, &status) &&
       status != 0) {
        if(COMPONENT_OK != BSP_PRESSURE_Sensor_Disable(HandlePresLps25Hb)) {
            SensibleErrorHandler();
        }
    }
    if(0 != BSP_LUX_IsInitalized()) {
        if(LUX_OK != BSP_LUX_PowerOFF()) {
            SensibleErrorHandler();
        }
    }

    if(COMPONENT_OK == BSP_MAGNETO_IsInitialized(MAGNETO_handle, &status) &&
       status != 0) {
        if(COMPONENT_OK != BSP_MAGNETO_Sensor_Disable(MAGNETO_handle)) {
            SensibleErrorHandler();
        }
    }
}

SensibleResult_t SensorsEnableMag(void)
{
    uint8_t status = 0;
    if(COMPONENT_OK == BSP_MAGNETO_IsInitialized(MAGNETO_handle, &status) &&
       status != 0) 
    {
        if(COMPONENT_OK != BSP_MAGNETO_Sensor_Enable(MAGNETO_handle)) {
            SensibleErrorHandler();
        }
        if(COMPONENT_OK != BSP_MAGNETO_Set_ODR_Value(MAGNETO_handle, 50.f)){
            SensibleErrorHandler();
        }
        return SENSIBLE_SENS_OK;
    }
    return SENSIBLE_SENS_ERR;
}

SensibleResult_t SensorsDisableMag(void)
{
    uint8_t status = 0;
    if(COMPONENT_OK == BSP_MAGNETO_IsInitialized(MAGNETO_handle, &status) &&
       status != 0) {
        if(COMPONENT_OK != BSP_MAGNETO_Sensor_Disable(MAGNETO_handle)) {
            SensibleErrorHandler();
        }
        return SENSIBLE_SENS_OK;
    }
    return SENSIBLE_SENS_ERR;
}

SensibleResult_t SensorsReadUv(uint16_t* val)
{
    if(COMPONENT_OK == BSP_ULTRAVIOLET_Get_Uv(HandleUvSensor, val)) {
        return SENSIBLE_SENS_OK;
    } else {
        return SENSIBLE_SENS_ERR;
    }
}

SensibleResult_t SesnsorsReadTemp1(float * val)
{
    if(COMPONENT_OK == BSP_TEMPERATURE_Get_Temp(HandleTempHts221, val)) {
        return SENSIBLE_SENS_OK;
    } else {
        return SENSIBLE_SENS_ERR;
    }
}

SensibleResult_t SensorsReadTemp2(float * val)
{
    if(COMPONENT_OK == BSP_TEMPERATURE_Get_Temp(HandleTempLps25Hb, val)) {
        return SENSIBLE_SENS_OK;
    } else {
        return SENSIBLE_SENS_ERR;
    }
}

SensibleResult_t SesnorsReadHumidity(float * val)
{
    if(COMPONENT_OK == BSP_HUMIDITY_Get_Hum(HandleHumHts221, val)) {
        return SENSIBLE_SENS_OK;
    } else {
        return SENSIBLE_SENS_ERR;
    }
}

SensibleResult_t SensorsReadPressure(float * val)
{
    if(COMPONENT_OK == BSP_PRESSURE_Get_Press(HandlePresLps25Hb, val)) {
        return SENSIBLE_SENS_OK;
    } else {
        return SENSIBLE_SENS_ERR;
    }
}

SensibleResult_t SensorsReadLux(uint16_t* val)
{
    if(LUX_OK == BSP_LUX_GetValue(val)) {
        return SENSIBLE_SENS_OK;
    } else {
        return SENSIBLE_SENS_ERR;
    }
}

SensibleResult_t SensorsReadMag(SensorAxes_t* axes)
{ 
    if(COMPONENT_OK == BSP_MAGNETO_Get_Axes(MAGNETO_handle, axes)) {
        return SENSIBLE_SENS_OK;
    } else {
        return SENSIBLE_SENS_ERR;
    }
}

/* Local functions -----------------------------------------------------------*/
static void SensibleErrorHandler(void)
{
    while(1) {
        PER_APP_Error_Handler();
    }
}
