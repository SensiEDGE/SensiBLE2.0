/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSIBLE_SENSORS_H
#define __SENSIBLE_SENSORS_H


#include "sensor.h"
/* Includes ------------------------------------------------------------------*/

typedef enum {
    SENSIBLE_SENS_OK = 0,
    SENSIBLE_SENS_ERR
} SensibleResult_t;


/* Feature mask for LED */
#define FEATURE_MASK_LED 0x20000000
/* Feature mask for Sensor fusion short precision */
#define FEATURE_MASK_SENSORFUSION_SHORT 0x00000100
/* Feature mask for e-compass */
#define FEATURE_MASK_ECOMPASS 0x00000040
/* Feature mask for hardware events */
#define FEATURE_MASK_ACC_EVENTS 0x00000400
/* Feature mask for Temperature1 */
#define FEATURE_MASK_TEMP1 0x00040000
/* Feature mask for Temperature2 */
#define FEATURE_MASK_TEMP2 0x00010000
/* Feature mask for Pressure */
#define FEATURE_MASK_PRESS 0x00100000
/* Feature mask for Humidity */
#define FEATURE_MASK_HUM   0x00080000
/* Feature mask for Accelerometer */
#define FEATURE_MASK_ACC   0x00800000
/* Feature mask for Gyroscope */
#define FEATURE_MASK_GRYO  0x00400000
/* Feature mask for Magnetometer */
#define FEATURE_MASK_MAG   0x00200000
/* Feature mask for Microphone */
#define FEATURE_MASK_MIC   0x04000000
/* Feature mask for BlueVoice */
#define FEATURE_MASK_BLUEVOICE   0x08000000

SensibleResult_t SensorsInit(void);
void SensorsEnable(void);
void SensorsDisable(void);
SensibleResult_t SensorsEnableMag(void);
SensibleResult_t SensorsDisableMag(void);

SensibleResult_t SensorsReadUv(int16_t* val);
SensibleResult_t SesnsorsReadTemp1(float * val);
SensibleResult_t SensorsReadTemp2(float * val);
SensibleResult_t SesnorsReadHumidity(float * val);
SensibleResult_t SensorsReadPressure(float * val);
SensibleResult_t SensorsReadLux(uint16_t* val);
SensibleResult_t SensorsReadRGB(uint32_t* red, uint32_t* green, uint32_t* blue);
SensibleResult_t SensorsReadMag(SensorAxes_t* axes);

#endif /* __SENSIBLE_SENSORS_H */
