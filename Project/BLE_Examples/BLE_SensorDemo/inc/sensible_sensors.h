/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSIBLE_SENSORS_H
#define __SENSIBLE_SENSORS_H

/* Includes ------------------------------------------------------------------*/

typedef enum {
    SENSIBLE_SENS_OK = 0,
    SENSIBLE_SENS_ERR
} SensibleResult_t;

SensibleResult_t SensorsInit(void);
void SensorsEnable(void);
void SensorsDisable(void);

SensibleResult_t SensorsEnableMag(void);
SensibleResult_t SensorsDisableMag(void);

SensibleResult_t SensorsReadUv(uint16_t* val);
SensibleResult_t SesnsorsReadTemp1(float * val);
SensibleResult_t SensorsReadTemp2(float * val);
SensibleResult_t SesnorsReadHumidity(float * val);
SensibleResult_t SensorsReadPressure(float * val);
SensibleResult_t SensorsReadLux(uint16_t* val);
SensibleResult_t SensorsReadMag(SensorAxes_t* axes);

#endif /* __SENSIBLE_SENSORS_H */
