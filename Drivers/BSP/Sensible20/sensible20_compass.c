/**
 ******************************************************************************
 * @file    sensible20_compass.c
 * @brief   This file implements compass feature.
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

/* Includes ------------------------------------------------------------------*/
#include "sensible20_compass.h"

#include <math.h>
#include "motion_mc_cm0p.h"
#include "peripheral_mngr_app.h"
#include "sensible_services.h"
#include "sensible20_accelero.h"
#include "AT25XE041B_Driver.h"

/* Private defines begin -----------------------------------------------------*/
typedef struct
{
    SensorAxes_t oldValues;
    SensorAxes_t newValues;
} CompassAxis_t;

#pragma pack(push, 1)
typedef struct
{
    SensorAxes_t HI_Offset;
    float        SI_Coeff[3][3];
    uint8_t      crc;
} CompassCalibrationData_t;
#pragma pack(pop)

//save calibration data on the last page of SPI flash
#define COMPASS_CONFIG_ADDR  ((AT25XE041B_FLASHSIZE) - (AT25XE041B_PAGESIZE))

#define COMPASS_ACCELERO_FILTER_COEFF   (0.05f)
#define COMPASS_MAGNETO_FILTER_COEFF    (0.05f)

#define FROM_MGAUSS_TO_UT               (0.1f)
#define FROM_UT_TO_MGAUSS               (10.0f)
#define FROM_RAD_TO_DEG                 (57.295779513f)//(180/3.14)

#define COMPASS_UPDATE_PERIOD_MS        (100)
#define COMPASS_CALIBRATION_PERIOD_MS   (40)

/* Private defines end -------------------------------------------------------*/

/* Private variables begin ---------------------------------------------------*/
static volatile uint16_t compassCounter = 0;
static volatile BOOL compassIsActive = FALSE;
static volatile BOOL needCalibration = FALSE;
static BOOL calibrationIsRunnig = FALSE;
static BOOL calibrated = FALSE;

static CompassAxis_t acceleroAxis = {0};
static CompassAxis_t magnetoAxis = {0};

static CompassCalibrationData_t calibData = {0};

/* Private variables begin ---------------------------------------------------*/

/* Private function prototypes begin -----------------------------------------*/
static float calculateAzimuth(SensorAxes_t* magAxis, float pitch, float roll);
static void calculatePitchRoll(SensorAxes_t* accAxis, float* pitch, float* roll);
static SensibleResult_t calibrate(void);
static void compassFilter(CompassAxis_t* axis, float coeff);
static BOOL countCRC(uint8_t *data, uint16_t size, uint8_t *crc);
static BOOL readCalibCoeffs(void);
static SensibleResult_t runAlgorithm(void);
static BOOL saveCalibCoeffs(CompassCalibrationData_t* data);

/* Private function prototypes end  ------------------------------------------*/

/* Public functions realization begin ----------------------------------------*/

/**
  * @brief  Start eCompass. 
  *         Set Accelero and Magneto Enable
  * @param  none
  * @retval none
  */
void BSP_Compass_Start(void)
{
    BSP_ACCELERO_Sensor_Enable(ACCELERO_handle);
    SensorsEnableMag();
    compassIsActive = TRUE;
}

/**
  * @brief  Stop eCompass. 
  *         Set Accelero and Magneto Disable
  * @param  none
  * @retval none
  */
void BSP_Compass_Stop(void)
{
    BSP_ACCELERO_Sensor_Disable(ACCELERO_handle);
    SensorsDisableMag();
    compassIsActive = FALSE;
}

/**
  * @brief  Run compass algorithm or calibration
  * @param  none
  * @retval SENSIBLE_SENS_OK  in case of success
  * @retval SENSIBLE_SENS_ERR in case of error
  */
SensibleResult_t BSP_Compass_Run(void)
{
    if(needCalibration){
        static uint8_t tmp = 0;
        SensibleResult_t res = calibrate();
        
        //Update compass every 200 ms
        if(calibrationIsRunnig && (tmp++ >= 5)){
            tmp = 0;
            runAlgorithm();
        }
        
        return res;
    } else {
        return runAlgorithm();
    }
}

/**
  * @brief  This function is used for applying calibration coefficients 
  *         to measured magneto values.
  * @param  magAxis  the pointer where measured magneto values are saved
  * @retval none
  */
void BSP_Compass_ApplyCalibrationCoefficients(SensorAxes_t* magAxis)
{
    if(calibrated){
        //Hard iron compensation
        magAxis->AXIS_X -= calibData.HI_Offset.AXIS_X;
        magAxis->AXIS_Y -= calibData.HI_Offset.AXIS_Y;
        magAxis->AXIS_Z -= calibData.HI_Offset.AXIS_Z;
        
        //Soft iron compensation
        magAxis->AXIS_X = (int32_t)((magAxis->AXIS_X * calibData.SI_Coeff[0][0]) + 
                                    (magAxis->AXIS_Y * calibData.SI_Coeff[0][1]) + 
                                    (magAxis->AXIS_Z * calibData.SI_Coeff[0][2]));
        magAxis->AXIS_Y = (int32_t)((magAxis->AXIS_X * calibData.SI_Coeff[1][0]) + 
                                    (magAxis->AXIS_Y * calibData.SI_Coeff[1][1]) + 
                                    (magAxis->AXIS_Z * calibData.SI_Coeff[1][2]));
        magAxis->AXIS_Z = (int32_t)((magAxis->AXIS_X * calibData.SI_Coeff[2][0]) + 
                                    (magAxis->AXIS_Y * calibData.SI_Coeff[2][1]) + 
                                    (magAxis->AXIS_Z * calibData.SI_Coeff[2][2]));
    }
}

/**
  * @brief  Check if magneto was calibrated
  * @param  none
  * @retval none
  */
void BSP_Compass_Check_Calibration(void)
{
    if(!calibrated){
        calibrated = readCalibCoeffs();
    }
    
    /* Notifications of Compass Calibration status*/
    Config_Notify(FEATURE_MASK_ECOMPASS,
                  W2ST_COMMAND_CAL_STATUS,
                  calibrated ? W2ST_COMPASS_CAL_OK : W2ST_COMPASS_CAL_ERROR);
}

/**
  * @brief  Inform about compass state (active or not)
  * @param  none
  * @retval BOOL
  */
BOOL BSP_Compass_Get_Active(void)
{
    return compassIsActive;
}

/**
  * @brief  Inform if compass is calibrated
  * @param  none
  * @retval BOOL
  */
BOOL BSP_Compass_Get_Calibrated(void)
{
    return !needCalibration;
}

/**
  * @brief  Increase compass counter
  * @param  none
  * @retval none
  */
void BSP_Compass_IncTick(void)
{
    compassCounter += 1;
}

/**
  * @brief  Inform if it is necessity to calibrate magneto sensor
  * @param  state TRUE if it is necessity to calibrate magneto sensor,
  *               FALSE in other way.
  * @retval none
  */
void BSP_Compass_Set_Need_Calibration(BOOL state)
{
    if(state){
        calibrated = FALSE;
    }
    needCalibration = state;
}

/**
  * @brief  Inform if it is time to update compass information
  * @param  none
  * @retval BOOL
  */
BOOL BSP_Compass_Need_Update(void)
{
    if(calibrationIsRunnig){
        //update every 40ms
        if(compassCounter >= COMPASS_CALIBRATION_PERIOD_MS){
            return TRUE;
        }
    } else {
        //update every 100 ms
        if(compassCounter >= COMPASS_UPDATE_PERIOD_MS){
            return TRUE;
        }
    }
    return FALSE;
}

/* Public functions realization end ------------------------------------------*/

/* Private functions realization begin ---------------------------------------*/
/**
  * @brief  Calculate azimuth based on magneto axis values, pitch and roll
  * @param  axis  the pointer to SensorAxes_t structure, where
  *               filtered magneto axis values are saved
  * @param  pitch pitch, calculated previously
  * @param  roll  roll, calculated previously
  * @retval azimuth
  */
static float calculateAzimuth(SensorAxes_t* magAxis, float pitch, float roll)
{
    double cosPitch = cos(pitch);
    double sinPitch = sin(pitch);
    double cosRoll = cos(roll);
    double sinRoll = sin(roll);
    
//    double X_h = (magAxis->AXIS_X * cosPitch) + (magAxis->AXIS_Y * sinRoll * sinPitch) - 
//                 (magAxis->AXIS_Z * cosRoll * sinPitch);
//    double Y_h = (magAxis->AXIS_Y * cosRoll) + (magAxis->AXIS_Z * sinRoll);
    
    double X_h = (magAxis->AXIS_X * cosPitch) + (magAxis->AXIS_Z * sinPitch);
    double Y_h = (magAxis->AXIS_X * sinRoll * sinPitch) + (magAxis->AXIS_Y * cosRoll) - 
                 (magAxis->AXIS_Z * sinRoll * cosPitch);
    
    float azimuth;
    
//    if((X_h > 0) && (Y_h >= 0)){
//        azimuth = 360.0f - (((float)atan(Y_h / X_h)) * FROM_RAD_TO_DEG);
//        
//    } else if(X_h < 0){
//        azimuth = 180.0f - (((float)atan(Y_h / X_h)) * FROM_RAD_TO_DEG);
//        
//    } else if((X_h > 0) && (Y_h <= 0)){
//        azimuth = (-1) * (((float)atan(Y_h / X_h)) * FROM_RAD_TO_DEG);
//        
//    } else if((X_h == 0) && (Y_h < 0)){
//        azimuth = 90.0f;
//        
//    } else if((X_h == 0) && (Y_h > 0)){
//        azimuth = 270.0f;
//        
//    } else {
//        azimuth = 0.0f;
//    }
    
    if((X_h > 0) && (Y_h >= 0)){
        azimuth = ((float)atan(Y_h / X_h)) * FROM_RAD_TO_DEG;
        
    } else if(X_h < 0){
        azimuth = 180.0f + (((float)atan(Y_h / X_h)) * FROM_RAD_TO_DEG);
        
    } else if((X_h > 0) && (Y_h <= 0)){
        azimuth = 360.0f + (((float)atan(Y_h / X_h)) * FROM_RAD_TO_DEG);
        
    } else if((X_h == 0) && (Y_h < 0)){
        azimuth = 90.0f;
        
    } else if((X_h == 0) && (Y_h > 0)){
        azimuth = 270.0f;

    } else {
        azimuth = 0.0f;
    }
    
    return azimuth;
}

/**
  * @brief  Calculate pitch and roll based on accelero axis values
  * @param  axis  the pointer to SensorAxes_t structure, where
  *               filtered accelero axis values are saved
  * @param  pitch the pointer where to save pitch
  * @param  roll  the pointer where to save roll
  * @retval none
  */
static void calculatePitchRoll(SensorAxes_t* accAxis, float* pitch, float* roll)
{
    float pitchDenominator = (float)sqrt((accAxis->AXIS_Y * accAxis->AXIS_Y) + 
                                         (accAxis->AXIS_Z * accAxis->AXIS_Z));
    float rollDenominator = (float)sqrt((accAxis->AXIS_X * accAxis->AXIS_X) + 
                                        (accAxis->AXIS_Z * accAxis->AXIS_Z));
    if(pitchDenominator == 0.0f){
        *pitch = 0;
    } else {
        *pitch = (float)atan(accAxis->AXIS_X / pitchDenominator);
    }
    
    if(rollDenominator == 0.0f){
        *roll = 0;
    } else {
        *roll = (float)atan(accAxis->AXIS_Y / rollDenominator);
    }
    
    
//    float pitchDenominator = (float)sqrt((accAxis->AXIS_Y * accAxis->AXIS_Y) + 
//                                         (accAxis->AXIS_Z * accAxis->AXIS_Z));
//    if(pitchDenominator == 0.0f){
//        *pitch = 0;
//    } else {
//        *pitch = (float)atan(accAxis->AXIS_X / pitchDenominator);
//    }
//    
//    if(accAxis->AXIS_Z == 0){
//        *roll = 0;
//    } else {
//        *roll = atan((-accAxis->AXIS_Y)/accAxis->AXIS_Z);
//    }
    
}

/**
  * @brief  Calibrate magneto sensor
  * @param  none
  * @retval SENSIBLE_SENS_OK  in case of success
  * @retval SENSIBLE_SENS_ERR in case of error
  */
static SensibleResult_t calibrate(void)
{
    MMC_CM0P_Input_t dataIn = {0};
    MMC_CM0P_Output_t dataOut = {0};
    SensorAxes_t magAxis = {0};
    
    if(!calibrationIsRunnig){
        calibrationIsRunnig = TRUE;
        MotionMC_CM0P_Initialize(COMPASS_CALIBRATION_PERIOD_MS,
                                 MMC_CM0P_HI_AND_SI, ENABLE);
    }
    
    if(SENSIBLE_SENS_OK != SensorsReadMag(&magAxis)){
        goto fail;
    }
    
    dataIn.Mag[0] = (float)(magAxis.AXIS_X * FROM_MGAUSS_TO_UT);
    dataIn.Mag[1] = (float)(magAxis.AXIS_Y * FROM_MGAUSS_TO_UT);
    dataIn.Mag[2] = (float)(magAxis.AXIS_Z * FROM_MGAUSS_TO_UT);
    
    MotionMC_CM0P_Update(&dataIn);
    MotionMC_CM0P_GetCalParams(&dataOut);
    
    if((dataOut.CalQuality == MMC_CM0P_CALQSTATUSOK) ||
       (dataOut.CalQuality == MMC_CM0P_CALQSTATUSGOOD))
    {
        calibrationIsRunnig = FALSE;
        
        needCalibration = FALSE;
        
        calibData.HI_Offset.AXIS_X = (int32_t)(dataOut.HI_Bias[0] * FROM_UT_TO_MGAUSS);
        calibData.HI_Offset.AXIS_Y = (int32_t)(dataOut.HI_Bias[1] * FROM_UT_TO_MGAUSS);
        calibData.HI_Offset.AXIS_Z = (int32_t)(dataOut.HI_Bias[2] * FROM_UT_TO_MGAUSS);
        
        //Switch off the warning [Pa039]: use of address of unaligned structure member
        #pragma diag_suppress=Pa039
        //copy calibration matrix
        memcpy(&calibData.SI_Coeff, &dataOut.SI_Matrix, sizeof(calibData.SI_Coeff));
        #pragma diag_default=Pa039
        
        /* Disable magnetometer calibration */
        MotionMC_CM0P_Initialize(COMPASS_CALIBRATION_PERIOD_MS,
                                 MMC_CM0P_HI_AND_SI, DISABLE);
        
        calibrated = TRUE;
        
        saveCalibCoeffs(&calibData);
        
        Config_Notify(FEATURE_MASK_ECOMPASS,
                      W2ST_COMMAND_CAL_STATUS,
                      W2ST_COMPASS_CAL_OK);

    }
    
    return SENSIBLE_SENS_OK;
    
fail:
    return SENSIBLE_SENS_ERR;
}

/**
  * @brief  This function is used for smoothing Accelero and Magneto values
  * @param  axis  the pointer to CompassAxis_t structure, where old 
  *               and new axis values are saved
  * @param  coeff filter coefficient
  * @retval none
  */
static void compassFilter(CompassAxis_t* axis, float coeff)
{
    axis->newValues.AXIS_X = (int32_t)((float)(coeff * axis->newValues.AXIS_X) 
                              + (float)((1 - coeff) * axis->oldValues.AXIS_X));
    axis->newValues.AXIS_Y = (int32_t)((float)(coeff * axis->newValues.AXIS_Y) 
                              + (float)((1 - coeff) * axis->oldValues.AXIS_Y));
    axis->newValues.AXIS_Z = (int32_t)((float)(coeff * axis->newValues.AXIS_Z) 
                              + (float)((1 - coeff) * axis->oldValues.AXIS_Z));
        
    axis->oldValues.AXIS_X = axis->newValues.AXIS_X;
    axis->oldValues.AXIS_Y = axis->newValues.AXIS_Y;
    axis->oldValues.AXIS_Z = axis->newValues.AXIS_Z;
}

/**
 * @brief  Count CRC. 
 * @param  data  The pointer to the data
 * @param  size  The size of the data
 * @param  crc   The pointer where the crc is written
 * @retval TRUE  in case of success  
 * @retval FALSE in case of failure
 */
static BOOL countCRC(uint8_t *data, uint16_t size, uint8_t *crc)
{
    if((data == NULL) || (crc == NULL) || (size == 0)){
        return FALSE;
    }
    *crc = 0x30;
    
    for(uint8_t i = 0; i < size; i++){
        *crc += data[i];
        *crc >>= 1;
    }
    
    return TRUE;
}

/**
  * @brief  Read calibration coefficients from external flash memory
  * @param  none
  * @retval TRUE  in case of success
  * @retval FALSE in case of error
  */
static BOOL readCalibCoeffs(void)
{
    CompassCalibrationData_t tmp = {0};
    uint8_t crc = 0;
    
    if(AT25XE041B_OK != AT25XE041B_ReadByteArray(COMPASS_CONFIG_ADDR, 
                                                (uint8_t*)&tmp, 
                                                sizeof(tmp)))
    {
        goto fail;
    }

    if(countCRC((uint8_t*)&tmp, (sizeof(tmp) - 1), &crc) == FALSE){
        goto fail;
    }
    
    if(crc != tmp.crc){
        goto fail;
    }
    
    memcpy(&calibData, &tmp, sizeof(tmp));
    
    needCalibration = FALSE;
    
    return TRUE;
    
fail: return FALSE;
}

/**
  * @brief  Calculate and update the Azimuth angle
  * @param  none
  * @retval SENSIBLE_SENS_OK  in case of success
  * @retval SENSIBLE_SENS_ERR in case of error
  */
static SensibleResult_t runAlgorithm(void)
{
    static BOOL firstIteration = TRUE;
    float pitch = 0;
    float roll = 0;
    float azimuth = 0;
    
//    double accNormCoeff = 0;
//    double magNormCoeff = 0;
    
    SensorAxes_t correctedAccelero = {0};
    SensorAxes_t correctedMagneto  = {0};
    
    if(COMPONENT_OK != BSP_ACCELERO_Get_Axes(ACCELERO_handle, 
                                             &acceleroAxis.newValues))
    {
        goto fail;
    }
    
    if(SENSIBLE_SENS_OK != SensorsReadMag(&magnetoAxis.newValues)){
        goto fail;
    }
    
    if(firstIteration){
        firstIteration = FALSE;
        
        acceleroAxis.oldValues.AXIS_X = acceleroAxis.newValues.AXIS_X;
        acceleroAxis.oldValues.AXIS_Y = acceleroAxis.newValues.AXIS_Y;
        acceleroAxis.oldValues.AXIS_Z = acceleroAxis.newValues.AXIS_Z;
        
        magnetoAxis.oldValues.AXIS_X = magnetoAxis.newValues.AXIS_X;
        magnetoAxis.oldValues.AXIS_Y = magnetoAxis.newValues.AXIS_Y;
        magnetoAxis.oldValues.AXIS_Z = magnetoAxis.newValues.AXIS_Z;
    }
    
    BSP_Compass_ApplyCalibrationCoefficients(&magnetoAxis.newValues);
    
    compassFilter(&acceleroAxis, COMPASS_ACCELERO_FILTER_COEFF);
    compassFilter(&magnetoAxis, COMPASS_MAGNETO_FILTER_COEFF);
    
//    correctedAccelero.AXIS_X = acceleroAxis.newValues.AXIS_Y;
//    correctedAccelero.AXIS_Y = acceleroAxis.newValues.AXIS_X;
//    correctedAccelero.AXIS_Z = -acceleroAxis.newValues.AXIS_Z;
//    
//    correctedMagneto.AXIS_X = -magnetoAxis.newValues.AXIS_X;
//    correctedMagneto.AXIS_Y = -magnetoAxis.newValues.AXIS_Y;
//    correctedMagneto.AXIS_Z = -magnetoAxis.newValues.AXIS_Z;
    
    correctedAccelero.AXIS_X = acceleroAxis.newValues.AXIS_Y;
    correctedAccelero.AXIS_Y = -acceleroAxis.newValues.AXIS_X;
    correctedAccelero.AXIS_Z = acceleroAxis.newValues.AXIS_Z;
    
    correctedMagneto.AXIS_X = -magnetoAxis.newValues.AXIS_X;
    correctedMagneto.AXIS_Y = magnetoAxis.newValues.AXIS_Y;
    correctedMagneto.AXIS_Z = magnetoAxis.newValues.AXIS_Z;
    
//    ////////////////////////////////////////////////////////////////////////////
//    //vectors normalization
//    accNormCoeff = sqrt((correctedAccelero.AXIS_X*correctedAccelero.AXIS_X)+(correctedAccelero.AXIS_Y*correctedAccelero.AXIS_Y)+(correctedAccelero.AXIS_Z*correctedAccelero.AXIS_Z));
//    magNormCoeff = sqrt((correctedMagneto.AXIS_X*correctedMagneto.AXIS_X)+(correctedMagneto.AXIS_Y*correctedMagneto.AXIS_Y)+(correctedMagneto.AXIS_Z*correctedMagneto.AXIS_Z));
//    
//    correctedAccelero.AXIS_X = (int32_t)((correctedAccelero.AXIS_X / accNormCoeff) * 1000);
//    correctedAccelero.AXIS_Y = (int32_t)((correctedAccelero.AXIS_Y / accNormCoeff) * 1000);
//    correctedAccelero.AXIS_Z = (int32_t)((correctedAccelero.AXIS_Z / accNormCoeff) * 1000);
//    
//    correctedMagneto.AXIS_X = (int32_t)((correctedMagneto.AXIS_X / magNormCoeff) * 1000);
//    correctedMagneto.AXIS_Y = (int32_t)((correctedMagneto.AXIS_Y / magNormCoeff) * 1000);
//    correctedMagneto.AXIS_Z = (int32_t)((correctedMagneto.AXIS_Z / magNormCoeff) * 1000);
//    ////////////////////////////////////////////////////////////////////////////
    
    calculatePitchRoll(&correctedAccelero, &pitch, &roll);
    
    azimuth = calculateAzimuth(&correctedMagneto, pitch, roll);
    
    //testing roll and pitch
//    azimuth = 90 + (roll * FROM_RAD_TO_DEG);
    
    ECompass_Update((uint16_t)(azimuth * 100));
    
    return SENSIBLE_SENS_OK;
    
fail: 
    ECompass_Update(0);
    return SENSIBLE_SENS_ERR;
}

/**
  * @brief  Save calibration coefficients to external flash memory
  * @param  data  the pointer where calibration data is saved
  * @retval TRUE  in case of success
  * @retval FALSE in case of error
  */
static BOOL saveCalibCoeffs(CompassCalibrationData_t* data)
{
    CompassCalibrationData_t tmp = {0};
    uint8_t crc = 0;
    uint16_t len = 0;
    
    if(data == NULL){
        goto fail;
    }
    
    if(FALSE == countCRC((uint8_t*)data, 
                         (sizeof(CompassCalibrationData_t) - 1), 
                         &data->crc))
    {
        goto fail;
    }
    
    
    if(AT25XE041B_OK != AT25XE041B_WriteByteArray(COMPASS_CONFIG_ADDR,
                                                (uint8_t*)data,
                                                &len,
                                                sizeof(CompassCalibrationData_t)))
    {
        goto fail;
    }
    
    if(AT25XE041B_OK != AT25XE041B_ReadByteArray(COMPASS_CONFIG_ADDR, 
                                                (uint8_t*)&tmp, 
                                                sizeof(tmp)))
    {
        goto fail;
    }
    
    
    if(countCRC((uint8_t*)&tmp, (sizeof(tmp) - 1), &crc) == FALSE){
        goto fail;
    }
    
    if(crc != tmp.crc){
        goto fail;
    }
    
    return TRUE;
    
fail: return FALSE;
}

/* Private functions realization end -----------------------------------------*/

/************************ (C) COPYRIGHT SensiEdge *****************************/
