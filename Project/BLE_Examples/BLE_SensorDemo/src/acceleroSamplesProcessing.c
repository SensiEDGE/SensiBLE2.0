/**
 ******************************************************************************
 * @file    acceleroSampleProcessing.c
 * @brief   acceleroSampleProcessing.
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
#include <math.h>
#include "acceleroSamplesProcessing.h"
#include "sensible20_steps.h"
#include "sensible20_accelero.h"

#include "main.h"

/* Private function prototypes begin -----------------------------------------*/
static void filterData(SensorAxes_t* data, uint8_t size, float coeff);
static BOOL processTilt(SensorAxes_t data);
static BOOL comparePosition(float newPosition, float oldPosition, float threshold);
/* Private function prototypes end  ------------------------------------------*/

/* Private variables begin ---------------------------------------------------*/
#define KALMAN_COEFF      0.1f
#define TILT_ANGLE        0.6109f//rad, it equals to 35 degrees
#define TILT_DURATION     1000//ms

static SensorAxes_t accValues[LIS2DW12_FIFO_SIZE] = {0};
static uint16_t steps = 0;
/* Private variables end -----------------------------------------------------*/

/* Public functions realization begin ----------------------------------------*/

/**
  * @brief  Get the number of steps
  * @param  none
  * @retval none
  */
uint16_t getSteps(void)
{
    return steps;
}

/**
  * @brief  Readout the accelero FIFO
  * @param  handle  the Accelero handle
  * @retval none
  */
void flushAcceleroFifo(void *handle)
{
    //clear buffer
    memset(accValues, 0x00, sizeof(accValues));
    
    BSP_ACCELERO_FIFO_Get_Data_Ext(handle, accValues, LIS2DW12_FIFO_SIZE);
    
    filterData(accValues, LIS2DW12_FIFO_SIZE, KALMAN_COEFF);
}

/**
  * @brief  Look for steps
  * @param  none
  * @retval TRUE    if steps were found
  * @retval FALSE   if steps were not found
  */
BOOL lookForSteps(void)
{
    static uint16_t tmpStepsCounter = 0;
    static uint64_t tmpSamplesCounter = 0;
    static uint16_t prevSteps = 0;
    BOOL stepsFound = FALSE;

    for(uint8_t i = 0; i < LIS2DW12_FIFO_SIZE; i++){
        uint8_t tmp = ProcessSteps(accValues[i]);
        
        tmpSamplesCounter += 1;
        
        if(tmp != 0){
            tmpSamplesCounter = 0;
            if(tmpStepsCounter >= 7){
                if(steps == prevSteps){
                    steps += tmpStepsCounter;
                } else {
                    steps += tmp;
                }
                stepsFound = TRUE;
            } else {
                tmpStepsCounter += tmp;
            }
        } else {
            if(tmpSamplesCounter > 200){//ODR = 50, ~ 4 sec
                if(tmpStepsCounter != 0){
                    tmpStepsCounter = 0;
                    prevSteps = steps;
                }
            }
        }
    }
    return stepsFound;
}

/**
  * @brief  Look for tilt
  * @param  none
  * @retval TRUE  if tilt was found
  * @retval FALSE if tilt was not found
  */
BOOL lookForTilt(void)
{
    BOOL tiltFound = FALSE;
    for(uint8_t i = 0; i < LIS2DW12_FIFO_SIZE; i++){
        BOOL tmp = processTilt(accValues[i]);
        if(tmp){
            tiltFound = TRUE;
        }
    }
    
    return tiltFound;
}

/**
  * @brief  Reset the number of steps
  * @param  none
  * @retval none
  */
void resetSteps(void)
{
    steps = 0;
}

/* Public functions realization end ------------------------------------------*/

/* Private functions realization begin ---------------------------------------*/
/**
 * @brief  Filter accelero data.
 * @param  data  the pointer to Accelero values.
 * @param  size  the size of data array.
 * @param  coeff the filter coefficient.
 * @retval none
 */
static void filterData(SensorAxes_t* data, uint8_t size, float coeff)
{
    static BOOL firstIteration = 1;
    static int32_t oldX = 0;
    static int32_t oldY = 0;
    static int32_t oldZ = 0;
    
    if(firstIteration){
        firstIteration = 0;
        oldX = data[0].AXIS_X;
        oldY = data[0].AXIS_Y;
        oldZ = data[0].AXIS_Z;
    }
    
    for(uint8_t i = 0; i < size; i++){
        data[i].AXIS_X = (int32_t)((float)(coeff * data[i].AXIS_X) + (float)((1 - coeff) * oldX));
        data[i].AXIS_Y = (int32_t)((float)(coeff * data[i].AXIS_Y) + (float)((1 - coeff) * oldY));
        data[i].AXIS_Z = (int32_t)((float)(coeff * data[i].AXIS_Z) + (float)((1 - coeff) * oldZ));
        
        oldX = data[i].AXIS_X;
        oldY = data[i].AXIS_Y;
        oldZ = data[i].AXIS_Z;
    }
}

/**
 * @brief  Processing the accelero values looking for a tilt.
 * @param  data  the SensorAxes_t value.
 * @retval TRUE  if tilt was found
 * @retval FALSE if tilt was not found
 */
static BOOL processTilt(SensorAxes_t data)
{
    static BOOL firstIteration = 1;
    static float startPositionX = 0.0f;
    static float startPositionY = 0.0f;
    static float startPositionZ = 0.0f;
    
    static uint32_t startTimeStamp = 0;
    static BOOL waiting = FALSE;
    
    BOOL result = FALSE;
    
    //calculating angle
    float currentX = (float)asin(data.AXIS_X / 1000.0f);
    float currentY = (float)asin(data.AXIS_Y / 1000.0f);
    float currentZ = (float)acos(data.AXIS_Z / 1000.0f);
    
    if(firstIteration){
        firstIteration = 0;
        startPositionX = currentX;
        startPositionY = currentY;
        startPositionZ = currentZ;
    }
    
    if(comparePosition(currentX, startPositionX, TILT_ANGLE) || 
       comparePosition(currentY, startPositionY, TILT_ANGLE) ||
       comparePosition(currentZ, startPositionZ, TILT_ANGLE))
    {
        if(waiting){
            if(tick_ms() >= startTimeStamp){
                if((tick_ms() - startTimeStamp) >= TILT_DURATION){
                    waiting = FALSE;
                    result = TRUE;
                }
            } else {
                //lSystickCounter overflow
                waiting = FALSE;
                result = TRUE;
            }
        } else {
            waiting = TRUE;
            startTimeStamp = tick_ms();
        }
    } else {
        waiting = FALSE;
    }
    
    if(result){
        startPositionX = currentX;
        startPositionY = currentY;
        startPositionZ = currentZ;
    }
    
    return result;
}

/**
 * @brief Compare the new position of the accelero axis with the previous one. 
 *        If the difference between them is more than the threshold return TRUE.
 * @param  newPosition the new axis angle relatively to the gravity.
 * @param  oldPosition the old axis angle relatively to the gravity.
 * @param  threshold   the threshold angle
 * @retval TRUE  if tilt was found
 * @retval FALSE if tilt was not found
 */
static BOOL comparePosition(float newPosition, float oldPosition, float threshold)
{
    float diff = newPosition - oldPosition;
    if(diff < 0){
        diff *= (-1);
    }

    if (diff > threshold) {
		return TRUE;
	} else {
		return FALSE;
	}
}

/* Private functions realization end -----------------------------------------*/

/************************ (C) COPYRIGHT SensiEdge *****************************/
