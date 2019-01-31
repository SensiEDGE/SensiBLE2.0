/**
 ******************************************************************************
 * @file    sensible20_steps.c
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
#include <string.h>
#include "steval_bluemic1.h" 
#include "sensible20_steps.h"

/* Private defines begin -----------------------------------------------------*/
typedef struct
{
    uint8_t stepsAxisX;
    uint8_t stepsAxisY;
    uint8_t stepsAxisZ;
    uint8_t stepsGlobal;
} StepsStorage_t;

typedef struct
{
    int32_t oldValue;
    int32_t newValue;
} AccAxisStorage_t;

typedef struct
{
    AccAxisStorage_t axisX;
    AccAxisStorage_t axisY;
    AccAxisStorage_t axisZ;
} AccStorage_t;
/* Private defines end -------------------------------------------------------*/

/* Private function prototypes begin -----------------------------------------*/
//static void filterData(SensorAxes_t* values, AccStorage_t* storage, float coeff);
static void transferToThreeStates(
    AccAxisStorage_t* values, 
    uint8_t samples, 
    int8_t *result, 
    uint8_t *index, 
    float trashold);
static void desideWhatValue(uint8_t samples, int8_t *arr);
static uint8_t countValues(uint8_t samples, int8_t *arr, int8_t value);
static void setValues(uint8_t samples, int8_t *arr, int8_t value);
static uint8_t countSteps(
    uint8_t samples, 
    int8_t *arr, 
    uint8_t *steps, 
    uint8_t stepMinFreq, 
    uint16_t *currentSample);
/* Private function prototypes end  ------------------------------------------*/

/* Private variables begin ---------------------------------------------------*/
AccStorage_t accValuesStorage = {0};

//static float const KALMAN_COEFF = 0.1f;
static float const TRASHOLD = 10;//0.005;
//static uint8_t const SAMPLES = 4;
#define SAMPLES 5
static uint8_t const STEP_MIN_FREQ = 10;//90;
static uint32_t counter = 0;
static uint8_t tmp = 0;
/* Private variables end -----------------------------------------------------*/

/* Public functions realization begin ----------------------------------------*/
/**
 * @brief  Process accelero data looking for steps.
 * @param  acc_data the accelero data.
 * @retval the number of steps (1 or 0);
 */
uint8_t ProcessSteps(SensorAxes_t acc_data)
{
    static int8_t resultsX[SAMPLES+2] = {0};
    static uint8_t indexX = 0;
    static uint16_t currentSampleX = 0;
    static int8_t resultsY[SAMPLES+2] = {0};
    static uint8_t indexY = 0;
    static uint16_t currentSampleY = 0;
    static int8_t resultsZ[SAMPLES+2] = {0};
    static uint8_t indexZ = 0;
    static uint16_t currentSampleZ = 0;
    static uint8_t firstIteration = 1;
    
    if(firstIteration){
        firstIteration = 0;
        accValuesStorage.axisX.newValue = acc_data.AXIS_X;
        accValuesStorage.axisY.newValue = acc_data.AXIS_Y;
        accValuesStorage.axisZ.newValue = acc_data.AXIS_Z;
    }
    
    //switch previous data
    accValuesStorage.axisX.oldValue = accValuesStorage.axisX.newValue;
    accValuesStorage.axisY.oldValue = accValuesStorage.axisY.newValue;
    accValuesStorage.axisZ.oldValue = accValuesStorage.axisZ.newValue;
    //write new data
    accValuesStorage.axisX.newValue = acc_data.AXIS_X;
    accValuesStorage.axisY.newValue = acc_data.AXIS_Y;
    accValuesStorage.axisZ.newValue = acc_data.AXIS_Z;

    currentSampleX += 1;
    currentSampleY += 1;
    currentSampleZ += 1;

    counter += 1;

    //instead of global StepsStorage_t steps
    StepsStorage_t steps = {0};

    transferToThreeStates(&accValuesStorage.axisX, SAMPLES, resultsX, &indexX, TRASHOLD);
    if(indexX == 2){
        tmp |= countSteps(SAMPLES, resultsX, &steps.stepsAxisX, STEP_MIN_FREQ, &currentSampleX);
    }
    transferToThreeStates(&accValuesStorage.axisY, SAMPLES, resultsY, &indexY, TRASHOLD);
    if(indexY == 2){
        tmp |= countSteps(SAMPLES, resultsY, &steps.stepsAxisY, STEP_MIN_FREQ, &currentSampleY);
    }
    transferToThreeStates(&accValuesStorage.axisZ, SAMPLES, resultsZ, &indexZ, TRASHOLD);
    if(indexZ == 2){
        tmp |= countSteps(SAMPLES, resultsZ, &steps.stepsAxisZ, STEP_MIN_FREQ, &currentSampleZ);
    }

    if(tmp != 0){
        if(counter > STEP_MIN_FREQ){
            steps.stepsGlobal += 1;
            counter = 0;
        }
        tmp = 0;
    }
    return steps.stepsGlobal;
}
/* Public functions realization end ------------------------------------------*/

/* Private functions realization begin ---------------------------------------*/
/**
 * @brief  Filter accelero data.
 * @param  axis     the pointer to Axis storage.
 * @param  samples  the number of samples to process.
 * @param  result   the pointer to the result of operation.
 * @param  index    the pointer to the sample index.
 * @param  trashold the trashold for filter.
 * @retval none
 */
static void transferToThreeStates(
    AccAxisStorage_t* axis, 
    uint8_t samples, 
    int8_t *result, 
    uint8_t *index, 
    float trashold)
{
    int32_t diff = axis->newValue - axis->oldValue;
	if((diff < 0) && ((diff * (-1)) > trashold)) {
		result[*index] = -1;
	} else if((diff > 0) && (diff > trashold)) {
		result[*index] = 1;
	} else {
		result[*index] = 0;
	}
	/*------------------- Filter part ---------------------*/
	if(*index == (samples + 1)) {
		uint8_t tmp = 0;
		for(uint8_t i = 1; i <= samples; i++){
			if(result[i] != result[0]){
				tmp = 0;
				if(i == 1){
					desideWhatValue(samples, result);
					result[0] = result[1];
					result[1] = result[samples + 1];
					*index = 2;
					break;
				} else {
					for(uint8_t k = 0; k <= ((samples + 2) - i); k++){
						result[k] = result[i - 1 + k];
					}
					*index = samples + 3 - i;
					break;
				}
			} else {
				tmp = 1;
			}
		}
		if(tmp == 1){
			*index = 2;
			result[1] = result[samples+1];
		}
	} else {
		*index += 1;
	}
}

/**
 * @brief  Deside what value choose for filter.
 * @param  samples the number of samples to process.
 * @param  arr     the pointer to the array with values.
 * @retval none
 */
static void desideWhatValue(uint8_t samples, int8_t *arr)
{
	uint8_t tmp = 0;
	for(uint8_t i = 1; i <= samples; i++){
		if(arr[i] != arr[samples + 1]){
			tmp = 0;
			break;
		} else {
			tmp = 1;
		}
	}
	if(tmp == 1){
		setValues(samples, arr, arr[samples + 1]);
	} else {
		uint8_t zeros = countValues(samples, arr, 0);
		uint8_t ones = countValues(samples, arr, 1);
		uint8_t minusOnes = countValues(samples, arr, (-1));
		if(zeros > ones && zeros > minusOnes){
			setValues(samples, arr, 0);
		} else if(ones > zeros && ones > minusOnes){
			setValues(samples, arr, 1);
		} else if(minusOnes > ones && minusOnes > zeros){
			setValues(samples, arr, (-1));
		} else {
			setValues(samples, arr, arr[samples + 1]);
		}
	}
}

/**
 * @brief  Count how much values (1, 0 or -1).
 * @param  samples the number of samples to process.
 * @param  arr     the pointer to the array with values.
 * @param  value   the value to count.
 * @retval the number of values
 */
static uint8_t countValues(uint8_t samples, int8_t *arr, int8_t value)
{
	uint8_t result = 0;
	for(uint8_t i = 1; i <= samples; i++){
		if(arr[i] == value){
			result += 1;
		}
	}
	return result;
}

/**
 * @brief  Fill the array with the value.
 * @param  samples the number of samples to be set.
 * @param  arr     the pointer to the array with values.
 * @param  value   the value to be set.
 * @retval none
 */
static void setValues(uint8_t samples, int8_t *arr, int8_t value)
{
	for(uint8_t i = 1; i <= samples; i++){
		arr[i] = value;
	}
}

/**
 * @brief  Count how much values (1, 0 or -1).
 * @param  samples       the number of samples to process.
 * @param  arr           the pointer to the array with values.
 * @param  steps         the pointer to the steps storage.
 * @param  stepMinFreq   the minimum period of steps.
 * @param  currentSample the pointer to the current sample number.
 * @retval the number of steps
 */
static uint8_t countSteps(
    uint8_t samples, 
    int8_t *arr, 
    uint8_t *steps, 
    uint8_t stepMinFreq, 
    uint16_t *currentSample)
{
	uint8_t result = 0;
	for(uint8_t i = 1; i <= samples; i++){
		if((arr[i - 1] == 1) && (arr[i] == (-1))){
			if(*currentSample > stepMinFreq){
				*steps = *steps + 1;
				result = 1;
				*currentSample = 0;
			}
		}
	}
	return result;
}

/* Private functions realization end -----------------------------------------*/

/************************ (C) COPYRIGHT SensiEdge *****************************/
