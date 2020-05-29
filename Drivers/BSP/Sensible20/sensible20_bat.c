/**
 ******************************************************************************
 * @file    sensible20_bat.c
 * @date    20-November-2018
 * @brief   sensible20_bat driver file.
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
#include "sensible20_bat.h"

#include "peripheral_mngr_app.h"
#include "sensible20_port_exp.h"

#include "BlueNRG1_adc.h"

/* Private function prototypes begin -----------------------------------------*/
static float parseVoltage(float value);
/* Private function prototypes end  ------------------------------------------*/

/* Public functions realization begin ----------------------------------------*/

/**
  * @brief  Initialization of the pin, destined for switching on/off 
  *         the transistor key for measuring battery level
  * @param  none
  * @retval none
  */
void BSP_BatLevel_Pin_Init(void)
{
    PE_InitStruct_t init = {
        .pins    = PE_Pin_2, // Voltage check
        .dir     = PE_DirectionOut,
        .pull    = PE_PullDis,
        .irqMask = PE_NonGenerateInt
    };

    PE_Init(PE_Addr1, &init);
    PE_ResetPin(PE_Pin_2);
//    PE_SetPin(PE_Pin_2);
}

/**
  * @brief  Deinitialization of the ADC,
  *         destined for measuring battery level
  * @param  none
  * @retval none
  */
void BSP_BatLevel_IN_DeInit(void)
{
    ADC_Cmd(DISABLE);
    ADC_DeInit();
    
    NVIC_SetPriority(ADC_IRQn, LOW_PRIORITY);
    SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_ADC, DISABLE);
    
    PE_ResetPin(PE_Pin_2);
}

/**
  * @brief  Initialization of the ADC,
  *         destined for measuring battery level
  * @param  none
  * @retval none
  */
void BSP_BatLevel_IN_Init(void)
{
    ADC_SwCalibration();

    SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_ADC, ENABLE);
    ADC_InitType xADC_InitType;

    /* Configure ADC */    
    xADC_InitType.ADC_OSR = ADC_OSR_200;
    xADC_InitType.ADC_Input = ADC_Input_AdcPin2;
    xADC_InitType.ADC_ConversionMode = ADC_ConversionMode_Single;//ADC_ConversionMode_Continuous;//
    xADC_InitType.ADC_ReferenceVoltage = ADC_ReferenceVoltage_0V6;
    xADC_InitType.ADC_Attenuation = ADC_Attenuation_9dB54;//ADC_Attenuation_6dB02;
    
    while(ADC->SR_REG_b.BUSY != RESET){
        ADC_Cmd(ENABLE);
        while(RESET == ADC_GetFlagStatus(ADC_FLAG_EOC));
        ADC_GetConvertedData(ADC_Input_AdcPin2, ADC_ReferenceVoltage_0V6);
    }
    ADC_Init(&xADC_InitType);

    /* Enable auto offset correction */
    ADC_AutoOffsetUpdate(ENABLE);
    ADC_Calibration(ENABLE);
    
    PE_SetPin(PE_Pin_2);
}

/**
  * @brief  This function receives the pointers to soc, voltage and current,
  *         where writes current battery and consumption values
  * @param  soc      the pointer where to save the battery level in percentage
  * @param  voltage  the pointer where to save the battery voltage level
  * @param  current  the pointer where to save current consumption in mA
  *                  (is not supported yet)
  * @retval none
  */
void BSP_BatLevel_GetValues(uint32_t *soc, uint32_t *voltage, int32_t *current)
{
    static uint8_t counter = 0;
    static float prevVoltValue = 0.0f;
    static float coeff = 0.005f;
    
    ADC_Cmd(ENABLE);
    while(RESET == ADC_GetFlagStatus(ADC_FLAG_EOC));
    float volt = ADC_GetConvertedData(ADC_Input_AdcPin2, ADC_ReferenceVoltage_0V6);
    volt *= 2.0f;

    if(counter < 5){
        counter++;
        prevVoltValue = volt;
    } else {
        volt = (coeff * volt) + ((1 - coeff) * prevVoltValue);
        prevVoltValue = volt;
    }

    *voltage = (uint32_t)(volt * 1000);
    *current = 0;
    *soc = (uint32_t)(parseVoltage(volt) * 10);
}

/* Public functions realization end ------------------------------------------*/

/* Private functions realization begin ---------------------------------------*/

/**
  * @brief  This function receives the current battery level in volts
  *         and expressed it in percentage
  * @param  value the battery level in volts
  * @retval battery level in percentage
  */
static float parseVoltage(float value)
{
    float result = 0;

    if(value >= 4.2f){
        result = 100.0f;
    } else if(value >= 4.1f){
        result = ((((value - 4.1f) * 2.2f) / 0.1f) + 97.8f);
    } else if(value >= 4.0f){
        result = ((((value - 4.0f) * 9.8f) / 0.1f) + 88.0f);
    } else if(value >= 3.9f){
        result = ((((value - 3.9f) * 11.2f) / 0.1f) + 76.8f);
    } else if(value >= 3.8f){
        result = ((((value - 3.8f) * 14.6f) / 0.1f) + 62.2f);
    } else if(value >= 3.7f){
        result = ((((value - 3.7f) * 19.0f) / 0.1f) + 43.2f);
    } else if(value >= 3.6f){
        result = ((((value - 3.6f) * 22.9f) / 0.1f) + 20.3f);
    } else if(value >= 3.5f){
        result = ((((value - 3.5f) * 20.3f) / 0.1f) + 0.0f);
    } else {
        result = 0.0f;
    }

    return result;
}
/* Private functions realization end -----------------------------------------*/

/************************ (C) COPYRIGHT SensiEdge *****************************/
