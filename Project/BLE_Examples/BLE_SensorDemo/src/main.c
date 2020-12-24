/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
* File Name          : main.c 
* Date               : January-2019
* Description        : Example code for SensiBLE2.0
********************************************************************************
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
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "ble_const.h" 
#include "bluenrg1_stack.h"
#include "SDK_EVAL_Config.h"
#include "sleep.h"
#include "OTA.h"  
#include "bluevoice_app.h"

#include "peripheral_mngr_app.h"
#include "BlueVoice_config.h"
#include "steval_bluemic1.h"
#include "steval_bluemic1_audio_in.h"

#include "clock.h"

#include "steval_bluemic1_uv.h"
#include "steval_bluemic1_temperature.h"
#include "steval_bluemic1_humidity.h"
#include "steval_bluemic1_pressure.h"
#include "steval_bluemic1_lux.h"

#include "sensible20_led.h"
#include "sensible20_port_exp.h"
#include "sensible_services.h"
#include "sensible_sensors.h"
#include "sensible20_bat.h"
#include "sensible20_compass.h"

#include "HWAdvanceFeatures.h"
#include "AT25XE041B_Driver.h"

#include <stdio.h>
#include <inttypes.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifndef DEBUG
#define DEBUG 0
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static volatile BOOL accMagIntOccurred = FALSE;

uint32_t FirstConnectionConfig = 0;

/* Private function prototypes -----------------------------------------------*/
void Error_Handler(void);

void setAccMagIntOccurred(BOOL status);

void InitGPIO(void);
void InertialDisable(void);
void InertialEnable(void);
void AudioDisable(void);
void AudioEnable(void);

/* Private functions ---------------------------------------------------------*/
static void sleep(void);

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void DeinitAll(void);
void EnableAll(void);

int main(void)
{
  /* System initialization function */
  SystemInit();
  
  /* SysTick initialization 1ms */  
  Clock_Init();
  
  /* Identify BlueNRG1 platform */
  SdkEvalIdentification();

#if DEBUG==1
  /* Configure I/O communication channel */
  SdkEvalComUartInit(UART_BAUDRATE);
#endif

  Sensor_IO_Init();
  
  /* BUTTON_1 initialization for start/stop audio streaming */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO /* BUTTON_MODE_EXTI */);

  /* BlueNRG-1 stack init */
  uint8_t ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
    PER_APP_Error_Handler();
  }
  
  /* BLE Initialization */
  ret = PER_APP_Init_BLE();
  if(ret != APP_SUCCESS)
  {
    PRINTF("Error in PER_APP_Init_BLE() 0x%02x\r\n", ret);
    PER_APP_Error_Handler();
  }
  
  /* BlueVoice profile Initialization */
  ret = BV_APP_profile_init(AUDIO_SAMPLING_FREQUENCY);  
  if(ret != APP_SUCCESS)
  {
    PRINTF("Error in BV_APP_profile_init(AUDIO_SAMPLING_FREQUENCY) 0x%02x\r\n", ret);
    PER_APP_Error_Handler();
  }
  
  /* Led Init */
  BSP_LED_Init(LED1); /* Activity led */
  BSP_LED_Init(LED2); /* Activity led */
  BSP_LED_On(LED2);
  BSP_LED_Off(LED2);
  
  /* BLE Service Initialization*/
  ret = PER_APP_Service_Init();
  if(ret != APP_SUCCESS)
  {
    PRINTF("Error in PER_APP_Service_Init() 0x%02x\r\n", ret);
    PER_APP_Error_Handler();
  }
  
  InitGPIO();
  
  /* Initialize SPI FLASH*/
  ret = AT25XE041B_Init();
  if(ret != APP_SUCCESS)
  {
    PRINTF("Error in AT25XE041B_Init() 0x%02x\r\n", ret);
    PER_APP_Error_Handler();
  }

  SensorsInit();
  
#if 0
  DeinitAll();
  PER_APP_Stop_Advertise();
#endif
  
  BSP_LED_On(LED1);
  BSP_LED_Off(LED1);
  AudioEnable();
  SensorsEnable();
  InertialEnable();
  InitHWFeatures();
  
  DeinitAll();
  
  PER_APP_Advertise();
  
  /* Infinite loop */
  while(1) 
  {
    /* BLE Stack Tick */
    BTLE_StackTick();
    /* Application Tick */
    APP_Tick();
      
    /* Handle Interrupt from MEMS */
    if(accMagIntOccurred || !GPIO_ReadBit(EXP_INT_PIN)){
        accMagIntOccurred = FALSE;
        uint8_t val = PE_ReadIntStatus();
        if((val & LIS2DW12_INT1_PIN) != 0) {
            HWFeaturesCallback(LIS2DW12_INT1_PIN);
        }
        if((val & LIS2DW12_INT2_PIN) != 0){
            HWFeaturesCallback(LIS2DW12_INT2_PIN);
        }
    }

    //Check if magneto is calibrated
    if(FirstConnectionConfig){
        FirstConnectionConfig = 0;
        BSP_Compass_Check_Calibration();
    }

    /* E-Compass Updated every 0.1 Seconds*/
    if(BSP_Compass_Get_Active() && BSP_Compass_Need_Update()){
        BSP_Compass_Run();
    }

    if(APP_PER_state == APP_STATUS_ADVERTISEMENT) {
      if (1)
      {
        BlueNRG_Sleep(SLEEPMODE_NOTIMER, 0, 0);
      }
      else
      {
        BlueNRG_Sleep(SLEEPMODE_CPU_HALT, 0, 0);
      }
    }
  
    /* While the button is pressed device does not go to sleep mode,
     * so it is easy to connect with debugger.
     */
    ButtonState = BSP_PB_GetState(BUTTON_USER);
    if(0 == ButtonState) {
        BSP_LED_On(LED2);
        while(!BSP_PB_GetState(BUTTON_USER)){}
        BSP_LED_Off(LED2);
        sleep();
    }

  }
}

static void sleep(void)
{
    uint8_t sleeping = 1;
    
    AT25XE041B_EnterDeepPowerDown();
    SensorsDisable();
    
    SysTick->CTRL  &= ~SysTick_CTRL_TICKINT_Msk;
    hci_le_set_advertise_enable(0);
    
    for(uint8_t i = 0; i < 0x80; i++)
    {
        BTLE_StackTick();
    }
    
    do
    {
        if (1)
        {
            if (HAL_VTimerStart_ms(0, 5000) != 0) {
              return;
            } else {
              /* The BTLE_StackTick() is necessary to activate the sleep timer */
              BTLE_StackTick();
            }

          BlueNRG_Sleep(SLEEPMODE_WAKETIMER, 0, 0);
        }
        else
        {
          BlueNRG_Sleep(SLEEPMODE_CPU_HALT, 0, 0);
        }

        uint8_t bntState = BSP_PB_GetState(BUTTON_USER);
        if(0 == bntState) {
            BSP_LED_On(LED2);
            while(!BSP_PB_GetState(BUTTON_USER)){}
            sleeping = 0;
            BSP_LED_Off(LED2);
        }
    } while(sleeping);
    
    SysTick->CTRL  |= SysTick_CTRL_TICKINT_Msk;
    hci_le_set_advertise_enable(1);
    AT25XE041B_ExitDeepPowerDown();
}

void EnableAll(void)
{
    SensorsEnable();
    InertialEnable();
    AudioEnable();
}

void DeinitAll(void)
{
    SensorsDisable();
    InertialDisable();
    AudioDisable();
}

void InitGPIO(void)
{
    BSP_BatLevel_Pin_Init();

    /*------------------------------------------------------------------------*/
    /* --- Configure EXP_INT as interrupt source begin -----------------------*/
    /*------------------------------------------------------------------------*/
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = EXP_INT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Input;
    GPIO_InitStructure.GPIO_Pull = DISABLE;
    GPIO_InitStructure.GPIO_HighPwr = DISABLE;
    GPIO_Init(&GPIO_InitStructure);

    NVIC_InitType NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = GPIO_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = LOW_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Configures EXTI line */
    GPIO_EXTIConfigType GPIO_EXTIStructure;
    GPIO_EXTIStructure.GPIO_Pin = EXP_INT_PIN;
    GPIO_EXTIStructure.GPIO_IrqSense = GPIO_IrqSense_Edge;
    GPIO_EXTIStructure.GPIO_Event = GPIO_Event_Low;
    GPIO_EXTIConfig(&GPIO_EXTIStructure);

    /* Clear pending interrupt */
    GPIO_ClearITPendingBit(EXP_INT_PIN);

    /* Enable the interrupt */
    GPIO_EXTICmd(EXP_INT_PIN, ENABLE);

    /* Port expander pins init start */
    PE_InitStruct_t init;

    /*------------------------------------------------------------------------*/
    /* --- Configure EXP_INT as interrupt source end -------------------------*/
    /*------------------------------------------------------------------------*/

    //ACC_INT2
    init.pins = LIS2DW12_INT1_PIN;
    init.dir = PE_DirectionIn;
    init.pull = PE_PullDn;
    init.irqMask = PE_GenerateInt;
    PE_Init(PE_Addr1, &init);

    //M_INT1
    init.pins = PE_Pin_4;
    init.dir = PE_DirectionIn;
    init.pull = PE_PullDis;
    init.irqMask = PE_NonGenerateInt;
    PE_Init(PE_Addr1, &init);

    //ACC_INT1
    init.pins = LIS2DW12_INT2_PIN;
    init.dir = PE_DirectionIn;
    init.pull = PE_PullDn;
    init.irqMask = PE_GenerateInt;
    PE_Init(PE_Addr1, &init);
    /* Port expander pins init end */

#if 0
    /* Configure SWD Clock as interrupt sources in order to wake up the chip */
    /* Configure SWD_CLK as source of interrupt */
    GPIO_EXTIStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_EXTIStructure.GPIO_IrqSense = GPIO_IrqSense_Edge;
    GPIO_EXTIStructure.GPIO_Event = GPIO_Event_High;
    GPIO_EXTIConfig( &GPIO_EXTIStructure);

    /* Clear GPIO pending interrupt on SWD_CLK */
    GPIO_ClearITPendingBit(GPIO_Pin_9);

    /* Enable interrupt on WD_CLK*/
    GPIO_EXTICmd(GPIO_Pin_9, ENABLE);
#endif
}

void InertialDisable(void)
{
    INERTIAL_APP_Disable();
    
    SensorsDisableMag();
    
    MFT_Disable();
}

void InertialEnable(void)
{
    /* Inertial sensors Initialization */
    INERTIAL_APP_Init();
    
    SensorsEnableMag();
    
    MFT_Configuration();
}

void AudioDisable(void)
{
    BSP_Audio_IN_DeInit();
}

void AudioEnable(void)
{
    BSP_AUDIO_IN_Init(AUDIO_SAMPLING_FREQUENCY);
}

uint32_t tick_ms(void)
{
  return Clock_Time();
}

/**
 * @brief  Delay function in ms.
 * @param  lTimeMs time in ms
 * @retval None
*/
void DelayMs(volatile uint32_t lTimeMs)
{
  Clock_Wait(lTimeMs);
}

/**
* @brief  EXTI line detection callback.
* @param  uint16_t GPIO_Pin Specifies the pins connected EXTI line
* @retval None
*/
void setAccMagIntOccurred(BOOL status)
{
    accMagIntOccurred = status;
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
      PER_APP_Error_Handler();
  }
}
#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
