/**
  ******************************************************************************
  * @file    BlueNRG1_it.c 
  * @author  VMA RF Application Team
  * @version V1.0.0
  * @date    September-2015
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "ble_const.h" 
#include "bluenrg1_stack.h"
#include "SDK_EVAL_Com.h"
#include "clock.h"

#include "steval_bluemic1.h"
#include "steval_bluemic1_audio_in.h"
#include "peripheral_mngr_app.h"
#include "sensible20_compass.h"

#ifndef SENSOR_EMULATION
#include "lsm6ds3.h"
#include "lsm6ds3_hal.h"
#include "gatt_db.h"
#endif

#include "bluevoice_adpcm_bnrg1.h"

/** @addtogroup BlueNRG1_StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_Examples
  * @{
  */ 

/** @addtogroup GPIO_IOToggle
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#ifndef SENSOR_EMULATION

extern LSM6DS3_DrvExtTypeDef *Imu6AxesDrvExt;

#endif

extern void setAccMagIntOccurred(BOOL status);

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
      PER_APP_Error_Handler();
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
    PER_APP_Error_Handler();
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
    PER_APP_Error_Handler();
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
    PER_APP_Error_Handler();
  }
}

/**
  * @brief  This function handles Debug Monitor exception.
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles SVCall exception.
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  */
void SysTick_Handler(void)
{
  SysCount_Handler();

  if(BSP_Compass_Get_Active()){
    BSP_Compass_IncTick();
  }

  BluevoiceADPCM_BNRG1_IncTick();
}


/******************************************************************************/
/*                 BlueNRG-1 Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (system_bluenrg1.c).                                               */
/******************************************************************************/


/**
  * @brief  This function handles DMA interrupt request.
  * @param  None
  * @retval None
  */
void DMA_Handler(void)
{
  /* Check DMA Half Transfer Complete interrupt */
  if(DMA_GetFlagStatus(DMA_FLAG_HT0)) {      
    DMA_ClearFlag(DMA_FLAG_HT0);
    HT_IT_Callback();
  }
    
  /* Check DMA Transfer Complete interrupt */
  if(DMA_GetFlagStatus(DMA_FLAG_TC0)) {      
    DMA_ClearFlag(DMA_FLAG_TC0);
    TC_IT_Callback();
  }
}

/**
  * @brief  This function handles GPIO interrupt request.
  * @param  None
  * @retval None
  */
void GPIO_Handler(void)
{
#ifndef SENSOR_EMULATION
  uint8_t free_fall_status; 
  /* Check if GPIO pin 12 interrupt event occured */
  if(GPIO_GetITPendingBit(LSM6DS3_IRQ_PIN) == SET) 
  {
    /* Clear the IRQ pending bit */
    GPIO_ClearITPendingBit(LSM6DS3_IRQ_PIN);
    
    /* Set the IRQ flag */
    Imu6AxesDrvExt->Get_Status_Free_Fall_Detection(&free_fall_status);
    if (free_fall_status == 1)
    {
      request_free_fall_notify = TRUE;
    }

  }  
#endif 
    /* If BUTTON_1 is pressed */
    if(BSP_PB_GetITPendingBit(BUTTON_USER) == SET)
    {
        BSP_PB_ClearITPendingBit(BUTTON_USER);
        if(APP_PER_state==APP_STATUS_CONNECTED){   
            BV_APP_StartStop_ctrl();
        }
    }

    /* Accelero/Magneto interrupt*/
    if(GPIO_GetITPendingBit(EXP_INT_PIN) == SET){
        GPIO_ClearITPendingBit(EXP_INT_PIN);
        setAccMagIntOccurred(TRUE);
    }
    
//    //SWD
//    if(GPIO_GetITPendingBit(GPIO_Pin_9) == SET) {
//        /* Clear GPIO pending interrupt */
//        GPIO_ClearITPendingBit(GPIO_Pin_9);
//    }
  // Added handlers

}

/**
* @brief  This function handles MFT1B interrupt request.
* @param  None
* @retval None
*/
void MFT1B_Handler(void)
{
  if ( MFT_StatusIT(MFT1,MFT_IT_TND) != RESET )
  {    
    /* Set the counter at 30 ms */
    MFT_SetCounter2(MFT1, 6000);
    
    /** Clear MFT11 pending interrupt */
    MFT_ClearIT(MFT1, MFT_IT_TND);

    AccGryro_DataReady = 1;    
  }
}

/**
* @brief  This function handles UART interrupt request.
* @param  None
* @retval None
*/
void UART_Handler(void)
{  

}

void Blue_Handler(void)
{
  // Call RAL_Isr
  RAL_Isr();
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

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
