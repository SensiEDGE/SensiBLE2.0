/**
 ******************************************************************************
 * @file    LIS2DW12_ACC_driver_HL.h
 * @author  MEMS Application Team
 * @brief   This file contains definitions for the LIS2DW12_ACC_driver_HL.c firmware driver
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
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
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIS2DW12_ACC_DRIVER_HL_H
#define __LIS2DW12_ACC_DRIVER_HL_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/

#include "accelerometer.h"

/* Include accelero sensor component drivers. */
#include "LIS2DW12_ACC_driver.h"



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup LIS2DW12 LIS2DW12
 * @{
 */

/** @addtogroup LIS2DW12_Public_Constants Public constants
 * @{
 */

/** @addtogroup LIS2DW12_ACC_SENSITIVITY Accelero sensitivity values based on selected full scale and power mode
 * @{
 */

#define LIS2DW12_ACC_SENSITIVITY_FOR_FS_2G_LOPOW1_MODE   0.976  /**< Sensitivity value for 2g full scale, Low-power1 mode [mg/LSB] */
#define LIS2DW12_ACC_SENSITIVITY_FOR_FS_2G_OTHER_MODES   0.244  /**< Sensitivity value for 2g full scale, all other modes except Low-power1 [mg/LSB] */

#define LIS2DW12_ACC_SENSITIVITY_FOR_FS_4G_LOPOW1_MODE   1.952  /**< Sensitivity value for 4g full scale, Low-power1 mode [mg/LSB] */
#define LIS2DW12_ACC_SENSITIVITY_FOR_FS_4G_OTHER_MODES   0.488  /**< Sensitivity value for 4g full scale, all other modes except Low-power1 [mg/LSB] */

#define LIS2DW12_ACC_SENSITIVITY_FOR_FS_8G_LOPOW1_MODE   3.904  /**< Sensitivity value for 8g full scale, Low-power1 mode [mg/LSB] */
#define LIS2DW12_ACC_SENSITIVITY_FOR_FS_8G_OTHER_MODES   0.976  /**< Sensitivity value for 8g full scale, all other modes except Low-power1 [mg/LSB] */

#define LIS2DW12_ACC_SENSITIVITY_FOR_FS_16G_LOPOW1_MODE  7.808  /**< Sensitivity value for 16g full scale, Low-power1 mode [mg/LSB] */
#define LIS2DW12_ACC_SENSITIVITY_FOR_FS_16G_OTHER_MODES  1.952  /**< Sensitivity value for 16g full scale, all other modes except Low-power1 [mg/LSB] */

/**
 * @}
 */

/** @addtogroup LIS2DW12_ACT_THRESHOLD Sleep-to-wake, return-to-sleep activation threshold in low-power mode
 * @{
 */

#define LIS2DW12_ACT_THRESHOLD_LOW       0x01  /**< Lowest  value of threshold */
#define LIS2DW12_ACT_THRESHOLD_MID_LOW   0x0F
#define LIS2DW12_ACT_THRESHOLD_MID       0x1F
#define LIS2DW12_ACT_THRESHOLD_MID_HIGH  0x2F
#define LIS2DW12_ACT_THRESHOLD_HIGH      0x3F  /**< Highest value of threshold */

/**
 * @}
 */

/** @addtogroup LIS2DW12_ACT_DURATION Sleep-to-wake, return-to-sleep duration
 * @{
 */

#define LIS2DW12_ACT_DURATION_LOW       0x01  /**< Lowest  value of threshold */
#define LIS2DW12_ACT_DURATION_MID_LOW   0x0F
#define LIS2DW12_ACT_DURATION_MID       0x1F
#define LIS2DW12_ACT_DURATION_MID_HIGH  0x2F
#define LIS2DW12_ACT_DURATION_HIGH      0x3F  /**< Highest value of threshold */

/**
 * @}
 */

/**
 * @}
 */

/** @addtogroup LIS2DW12_Public_Types LIS2DW12 Public Types
 * @{
 */

/**
 * @brief LIS2DW12 accelero extended features driver internal structure definition
 */
typedef struct
{
    DrvStatusTypeDef ( *Read_All_Int_Src_Register       ) ( DrvContextTypeDef*, uint8_t* );
    DrvStatusTypeDef ( *Enable_Free_Fall_Detection      ) ( DrvContextTypeDef* );
    DrvStatusTypeDef ( *Disable_Free_Fall_Detection     ) ( DrvContextTypeDef* );
    DrvStatusTypeDef ( *Get_Free_Fall_Detection_Status  ) ( DrvContextTypeDef*, uint8_t* );
    DrvStatusTypeDef ( *Set_Free_Fall_Threshold         ) ( DrvContextTypeDef*, uint8_t );
    DrvStatusTypeDef ( *Enable_Wake_Up_Detection        ) ( DrvContextTypeDef* );
    DrvStatusTypeDef ( *Disable_Wake_Up_Detection       ) ( DrvContextTypeDef* );
    DrvStatusTypeDef ( *Get_Wake_Up_Detection_Status    ) ( DrvContextTypeDef*, uint8_t* );
    DrvStatusTypeDef ( *Set_Wake_Up_Threshold           ) ( DrvContextTypeDef*, uint8_t );
    DrvStatusTypeDef ( *Enable_Single_Tap_Detection     ) ( DrvContextTypeDef* );
    DrvStatusTypeDef ( *Disable_Single_Tap_Detection    ) ( DrvContextTypeDef* );
    DrvStatusTypeDef ( *Get_Single_Tap_Detection_Status ) ( DrvContextTypeDef*, uint8_t* );
    DrvStatusTypeDef ( *Enable_Double_Tap_Detection     ) ( DrvContextTypeDef* );
    DrvStatusTypeDef ( *Disable_Double_Tap_Detection    ) ( DrvContextTypeDef* );
    DrvStatusTypeDef ( *Get_Double_Tap_Detection_Status ) ( DrvContextTypeDef*, uint8_t* );
    DrvStatusTypeDef ( *Set_Tap_Threshold               ) ( DrvContextTypeDef*, uint8_t );
    DrvStatusTypeDef ( *Set_Tap_Shock_Time              ) ( DrvContextTypeDef*, uint8_t );
    DrvStatusTypeDef ( *Set_Tap_Quiet_Time              ) ( DrvContextTypeDef*, uint8_t );
    DrvStatusTypeDef ( *Set_Tap_Duration_Time           ) ( DrvContextTypeDef*, uint8_t );
    DrvStatusTypeDef ( *Enable_6D_Orientation           ) ( DrvContextTypeDef* );
    DrvStatusTypeDef ( *Disable_6D_Orientation          ) ( DrvContextTypeDef* );
    DrvStatusTypeDef ( *Get_6D_Orientation_Status       ) ( DrvContextTypeDef*, uint8_t* );
    DrvStatusTypeDef ( *Get_6D_Orientation_XL           ) ( DrvContextTypeDef*, uint8_t* );
    DrvStatusTypeDef ( *Get_6D_Orientation_XH           ) ( DrvContextTypeDef*, uint8_t* );
    DrvStatusTypeDef ( *Get_6D_Orientation_YL           ) ( DrvContextTypeDef*, uint8_t* );
    DrvStatusTypeDef ( *Get_6D_Orientation_YH           ) ( DrvContextTypeDef*, uint8_t* );
    DrvStatusTypeDef ( *Get_6D_Orientation_ZL           ) ( DrvContextTypeDef*, uint8_t* );
    DrvStatusTypeDef ( *Get_6D_Orientation_ZH           ) ( DrvContextTypeDef*, uint8_t* );
    DrvStatusTypeDef ( *FIFO_Set_ODR_Value              ) ( DrvContextTypeDef*, float );
    DrvStatusTypeDef ( *FIFO_Get_Full_Status            ) ( DrvContextTypeDef*, uint8_t* );
    DrvStatusTypeDef ( *FIFO_Get_Empty_Status           ) ( DrvContextTypeDef*, uint8_t* );
    DrvStatusTypeDef ( *FIFO_Get_Overrun_Status         ) ( DrvContextTypeDef*, uint8_t* );
    DrvStatusTypeDef ( *FIFO_Get_Data                   ) ( DrvContextTypeDef*, SensorAxes_t*, uint8_t size );
    DrvStatusTypeDef ( *FIFO_Get_Num_Of_Samples         ) ( DrvContextTypeDef*, uint8_t* );
    DrvStatusTypeDef ( *FIFO_Set_Mode                   ) ( DrvContextTypeDef*, uint8_t );
    DrvStatusTypeDef ( *FIFO_Set_INT1_FIFO_Full         ) ( DrvContextTypeDef*, uint8_t );
    DrvStatusTypeDef ( *FIFO_Set_INT1_FIFO_Treshold     ) ( DrvContextTypeDef*, uint8_t );
    DrvStatusTypeDef ( *FIFO_Set_INT2_FIFO_Treshold     ) ( DrvContextTypeDef*, uint8_t );
    DrvStatusTypeDef ( *FIFO_Set_Watermark_Level        ) ( DrvContextTypeDef*, uint8_t );
    DrvStatusTypeDef ( *Set_Interrupt_Latch             ) ( DrvContextTypeDef*, uint8_t );
} LIS2DW12_ExtDrv_t;

/**
 * @brief LIS2DW12 specific data internal structure definition
 */
typedef struct
{
  float Previous_ODR;
} LIS2DW12_Data_t;

/**
 * @}
 */

/** @addtogroup LIS2DW12_Public_Variables Public variables
 * @{
 */

extern ACCELERO_Drv_t LIS2DW12_Drv;
extern LIS2DW12_ExtDrv_t LIS2DW12_ExtDrv;

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __LIS2DW12_ACC_DRIVER_HL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
