/**
  ******************************************************************************
  * @file    HWAdvanceFeatures.h 
  * @author  Central LAB
  * @version V3.0.0
  * @date    12-May-2017
  * @brief   DS3/DSM HW Advance Features API
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
#ifndef _HW_ADVANCE_FEATURES_H_
#define _HW_ADVANCE_FEATURES_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "BlueNRG1_conf.h"

/* Exported types ------------------------------------------------------- */
typedef enum
{
    ACC_NOT_USED     = 0x00,
    ACC_6D_OR_TOP    = 0x01,
    ACC_6D_OR_LEFT   = 0x02,
    ACC_6D_OR_BOTTOM = 0x03,
    ACC_6D_OR_RIGTH  = 0x04,
    ACC_6D_OR_UP     = 0x05,
    ACC_6D_OR_DOWN   = 0x06,
    ACC_TILT         = 0x08,
    ACC_FREE_FALL    = 0x10,
    ACC_SINGLE_TAP   = 0x20,
    ACC_DOUBLE_TAP   = 0x40,
    ACC_WAKE_UP      = 0x80,
	ACC_PEDOMETER    = 0x80
} AccEventType;

typedef enum
{
    ALL_INT_REG_FF_IA           = 0x01,//Free-fall event detection status.
    ALL_INT_REG_WU_IA           = 0x02,//Wakeup event detection status.
    ALL_INT_REG_SINGLE_TAP      = 0x04,//Single-tap event status.
    ALL_INT_REG_DOUBLE_TAP      = 0x08,//Double-tap event status.
    ALL_INT_REG_6D_IA           = 0x10,//Source of change in position portrait/landscape/face-up/face-down.
    ALL_INT_REG_SLEEP_CHANGE_IA = 0x20,//Sleep change status
} ALL_INT_SRC_Register_Msk;

/* Exported functions ------------------------------------------------------- */

void HWFeaturesCallback(uint8_t irqPin);

void InitHWFeatures(void);

void EnableHWMultipleEvents(void);
void DisableHWMultipleEvents(void);

void EnableSWPedometer(void);
void DisableSWPedometer(void);
void ResetSWPedometer(void);
uint16_t GetStepSWPedometer(void);

void EnableSWTilt (void);
void DisableSWTilt(void);

void EnableHWFreeFall (void);
void DisableHWFreeFall(void);

void EnableHWDoubleTap (void);
void DisableHWDoubleTap(void);

void EnableHWSingleTap (void);
void DisableHWSingleTap(void);

void EnableHWWakeUp (void);
void DisableHWWakeUp(void);

void EnableHWOrientation6D (void);
void DisableHWOrientation6D(void);
AccEventType GetHWOrientation6D(void);

/* Exported variables */
extern uint32_t HWAdvanceFeaturesStatus;

/* Exported defines */
#define W2ST_HWF_PEDOMETER        (1   )
#define W2ST_HWF_FREE_FALL        (1<<1)
#define W2ST_HWF_DOUBLE_TAP       (1<<2)
#define W2ST_HWF_SINGLE_TAP       (1<<3)
#define W2ST_HWF_WAKE_UP          (1<<4)
#define W2ST_HWF_TILT             (1<<5)
#define W2ST_HWF_6DORIENTATION    (1<<6)
#define W2ST_HWF_MULTIPLE_EVENTS  (1<<7)

#define W2ST_CHECK_HW_FEATURE(Feature) ((HWAdvanceFeaturesStatus&(Feature)) ? 1 : 0)
#define W2ST_ON_HW_FEATURE(Feature)    (HWAdvanceFeaturesStatus|=(Feature))
#define W2ST_OFF_HW_FEATURE(Feature)   (HWAdvanceFeaturesStatus&=(~Feature))


#ifdef __cplusplus
}
#endif

#endif /* _HW_ADVANCE_FEATURES_H_ */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
