/**
 ******************************************************************************
 * @file    sensible20_led.c
 * @brief   LED driver file.
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

#include "sensible20_led.h"
#include "sensible20_port_exp.h"

static PE_Addr_t SENSI_PeAddr = PE_Addr1;
static PE_Pins_t SENSI_LedPins[SENSI_LedCount] = {PE_Pin_1, PE_Pin_5};

void SENSI_LedInit (SENSI_Led_t led)
{
    if(led >= SENSI_LedCount) {
        return;
    }
    
    PE_InitStruct_t init = {
        .pins    = SENSI_LedPins[led],
        .dir     = PE_DirectionOut,
        .pull    = PE_PullDis,
        .irqMask = PE_NonGenerateInt
    };
    
    PE_Init(SENSI_PeAddr, &init);
    
    SENSI_LedOff(led);
}

void SENSI_LedOn (SENSI_Led_t led)
{
    PE_SetPin(SENSI_LedPins[led]);
}

void SENSI_LedOff (SENSI_Led_t led)
{
    PE_ResetPin(SENSI_LedPins[led]);
}

void SENSI_LedToggle (SENSI_Led_t led)
{
    PE_TogglePin(SENSI_LedPins[led]);
}

void SENSI_LedSetState (SENSI_Led_t led, SENSI_LedState_t state)
{
    if(state == SENSI_LedStateOff) {
        PE_ResetPin(SENSI_LedPins[led]);
    } else {
        PE_SetPin(SENSI_LedPins[led]);
    }
}

SENSI_LedState_t SENSI_LedGetState (SENSI_Led_t led)
{
    if(0 == PE_ReadPin(SENSI_LedPins[led])) {
        return SENSI_LedStateOff;
    } else {
        return SENSI_LedStateOn;
    }
}

/************************ (C) COPYRIGHT SensiEdge *****************************/
