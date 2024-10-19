/**
 ******************************************************************************
 * @file    sensible20_port_exp.c
 * @brief   PortExpander driver file.
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

#include "sensible20_port_exp.h"

/* Register list -------------------------------------------------------------*/
typedef enum {
    PE_RegId             = 0x01,
    PE_RegIoDir          = 0x03,
    PE_RegOutState       = 0x05,
    PE_RegOutHiz         = 0x07,
    PE_RegInDefaultState = 0x09,
    PE_RegPullEn         = 0x0B,
    PE_RegPullUpDn       = 0x0D,
    PE_RegInStatus       = 0x0F,
    PE_RegIrqMask        = 0x11,
    PE_RegIrqStat        = 0x13,
} PE_Regs_t;

/* Local data ----------------------------------------------------------------*/
static struct {
    uint8_t address;
} PE_State;

/* Static fucntions prototypes -----------------------------------------------*/
static uint8_t PE_SetDirection(uint8_t pins, PE_Direction_t dir);
static uint8_t PE_WriteIrqMask(uint8_t pin, PE_IrqMask_t irq);
static uint8_t PE_WriteReg(PE_Regs_t reg, uint8_t val);
static uint8_t PE_ReadReg(PE_Regs_t reg, uint8_t * val);
static uint8_t PE_SetHiz(uint8_t pins, PE_OutHiZ_t state);
static uint8_t PE_SetPull(uint8_t pins, PE_PullUpDn_t val);

/* Public fucntions ----------------------------------------------------------*/
void PE_Init(PE_Addr_t addr, PE_InitStruct_t* conf)
{
    static uint8_t isInitialized = 0;
    uint8_t regval = 0;

    if(addr == PE_Addr0 ||
       addr == PE_Addr1) {
           PE_State.address = addr;
       } else {
           PE_State.address = PE_NotAnAddr;
           return;
       }
    
    if(isInitialized == 0){
        isInitialized = 1;
        PE_WriteReg(PE_RegIrqMask, 0xFF);
    }

   if(0 == PE_ReadReg(PE_RegId, &regval)) {
       return;
   }
   PE_SetPull(conf->pins, conf->pull);
   PE_WriteIrqMask(conf->pins, conf->irqMask);
   PE_SetDirection(conf->pins, conf->dir);
   PE_SetHiz(conf->pins, PE_OutHiZDis);
   PE_SetPin(conf->pins);
}

void PE_WritePin(uint8_t pin, uint8_t value)
{
    if(value == 0) {
        PE_ResetPin(pin);
    } else {
        PE_SetPin(pin);
    }
}

void PE_SetPin(uint8_t pins)
{
    uint8_t val = 0;
    
    if(0 != PE_ReadReg(PE_RegOutState, &val)) {
        val |= pins;
        PE_WriteReg(PE_RegOutState, val);
    }
}

void PE_ResetPin(uint8_t pins)
{
    uint8_t val = 0;
    
    if(0 != PE_ReadReg(PE_RegOutState, &val)) {
        val &= ~pins;
        PE_WriteReg(PE_RegOutState, val);
    }
}

void PE_TogglePin(uint8_t pins)
{
    uint8_t val = 0;
    
    if(0 != PE_ReadReg(PE_RegOutState, &val)) {
        val ^= pins;
        PE_WriteReg(PE_RegOutState, val);
    }
}

uint8_t PE_ReadPin(PE_Pins_t pin)
{
    uint8_t val = 0;
    PE_ReadReg(PE_RegIoDir, &val);
    
    if((val & (uint8_t) pin) != 0) {
        // In the case of the output mode,
        // we read the pin state from out state register
        PE_ReadReg(PE_RegOutState, &val);
    } else {
        // Input status register shows the state of input pins only
        PE_ReadReg(PE_RegInStatus, &val);
    }
    
    return ((val & (uint8_t)pin) == 0) ? 0 : 1;
}

uint8_t PE_Read(void)
{
    uint8_t val = 0;
    // Input status register shows the state of input pins only
    PE_ReadReg(PE_RegInStatus, &val);
    return val;
}

uint8_t PE_ReadIntStatus(void)
{
    uint8_t val = 0;
    PE_ReadReg(PE_RegIrqStat, &val);
    return val;
}

/* Local fucntions -----------------------------------------------------------*/
static uint8_t PE_WriteIrqMask(uint8_t pin, PE_IrqMask_t irq)
{
    uint8_t val = 0;
    
    if(0 == PE_ReadReg(PE_RegIrqMask, &val)) {
        return 0;
    }
    
    if(irq == PE_NonGenerateInt) {
        val |= pin;
    } else {
        val &= ~pin;
    }
    
    return PE_WriteReg(PE_RegIrqMask, val);
}

static uint8_t PE_SetHiz(uint8_t pins, PE_OutHiZ_t state)
{
    uint8_t val = 0;
    
    if(0 == PE_ReadReg(PE_RegOutHiz, &val)) {
        return 0;
    }
    
    if(state == PE_OutHiZEn) {
        val |= pins;
    } else {
        val &= ~pins;
    }
    
    return PE_WriteReg(PE_RegOutHiz, val);
}

static uint8_t PE_SetPull(uint8_t pins, PE_PullUpDn_t val)
{
    
    uint8_t reg = 0;
    if(!PE_ReadReg(PE_RegPullEn, &reg)) {
        return 0;
    }
    
    if(val == PE_PullDis) {
        reg &= ~(pins);
    } else {
        reg |= pins;
    }
    
    if(!PE_WriteReg(PE_RegPullEn, reg)) {
        return 0;
    }
    
    if(val == PE_PullDis) {
        // Nothing more to do. return;
        return 1;
    }
       
    if(!PE_ReadReg(PE_RegPullUpDn, &reg)) {
        return 0;
    }

    if(val == PE_PullUp) {
        reg |= pins;
    } else {
        reg &= ~(pins);
    }
    if(!PE_WriteReg(PE_RegPullUpDn, reg)) {
        return 0;
    }
    
    return 1;
}

static uint8_t PE_SetDirection(uint8_t pins, PE_Direction_t dir)
{
    uint8_t val = 0;
    
    if(0 == PE_ReadReg(PE_RegIoDir, &val)) {
        return 0;
    }
    
    if(dir == PE_DirectionOut) {
        val |= pins;
    } else {
        val &= ~pins;
    }
    
    return PE_WriteReg(PE_RegIoDir, val);
}

static uint8_t PE_WriteReg(PE_Regs_t reg, uint8_t val)
{
    if(PE_State.address == PE_NotAnAddr) {
        return 0;
    }
    if(COMPONENT_OK != StevalBlueMic1_I2CWrite(&val, PE_State.address, (uint8_t)reg, 1)) {
        return 0;
    }
    return 1;
}

static uint8_t PE_ReadReg(PE_Regs_t reg, uint8_t * val)
{
    if(val == NULL || PE_State.address == PE_NotAnAddr) {
        return 0;
    }
    
    if(COMPONENT_OK != StevalBlueMic1_I2CRead(val,PE_State.address,(uint8_t)reg,1))
    {
        return 0;
    }
    return 1;
}

/************************ (C) COPYRIGHT SensiEdge *****************************/
