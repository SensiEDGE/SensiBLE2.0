/**
 ******************************************************************************
 * @file    sensible20_port_exp.h
 * @brief   PortExpander driver header file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USE_SENSIBLE2_PORT_EXPANDER_H
#define __USE_SENSIBLE2_PORT_EXPANDER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "steval_bluemic1.h"
#include "sensible20_port_exp.h"
    
/* Types ---------------------------------------------------------------------*/
    typedef enum {
        PE_Pin_0 = 1 << 0,
        PE_Pin_1 = 1 << 1,
        PE_Pin_2 = 1 << 2,
        PE_Pin_3 = 1 << 3,
        PE_Pin_4 = 1 << 4,
        PE_Pin_5 = 1 << 5,
        PE_Pin_6 = 1 << 6,
        PE_Pin_7 = 1 << 7,
        PE_Pin_All = 0xFF
    } PE_Pins_t;
    
    typedef enum {
        PE_Addr0 = 0x43,
        PE_Addr1 = 0x44,
        PE_NotAnAddr = 0xFF
    } PE_Addr_t;
    
    typedef enum {
        PE_DirectionIn = 0,
        PE_DirectionOut = 1,
    } PE_Direction_t;
    
    typedef enum {
        PE_PullDis = 0,
        PE_PullUp,
        PE_PullDn
    } PE_PullUpDn_t;
    
    typedef enum {
        PE_OutHiZDis = 0,
        PE_OutHiZEn
    } PE_OutHiZ_t;
    
    typedef enum {
        PE_GenerateInt = 0,
        PE_NonGenerateInt
    } PE_IrqMask_t;
    
    typedef struct {
        uint8_t        pins;
        PE_Direction_t dir;
        PE_PullUpDn_t  pull;
        PE_IrqMask_t   irqMask;
    } PE_InitStruct_t;
    
/* Prototypes ----------------------------------------------------------------*/
    void PE_Init          (PE_Addr_t addr, PE_InitStruct_t* conf);
    void PE_WritePin      (uint8_t pin, uint8_t value);
    void PE_SetPin        (uint8_t pins);
    void PE_ResetPin      (uint8_t pins);
    void PE_TogglePin     (uint8_t pins);
    uint8_t PE_ReadPin    (PE_Pins_t pin);
    uint8_t PE_Read       (void);
    uint8_t PE_ReadIntStatus(void);

#endif /* __USE_SENSIBLE2_PORT_EXPANDER_H */

/************************ (C) COPYRIGHT SensiEdge *****************************/
