/**
 ******************************************************************************
 * @file    steval_bluemic1_lux_apds9250.c
 * @brief   apds9250 driver file.
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

#include "steval_bluemic1_lux.h"
#include "steval_bluemic1.h"

// I2C address
#define APDS9250_I2C_ADDRESS           (0x52)

// Register definitions
#define APDS9250_REG_MAIN_CTRL          0x00
#define APDS9250_REG_LS_MEAS_RATE       0x04
#define APDS9250_REG_LS_GAIN            0x05
#define APDS9250_REG_PART_ID            0x06
#define APDS9250_REG_MAIN_STATUS        0x07
#define APDS9250_REG_DATA_IR_0          0x0A
#define APDS9250_REG_DATA_IR_1          0x0B
#define APDS9250_REG_DATA_IR_2          0x0C
#define APDS9250_REG_DATA_GREEN_0       0x0D
#define APDS9250_REG_DATA_GREEN_1       0x0E
#define APDS9250_REG_DATA_GREEN_2       0x0F
#define APDS9250_REG_DATA_BLUE_0        0x10
#define APDS9250_REG_DATA_BLUE_1        0x11
#define APDS9250_REG_DATA_BLUE_2        0x12
#define APDS9250_REG_DATA_RED_0         0x13
#define APDS9250_REG_DATA_RED_1         0x14
#define APDS9250_REG_DATA_RED_2         0x15
#define APDS9250_REG_INT_CFG            0x19
#define APDS9250_REG_INT_PERSISTENCE    0x1A
#define APDS9250_REG_THRES_UP_0         0x21
#define APDS9250_REG_THRES_UP_1         0x22
#define APDS9250_REG_THRES_UP_2         0x23
#define APDS9250_REG_THRES_LOW_0        0x24
#define APDS9250_REG_THRES_LOW_1        0x25
#define APDS9250_REG_THRES_LOW_2        0x26
#define APDS9250_REG_THRES_VAR          0x27

// Register values
#define APDS9250_PART_ID                0xB2


static uint8_t LuxInitialized = 0;


static LUX_StatusTypeDef APDS9250_IO_Init(void);
static LUX_StatusTypeDef APDS9250_IO_Read(uint8_t reg, uint8_t *buf, uint16_t count);
static LUX_StatusTypeDef APDS9250_IO_Write(uint8_t reg, uint8_t *buf, uint16_t count);


LUX_StatusTypeDef BSP_LUX_Init(void)
{
    if (!LuxInitialized)
    {
        uint8_t gain = 0x01; // gain = 3;
        uint8_t meas_rate = 0x22; // 18bit, 100 mS

        if (APDS9250_IO_Init() != LUX_OK)
        {
            return LUX_ERROR;
        }

        // Set gain
        if (APDS9250_IO_Write(APDS9250_REG_LS_GAIN, &gain, 1) != LUX_OK)
        {
            return LUX_ERROR;
        }
        // Set measurement rate
        if (APDS9250_IO_Write(APDS9250_REG_LS_MEAS_RATE, &meas_rate, 1) != LUX_OK)
        {
            return LUX_ERROR;
        }
        LuxInitialized = 1;
    }

    return LUX_OK;
}


uint8_t BSP_LUX_IsInitalized(void)
{
    return LuxInitialized;
}


LUX_StatusTypeDef BSP_LUX_PowerON(void)
{
    uint8_t main_ctrl = 0x02; // ALS, active
    //uint8_t main_ctrl = 0x06; // RGB, active
    return APDS9250_IO_Write(APDS9250_REG_MAIN_CTRL, &main_ctrl, 1);
}


LUX_StatusTypeDef BSP_LUX_PowerOFF(void)
{
    uint8_t main_ctrl = 0;
    return APDS9250_IO_Write(APDS9250_REG_MAIN_CTRL, &main_ctrl, 1);
}


uint8_t BSP_LUX_IsDataReady(void)
{
    LUX_StatusTypeDef ret;
    uint8_t main_status;

    ret = APDS9250_IO_Read(APDS9250_REG_MAIN_STATUS, &main_status, 1);
    if (ret != LUX_OK)
    {
        return 0;
    }

    return (main_status & 0x08) ? 1 : 0;
}


LUX_StatusTypeDef BSP_LUX_GetValue(uint16_t *pData)
{
    LUX_StatusTypeDef ret;
    uint8_t adc_data[6];

    ret = APDS9250_IO_Read(APDS9250_REG_DATA_IR_0, adc_data, 6);
    if (ret == LUX_OK)
    {
        uint32_t ir_value    = adc_data[0] | (adc_data[1] << 8) | (adc_data[2] << 16);
        uint32_t green_value = adc_data[3] | (adc_data[4] << 8) | (adc_data[5] << 16);

        uint32_t factor = ir_value > green_value ? 35 : 46;

        uint32_t lux = ((green_value * factor) / 3) / 100;
        *pData = (uint16_t)lux;
    }

    return ret;
}



LUX_StatusTypeDef APDS9250_IO_Init(void)
{
    // // Init I2C
    // if(I2C_Global_Init() != HAL_OK)
    // {
    //     return LUX_ERROR;
    // }
    return LUX_OK;
}


static LUX_StatusTypeDef APDS9250_IO_Read(uint8_t reg, uint8_t *buf, uint16_t count)
{
    LUX_StatusTypeDef ret_val = LUX_OK;

    
    if (COMPONENT_OK != StevalBlueMic1_I2CRead(buf, APDS9250_I2C_ADDRESS, reg, count))
    {
        ret_val = LUX_ERROR;
    }

    return ret_val;
}


static LUX_StatusTypeDef APDS9250_IO_Write(uint8_t reg, uint8_t *buf, uint16_t count)
{
    LUX_StatusTypeDef ret_val = LUX_OK;

    if (COMPONENT_OK != StevalBlueMic1_I2CWrite(buf, APDS9250_I2C_ADDRESS, reg, count))
    {
        ret_val = LUX_ERROR;
    }

    return ret_val;
}

/************************ (C) COPYRIGHT SensiEdge *****************************/
