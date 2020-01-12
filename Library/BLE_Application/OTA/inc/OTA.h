/**
 ******************************************************************************
 * @file    OTA.h
 * @date    27-November-2018
 * @brief   This file implements OTA feature.
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
#ifndef __OTA_H
#define __OTA_H

/* Includes ------------------------------------------------------------------*/
#include "steval_bluemic1.h"

/* Public defines ------------------------------------------------------------*/
#define OTA_FLASH_PAGE_SIZE     (AT25XE041B_PAGESIZE) 
#define OTA_MAX_PROG_SIZE       (0x24800)//must be multiple to OTA_FLASH_PAGE_SIZE
#define OTA_MAX_PAGES_QUANTITY  ((OTA_MAX_PROG_SIZE) / (OTA_FLASH_PAGE_SIZE))
#define OTA_ADDRESS_START       (0)

#define BOOTLOADER_START_UPDATE_COMMAND (0xAAAABBBB)
#define BOOTLOADER_CONFIG_PAGE_ADDR     (0x10041800)
#define BOOTLOADER_CONFIG_PAGE_NUM      (((BOOTLOADER_CONFIG_PAGE_ADDR) - (FLASH_START)) / (N_BYTES_PAGE))
/* Public variables begin ----------------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/
/**
 * @brief  Start Function for Updating the Firmware
 * @param  updateSize   size of the firmware image [bytes]
 * @param  uwCRCValue   aspected CRV value
 * @retval None
 */
BOOL StartUpdateFWBlueMS(uint32_t updateSize, uint32_t uwCRCValue);

/**
 * @brief  Function for Updating the Firmware
 * @param  size         Remaining size of the firmware image [bytes]
 * @param  att_data     attribute data
 * @param  data_length  length of the data
 * @retval Return value for checking purpouse (1/-1 == Ok/Error)
 */
int8_t UpdateFWBlueMS(uint32_t* size, uint8_t* att_data, uint8_t data_length);
#endif 

/************************ (C) COPYRIGHT SensiEdge *****************************/
