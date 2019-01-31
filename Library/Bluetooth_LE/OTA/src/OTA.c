/**
 ******************************************************************************
 * @file    OTA.c
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

/* Includes ------------------------------------------------------------------*/
#include "OTA.h"
#include "AT25XE041B_Driver.h"

#include "steval_bluemic1.h"

/* Private defines begin -----------------------------------------------------*/
#define SPI_FLASH_READ_BYTES        (AT25XE041B_ReadByteArray)
#define SPI_FLASH_WRITE_WITH_ERASE  (AT25XE041B_WriteByteArray)
#define SPI_FLASH_WRITE             (AT25XE041B_WriteByteArrayWithoutErase)
#define SPI_FLASH_ERASE_PAGES       (AT25XE041B_ErasePages)
#define SPI_FLASH_OK                (AT25XE041B_OK)

/* Private defines end -------------------------------------------------------*/


/* Private function prototypes begin -----------------------------------------*/
static BOOL checkCrc(uint32_t value);
static void countCrc32(uint32_t* crc, uint8_t* data, uint32_t size);
/* Private function prototypes end  ------------------------------------------*/


/* Private variables begin ---------------------------------------------------*/
static uint32_t aspectedCRCValue = 0;
static uint32_t sizeOfUpdate = 0;
/* Private variables end -----------------------------------------------------*/

/* Public functions realization begin ----------------------------------------*/

/**
 * @brief  Start Function for Updating the Firmware
 * @param  SizeOfUpdate  size of the firmware image [bytes]
 * @param  uwCRCValue    aspected CRV value
 * @retval None
 */
BOOL StartUpdateFWBlueMS(uint32_t updateSize, uint32_t uwCRCValue)
{
    sizeOfUpdate = updateSize;
    aspectedCRCValue = uwCRCValue;
    uint32_t tmp[] = {updateSize, uwCRCValue};
    uint16_t writtenData = 0;
    
    uint32_t pagesToClear = (updateSize + 8) / OTA_FLASH_PAGE_SIZE;//8 bytes for size and crc
    if(((updateSize + 8) % OTA_FLASH_PAGE_SIZE) != 0){
        pagesToClear += 1;
    }
    
    if(SPI_FLASH_ERASE_PAGES(OTA_ADDRESS_START, pagesToClear) != SPI_FLASH_OK){
        goto fail;
    }
    
    if(SPI_FLASH_WRITE(OTA_ADDRESS_START, (uint8_t*)tmp, &writtenData, sizeof(tmp)) != SPI_FLASH_OK){
        goto fail;
    }
    
    return TRUE;
    
    fail:
    return FALSE;
}

/**
 * @brief  Function for Updating the Firmware
 * @param  SizeOfUpdate  Remaining size of the firmware image [bytes]
 * @param  att_data      attribute data
 * @param  data_length   length of the data
 * @retval Return value for checking purpouse (1/-1 == Ok/Error)
 */
int8_t UpdateFWBlueMS(uint32_t* size, uint8_t* att_data, uint8_t data_length)
{
    static uint32_t address = OTA_ADDRESS_START + 8;
    uint16_t writtenData = 0;
   
    /* Save the Packed received */
    if(data_length > (*size)){
        /* Too many bytes...Something wrong... necessity to send it again... */
        *size = 0;
        goto fail;
    }
    
    /* Save the received OTA packed to spi flash */
    if(SPI_FLASH_WRITE(address, att_data, &writtenData, data_length) != SPI_FLASH_OK){
//    if(SPI_FLASH_WRITE_WITH_ERASE(address, att_data, &writtenData, data_length) != SPI_FLASH_OK){
        goto fail;
    }
    
    if(writtenData != data_length){
        goto fail;
    }

    // update address
    address += writtenData;
    
    // Reduce the remaing bytes for OTA completition
    *size -= data_length;

    if(*size == 0) {
        //reset address
        address = OTA_ADDRESS_START + 8;
        
        /* We had received the whole firmware and we have saved it in Flash */
        if(!checkCrc(aspectedCRCValue)) {
            //wrong crc
            goto fail;
        }
        
        goto success;
    }
    
    return 0;
    
    success: 
    return 1;
    
    /* Reset for Restarting again */
    fail: 
    return -1;
}

/* Public functions realization end ------------------------------------------*/

/* Private functions realization begin ---------------------------------------*/

/**
 * @brief  Check if crc of written data is the same as value
 * @param  value crc value
 * @retval TRUE or FALSE
 */
static BOOL checkCrc(uint32_t value)
{
    volatile uint32_t crc = 0xFFFFFFFF;
    uint32_t address = OTA_ADDRESS_START + 8;// 8 bytes for size and crc
    uint32_t readBytes = 0;
    uint8_t data[OTA_FLASH_PAGE_SIZE] = {0};
    
    while(readBytes < sizeOfUpdate){
        uint16_t bytesToRead = 0;
        if((sizeOfUpdate - readBytes) > OTA_FLASH_PAGE_SIZE){
            bytesToRead = OTA_FLASH_PAGE_SIZE;
        } else {
            bytesToRead = (sizeOfUpdate - readBytes);
        }
        if(SPI_FLASH_READ_BYTES(address, (uint8_t*)data, bytesToRead) != SPI_FLASH_OK){
            goto fail;
        }
        countCrc32((uint32_t*)&crc, data, bytesToRead);
        readBytes += bytesToRead;
        address += bytesToRead;
    }

    if(crc != value){
        goto fail;
    }

    return TRUE;
    
    fail: return FALSE;
}

/**
 * @brief  Calculate crc32 of the array proceeding from the previous value.
 * @param  crc  the pointer to previously calculated crc value.
 *              The initial value of crc must be 0xFFFFFFFF.
 * @param  data the array of values for calculating
 * @param  size the size of the array
 * @retval none
 */
static void countCrc32(uint32_t* crc, uint8_t* data, uint32_t size)
{
    //ST BlueMS has a bug and does not calculate last bytes 
    //if their quantity is less then 4 (one word)
//    uint32_t last_data;
	uint32_t numOfDWord = size >> 2;
//	uint32_t numOfTailByte = size & 3;

	while (numOfDWord--){
		*crc = *crc ^ *((uint32_t*)data);
		data += 4;

		for (int i = 0; i < 32; i++)
		{
			if (*crc & 0x80000000)
				*crc = (*crc << 1) ^ 0x04C11DB7;// Polynomial used in STM32
			else
				*crc = (*crc << 1);
		}
	}
    
//	switch (numOfTailByte)
//	{
//	case 0:
//		return;
//	case 1:
//		last_data = data[0] << 24;
//		break;
//	case 2:
//		last_data = *((uint16_t *)(&data[0]));
//		last_data <<= 16;
//		break;
//	case 3:
//		last_data = *((uint16_t *)(&data[0]));
//		last_data <<= 8;
//		last_data += data[2] << 24;
//		break;
//	}
//    
//	countCrc32(crc, (uint8_t*)&last_data, 4);
}

/* Private functions realization end -----------------------------------------*/

/************************ (C) COPYRIGHT SensiEdge *****************************/
