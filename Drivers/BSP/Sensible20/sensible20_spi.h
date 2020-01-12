/**
  ******************************************************************************
  * @file    sensible20_spi.h
  * @author  AMS VMA RF application team
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
  * <h2><center>&copy; COPYRIGHT STMicroelectronics</center></h2>
  ******************************************************************************
  */

#include "BlueNRG1_spi.h"
#include "main.h"

#define BlueNRG1_MAX_DELAY   0xFFFFFFFFU

typedef enum
{
    BlueNRG1_SPI_OK       = 0x00,
    BlueNRG1_SPI_ERROR    = 0x01,
    BlueNRG1_SPI_TIMEOUT  = 0x02,
} BlueNRG1_SPI_StatusTypeDef;

void BlueNRG1_SPI_Init();
void BlueNRG1_SPI_FlushRxFifo();
BlueNRG1_SPI_StatusTypeDef BlueNRG1_SPI_Transmit(uint8_t* msg, uint16_t size, uint32_t timeout);
BlueNRG1_SPI_StatusTypeDef BlueNRG1_SPI_Receive(uint8_t *pData, uint16_t size, uint32_t timeout);

/******************* (C) COPYRIGHT STMicroelectronics **********END OF FILE****/
