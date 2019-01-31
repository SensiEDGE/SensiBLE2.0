/**
  ******************************************************************************
  * @file    sensible20_spi.c
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

#include "sensible20_spi.h"
#include "BlueNRG1_gpio.h"
#include "BlueNRG1_sysCtrl.h"

/* Private defines begin -----------------------------------------------------*/
#define BLUENRG1_SPI_BAUDRATE   8000000

#define BLUENRG1_SPI_MOSI_PIN   GPIO_Pin_2
#define BLUENRG1_SPI_MOSI_MODE  Serial0_Mode
#define BLUENRG1_SPI_MISO_PIN   GPIO_Pin_3
#define BLUENRG1_SPI_MISO_MODE  Serial0_Mode
#define BLUENRG1_SPI_CLK_PIN    GPIO_Pin_0
#define BLUENRG1_SPI_CLK_MODE   Serial0_Mode
/* Private defines end -------------------------------------------------------*/

/* Public functions realization begin ----------------------------------------*/
/**
* @brief   Private function. Init spi periferal.
*          It is used as a wrapper for platform-dependent functions.
* @param   none.
* @retval  none.
*/
void BlueNRG1_SPI_Init()
{
    SPI_InitType SPI_InitStructure;
    GPIO_InitType GPIO_InitStructure;
    
    /* Enable SPI and GPIO clocks */
    SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO | CLOCK_PERIPH_SPI, ENABLE);   
    
    /* Configure SPI pins */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = BLUENRG1_SPI_MOSI_PIN;
    GPIO_InitStructure.GPIO_Mode = BLUENRG1_SPI_MOSI_MODE;
    GPIO_InitStructure.GPIO_Pull = ENABLE;
    GPIO_InitStructure.GPIO_HighPwr = DISABLE;
    GPIO_Init(&GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = BLUENRG1_SPI_MISO_PIN;
    GPIO_InitStructure.GPIO_Mode = BLUENRG1_SPI_MISO_MODE;
    GPIO_Init(&GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = BLUENRG1_SPI_CLK_PIN;
    GPIO_InitStructure.GPIO_Mode = BLUENRG1_SPI_CLK_MODE;
    GPIO_Init(&GPIO_InitStructure);
    
    /* Configure SPI in master mode */
    SPI_StructInit(&SPI_InitStructure);
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b ;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_BaudRate = BLUENRG1_SPI_BAUDRATE;
    SPI_Init(&SPI_InitStructure);
    
    /* Clear RX and TX FIFO */
    SPI_ClearTXFIFO();
    SPI_ClearRXFIFO();
    
    /* Set null character */
    SPI_SetDummyCharacter(0xFF);
    
    /* Set communication mode */
    SPI_SetMasterCommunicationMode(SPI_FULL_DUPLEX_MODE);
    
    /* Enable SPI functionality */
    SPI_Cmd(ENABLE);  
    
}

/**
* @brief   Public function. Flush the RX fifo.
* @param   none.
* @retval  none.
*/
void BlueNRG1_SPI_FlushRxFifo()
{
    SPI_ClearRXFIFO();
}

/**
* @brief   Public function. Transmit an amount of data in blocking mode.
* @param   pData     Specify the pointer to data buffer.
* @param   size      Specify the amount of data to be sent.
* @param   timeout   Specify the timeout duration.
* @retval  BlueNRG1_SPI status
*/
BlueNRG1_SPI_StatusTypeDef BlueNRG1_SPI_Transmit(uint8_t* msg, uint16_t size, 
                                                 uint32_t timeout)
{
    if(msg == 0){
        return BlueNRG1_SPI_ERROR;
    }
    if((timeout == 0) || (timeout >= BlueNRG1_MAX_DELAY)){
        return BlueNRG1_SPI_TIMEOUT;
    }

    uint32_t tickstart = lSystickCounter;
    for(uint16_t i = 0; i < size; i++)
    {
        /* Write data to send to TX FIFO */
        while(RESET == SPI_GetFlagStatus(SPI_FLAG_TFE));
        SPI_SendData(msg[i]);
        while(RESET == SPI_GetFlagStatus(SPI_FLAG_RNE));
        SPI_ReceiveData();
        while (SET == SPI_GetFlagStatus(SPI_FLAG_BSY));
        
        if((lSystickCounter - tickstart) >= timeout){
            return BlueNRG1_SPI_TIMEOUT;
        }
    }
    return BlueNRG1_SPI_OK;
}

/**
* @brief   Public function. Receive an amount of data in blocking mode.
*          It is used as a wrapper for platform-dependent functions.
* @param   pData     Specify the pointer to data buffer.
* @param   size      Specify the amount of data to be received.
* @param   timeout   Specify the timeout duration.
* @retval  BlueNRG1_SPI status
*/
BlueNRG1_SPI_StatusTypeDef BlueNRG1_SPI_Receive(uint8_t *pData, uint16_t size, 
                                                uint32_t timeout)
{
    if(pData == 0){
        return BlueNRG1_SPI_ERROR;
    }
    if((timeout == 0) || (timeout >= BlueNRG1_MAX_DELAY)){
        return BlueNRG1_SPI_TIMEOUT;
    }

    uint32_t tickstart = lSystickCounter;
    for(uint16_t i = 0; i < size; i++)
    {
        while(RESET == SPI_GetFlagStatus(SPI_FLAG_TFE));
        SPI_SendData(0);
        while(RESET == SPI_GetFlagStatus(SPI_FLAG_RNE));
        uint32_t tmp = SPI_ReceiveData();
        pData[i] = tmp & 0xFF;
        while (SET == SPI_GetFlagStatus(SPI_FLAG_BSY));
        
        if((lSystickCounter - tickstart) >= timeout){
            return BlueNRG1_SPI_TIMEOUT;
        }
    }
    return BlueNRG1_SPI_OK;
}

/* Public functions realization end ------------------------------------------*/

/******************* (C) COPYRIGHT STMicroelectronics **********END OF FILE****/
