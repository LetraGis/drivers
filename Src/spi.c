/**
 ******************************************************************************
 * @file           : spi.c
 * @author         : Letra Gis
 * @brief          : This files contains API's for using SPI Peripheral. A
 * 					 brief description of each API capabilities can be found on
 * 					 in its header.
 ******************************************************************************
 */

/******************************************************************************
EXTERNAL DEPENDENCIES
******************************************************************************/

#include "spi.h"
#include "sys_f446xx.h"
#include "stddef.h"

/******************************************************************************
MODULE DEFINES
******************************************************************************/

/******************************************************************************
MODULE TYPES
******************************************************************************/

/******************************************************************************
DECLARATION OF LOCAL FUNCTIONS
******************************************************************************/

/******************************************************************************
DEFINITION OF LOCAL VARIABLES
******************************************************************************/

/******************************************************************************
DEFINITION OF EXPORTED VARIABLES
******************************************************************************/

/******************************************************************************
DEFINITION OF LOCAL CONSTANT DATA
******************************************************************************/

/******************************************************************************
DEFINITION OF EXPORTED CONSTANT DATA
******************************************************************************/

/******************************************************************************
MODULE FUNCTION-LIKE MACROS
******************************************************************************/

/******************************************************************************
DEFINITION OF APIs
******************************************************************************/

/*!****************************************************************************
 * @brief			Calls callback function to perform SPI initialization.
 * @details		   	Initializes SPI by calling function Spi_InitCallback. The
 * 					implementation needs to be designed by the developer.
 * @param[in]      	void    No input parameters.
 * @return         	void    No output parameters.
 ******************************************************************************/
void Spi_Init(void)
{
    Spi_InitCallback();
}

/*!****************************************************************************
 * @brief			Resets all SPI related registers to its reset state.
 * @details		   	By means of setting the correpsonding bit on the RCC 
 *                  AHB1/APB2 Reset Registers, all SPI registers will be 
 *                  resetted to its initial values.
 * @param[in]      	void    No input parameters.    
 * @return         	void    No output parameters.
 ******************************************************************************/
void Spi_Deinit(void)
{
    
}

/*!****************************************************************************
 * @brief			Sends data over SPI.
 * @details		   	Sends data over SPI, using the specified data format (8-bit
 *                  or 16-bit) on a blocking call.
 * @param[in]      	pSpiHandle  SPI Data Structure, holds Tx Buffer among other
 *                  flags.
 * @param[in]       pTxBuffer   Pointer to Transmit Buffer
 * @param[in]       len         Length of data to be sent (bytes)
 * @return         	void        No output parameters.
 ******************************************************************************/
void Spi_SendData(SPI_TypeDef *pSpiHandle, uint8_t *pTxBuff, uint32_t len)
{
    while (len > 0)
    {
        /* Wait until TXE Flag is set. When set, it indicates that Tx Buffer 
        is empty and we can load data into DR. */
        while (SPI_FLAG_SET != (Spi_GetFlagStatus(pSpiHandle, TXE_Flag)));
        /* DFF = 1 byte */
        if(SPI_CR1_DFF != (pSpiHandle->CR1 & SPI_CR1_DFF_Msk))
        {
            pSpiHandle->DR = *pTxBuff;
            len--;
            pTxBuff++;
        }
        /* DFF = 2 bytes */
        else
        {
            pSpiHandle->DR = *((uint16_t*)pTxBuff);
            len-=DFF_TWO_BYTES;
            /* Pointer is casted to uint16_t (2 bytes) to increment it
            according to DFF of 2 bytes. */
            (uint16_t*)pTxBuff++;   
        }
    }
}

/*!****************************************************************************
 * @brief			Receives data over SPI.
 * @details		   	Receives data over SPI, using the specified data format 
 *                  (8-bit or 16-bit) on a blocking call.
 * @param[in]      	pSpiHandle  SPI Data Structure, holds Rx Buffer among other
 *                  flags.
 * @param[in]       pRxBuffer   Pointer to Receive Buffer
 * @param[in]       len         Length of data to be received (bytes)
 * @return         	void        No output parameters.
 ******************************************************************************/
void Spi_ReceiveData(SPI_TypeDef *pSpiHandle, uint8_t *pRxBuff, uint32_t len)
{
    while (len > 0)
    {
        /* Wait until RXNE Flag is set. When set, it indicates that Rx Buffer 
        has content and we can read data from DR. */
        while (SPI_FLAG_SET != (Spi_GetFlagStatus(pSpiHandle, RXNE_Flag)));
        /* DFF = 1 byte */
        if(SPI_CR1_DFF != (pSpiHandle->CR1 & SPI_CR1_DFF_Msk))
        {
            *pRxBuff = pSpiHandle->DR;
            len--;
            pRxBuff++;
        }
        /* DFF = 2 bytes */
        else
        {
            *((uint16_t*)pRxBuff) = pSpiHandle->DR;
            len-=DFF_TWO_BYTES;
            /* Pointer is casted to uint16_t (2 bytes) to increment it
            according to DFF of 2 bytes. */
            (uint16_t*)pRxBuff++;
        }
    }
}

/*!****************************************************************************
 * @brief			Sends and Receives data over SPI in Full-Duplex mode.
 * @details		   	Sends and Receives data over SPI (in Full-Duplex mode) in
 *                  blocking call. In Full-Duplex mode, both MISO and MOSI
 *                  lines are used to exchange data between Controller and
 *                  Peripheral. While transmitting from Controller, Peripheral
 *                  also sends data, which is transferred as the return from
 *                  the function.
 * @param[in]      	pSpiHandle  SPI Data Structure, holds Tx and Rx Buffers,
 *                              among other flags.
 * @param[in]       pTxBuffer   Pointer to Transmit Buffer
 * @return         	void        No output parameters.
 ******************************************************************************/
uint16_t Spi_SendReceiveData(SPI_TypeDef *pSpiHandle, uint16_t TxBuff)
{
    /* Send Data. Waits until Tx Buffer is empty before putting data into the
    Data Register. */
    Spi_WriteData(pSpiHandle, TxBuff);
    /* Receive Data. Waits until Rx Buffer is bull before reading data from the
    Data Register. */
    return Spi_ReadData(pSpiHandle);
}

/******************************************************************************
End Of File
******************************************************************************/
