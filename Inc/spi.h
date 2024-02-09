/*
 * spi.h
 *
 *  Created on: Jun 23, 2024
 *      Author: LetraGis
 */

#ifndef SPI_H_
#define SPI_H_

/******************************************************************************
EXTERNAL DEPENDENCIES
******************************************************************************/
#include <stdint.h>
#include "stm32f446xx.h"

/******************************************************************************
DEFINITION OF CONSTANTS
******************************************************************************/

/******************************************************************************
DECLARATION OF TYPES
******************************************************************************/
typedef union Spi_Config1Type_Tag
{
    uint16_t Bytes;
    struct
    {
        uint16_t CPHA       : 1;    /* Clock Phase. Determines clock capture edge */
        uint16_t CPOL       : 1;    /* Clock Polarity. */
        uint16_t MSTR       : 1;    /* Device Selection. 0 - slave, 1 master */
        uint16_t BR         : 3;    /* Baud Rate. Source Clock division ratio */
        uint16_t SPE        : 1;    /* SPI Enable. */
        uint16_t LSBFIRST   : 1;    /* Frame Format. 0-MSB First, 1-LSB First */
        uint16_t SSI        : 1;    /* Internal Slave Select. NSS Pin related */
        uint16_t SSM        : 1;    /* Software Slave Mgt. 0-enabled, 1-disabled */
        uint16_t RXONLY     : 1;    /* Receive Only Mode enabled. */
        uint16_t DFF        : 1;    /* Data Frame Format. 0-8 bits, 1-16 bits */
        uint16_t CRCNEXT    : 1;    /* CRC Transfer Next. 0-data, 1-CRC */
        uint16_t CRCEN      : 1;    /* HW-CRC calculation enabled */
        uint16_t BIDIOE     : 1;    /* Output Enable in bidirectional mode */
        uint16_t BIDIMODE   : 1;    /* Bidirectional data mode enabled. */
    } Fields;
} Spi_Config1Type;

typedef union Spi_Config2Type_Tag
{
    uint8_t Bytes;
    struct
    {      
        uint8_t RXDMAEN     : 1;    /* RxBuffer DMA Enable */
        uint8_t TXDMAEN     : 1;    /* TxBuffer DMA Enable */
        uint8_t SSOE        : 1;    /* Slave Select Output Enable */
        uint8_t             : 1;    /* Reserved */
        uint8_t FRF         : 1;    /* Frame Format. 0-Motorola, 1-TexasI */
        uint8_t ERRIE       : 1;    /* Error Interrupt Enable */
        uint8_t RXNEIE      : 1;    /* RxBuffer Not-Empty Interrupt Enable */
        uint8_t TXEIE       : 1;    /* TxBuffer Empty Interrupt Enable */
    } Fields;
} Spi_Config2Type;

#define SPI_1    1
#define SPI_2    2
#define SPI_3    3
#define SPI_4    4

/* BIDIMODE */
typedef enum
{
    twoLineUni = 0, /* Two-Line Unidirectional Mode */
    oneLineBi,      /* One-Line Bidirectional Mode */
} Spi_BidirectionalMode;

/* BIDIOE */
typedef enum
{
    outDisabled = 0,    /* Output Disabled (receive-only mode) */
    outEnabled,         /* Output Enabled (transmit-only mode) */
} Spi_BidiOutputEnable;

/* DFF */
typedef enum
{
    oneByte = 0,    /* 8-bit Frame */
    twoBytes,       /* 16-bit Frame */
} Spi_DataFormat;

/* SSM */
typedef enum
{
    swMgtDisabled = 0,  /* SW Slave Mgt disabled */
    swMgtEnabled,       /* SW Slave Mgt enabled */
} Spi_SoftwareSlaveMgt;

/* BR */
typedef enum
{
    pclkDivBy2 = 0,
    pclkDivBy4,
    pclkDivBy8,
    pclkDivBy16,
    pclkDivBy32,
    pclkDivBy64,
    pclkDivBy128,
    pclkDivBy256,
} Spi_ClockSpeed;

/* MSTR */
typedef enum
{
    spiSlave = 0,   /* Slave Configuration */
    spiMaster,      /* Master Configuration */
} Spi_DeviceMode;

/* CPOL */
typedef enum
{
    clkPolLowIdle = 0, /* Clock to 0 when idle */
    clkPolHighIdle,      /* Clock to 1 when idle */
} Spi_ClockPolarity;

/* CPHA */
typedef enum
{
    firstEdge = 0,  /* Data Capture: First clock transition */
    secondEdge,     /* Data Capture: Second clock transition */
} Spi_ClockPhase;

typedef enum
{
    fullDuplex = 0,
    halfDuplex,
    simplexRx,
    simplexTx,
} Spi_BusConfig;

/******************************************************************************
DECLARATION OF VARIABLES
******************************************************************************/

/******************************************************************************
DECLARATION OF CONSTANT DATA
******************************************************************************/

/******************************************************************************
DECLARATION OF FUNCTIONS
******************************************************************************/
void Spi_Init(void);
void Spi_InitCallback(void); /* Needs to be defined by the user to initialize
								SPI pin, set clock, etc. */
void Spi_Deinit(void);
void Spi_SendData(SPI_TypeDef *pSpiHandle, uint8_t *pTxBuff, uint32_t len);
void Spi_ReceiveData(SPI_TypeDef *pSpiHandle, uint8_t *pRxBuff, uint32_t len);

/******************************************************************************
DECLARATION OF FUNCTION-LIKE MACROS
******************************************************************************/
/* Enable SPI1 Peripheral */
#define SPI1_ENABLE()	(RCC->APB2ENR |= RCC_APB2ENR_SPI1EN_Msk)
/* Enable SPI2 Peripheral */
#define SPI2_ENABLE()	(RCC->APB1ENR |= RCC_APB1ENR_SPI2EN_Msk)
/* Enable SPI3 Peripheral */
#define SPI3_ENABLE()	(RCC->APB1ENR |= RCC_APB1ENR_SPI3EN_Msk)
/* Enable SPI4 Peripheral */
#define SPI4_ENABLE()	(RCC->APB2ENR |= RCC_APB2ENR_SPI4EN_Msk)

/* Sets the value of Configuration Register 1 to the value of input 'y'
*  of the given SPI peripheral, specified by input 'x'.
*/
#define Spi_Cfg_CR1_Helper(x, y)    SPI##x->CR1 |= (uint32_t)y
#define Spi_Config_CR1(x, y)        Spi_Cfg_CR1_Helper(x, y)

/* Sets the value of Configuration Register 2 to the value of input 'y'
*  of the given SPI peripheral, specified by input 'x'.
*/
#define Spi_Cfg_CR2_Helper(x)       SPI##x->CR2
#define Spi_Config_CR2(x)           Spi_Cfg_CR2_Helper(x)

/* Sets the SSI bit of the Configuration Register 1, of the given SPI 
*  peripheral, specified by input 'x'.
*/
#define Spi_SetSSI_Helper(x)        SPI##x->CR1 |= SPI_CR1_SSI
#define Spi_SetSSI(x)               Spi_SetSSI_Helper(x)

/* Sets the SPE bit of the Configuration Register 1, of the given SPI 
*  peripheral, specified by input 'x'. This will enable the SPI Peripheral.
*/
#define Spi_PeripheralEnable_Helper(x)      SPI##x->CR1 |= SPI_CR1_SPE
#define Spi_PeripheralEnable(x)             Spi_PeripheralEnable_Helper(x)
/******************************************************************************
End Of File
******************************************************************************/

#endif /* SPI_H_ */
