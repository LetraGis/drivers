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
/* Number of SPI Peripherals. */
#define SPI_1    1  /* SPI1 Peripheral Number */
#define SPI_2    2  /* SPI2 Peripheral Number */
#define SPI_3    3  /* SPI3 Peripheral Number */
#define SPI_4    4  /* SPI4 Peripheral Number */

/* Number of elements to be substracted from length counter when transmitting
or receiving DFF's of two bytes of data over SPI. */
#define DFF_TWO_BYTES   2u

/* Spi_GetFlagStatus Return values */
#define SPI_FLAG_NOT_SET    0u
#define SPI_FLAG_SET        1u
/******************************************************************************
DECLARATION OF TYPES
******************************************************************************/

/***********************  SPI CR1 Related Definitions  ***********************/
typedef union Spi_Cfg1RegType_Tag
{
    uint16_t Bytes;
    struct
    {
        uint16_t CPHA       : 1;    /* Clock Phase. Sets clock capture edge */
        uint16_t CPOL       : 1;    /* Clock Polarity. */
        uint16_t MSTR       : 1;    /* Device Selection. 0 - slave, 1 master */
        uint16_t BR         : 3;    /* Baud Rate. SourceClock division ratio */
        uint16_t SPE        : 1;    /* SPI Enable. */
        uint16_t LSBFIRST   : 1;    /* Frame Format. 0-MSB First, 1-LSB First */
        uint16_t SSI        : 1;    /* Internal Slave Select. NSS Pin related */
        uint16_t SSM        : 1;    /* SW Slave Mgt. 0-enabled, 1-disabled */
        uint16_t RXONLY     : 1;    /* Receive Only Mode enabled. */
        uint16_t DFF        : 1;    /* Data Frame Format. 0-8 bits, 1-16 bits */
        uint16_t CRCNEXT    : 1;    /* CRC Transfer Next. 0-data, 1-CRC */
        uint16_t CRCEN      : 1;    /* HW-CRC calculation enabled */
        uint16_t BIDIOE     : 1;    /* Output Enable in bidirectional mode */
        uint16_t BIDIMODE   : 1;    /* Bidirectional data mode enabled. */
    } Fields;
} Spi_Cfg1RegType;

/* CPHA */
typedef enum
{
    firstEdge = 0,  /* Data Capture: First clock transition */
    secondEdge,     /* Data Capture: Second clock transition */
} Spi_ClockPhase;

/* CPOL */
typedef enum
{
    clkPolLowIdle = 0,  /* Clock to 0 when idle */
    clkPolHighIdle,     /* Clock to 1 when idle */
} Spi_ClockPolarity;

/* MSTR */
typedef enum
{
    spiSlave = 0,   /* Slave Configuration */
    spiMaster,      /* Master Configuration */
} Spi_DeviceMode;

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

/* SSM */
typedef enum
{
    swMgtDisabled = 0,  /* SW Slave Mgt disabled */
    swMgtEnabled,       /* SW Slave Mgt enabled */
} Spi_SoftwareSlaveMgt;

/* DFF */
typedef enum
{
    oneByte = 0,    /* 8-bit Frame */
    twoBytes,       /* 16-bit Frame */
} Spi_DataFormat;

/* BIDIOE */
typedef enum
{
    outDisabled = 0,    /* Output Disabled (receive-only mode) */
    outEnabled,         /* Output Enabled (transmit-only mode) */
} Spi_BidiOutputEnable;

/* BIDIMODE */
typedef enum
{
    twoLineUni = 0, /* Two-Line Unidirectional Mode */
    oneLineBi,      /* One-Line Bidirectional Mode */
} Spi_BidirectionalMode;

/***********************  SPI CR2 Related Definitions  ***********************/
typedef union Spi_Cfg2RegType_Tag
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
} Spi_Cfg2RegType;

/* TXEIE, RXNEIE, ERRIE */
typedef enum
{
    interruptDisabled = 0,  /* Will not fire an ISR request */
    interruptEnabled        /* Will fire an ISR request when Flag is set */
} Spi_InterruptState;

/* FRF */
typedef enum
{
    MotorolaMode = 0,   /* SPI Motorola Mode */
    TexasInstMode       /* SPI TI Mode */
} Spi_FrameFormat;

/* TXDMAEN, RXDMAEN */
typedef enum
{
    DMASupportDisable = 0,  /* Tx/Rx Buffer DMA disabled */
    DMASupportEnable        /* Tx/Rx Buffer DMA enabled */
} Spi_DMASupport;

/* Interrupt related type, used to enable or disable ERRIE, RXNE and TXE */
typedef enum
{
    ERR_Flag    = SPI_CR2_ERRIE,    /* Error Interrupt Enable */
    RXNE_Flag   = SPI_CR2_RXNEIE,   /* Rx-Buffer Not Empty Interrupt Enable */
    TXE_Flag    = SPI_CR2_TXEIE     /* Tx-Buffer Empty Interrupt Enable */  
} Spi_InterruptFlag;

/************************ SPI SR Related Definitions *************************/
/* Status Register Flags */
typedef enum
{
    RXNE_Flag   = SPI_SR_RXNE,      /* Receive buffer Not Empty */
    TXE_Flag    = SPI_SR_TXE,       /* Transmit buffer Empty */
    CHSIDE_Flag = SPI_SR_CHSIDE,    /* Channel side */
    UDR_Flag    = SPI_SR_UDR,       /* Underrun flag */
    CRCERR_Flag = SPI_SR_CRCERR,    /* CRC Error flag */
    MODF_Flag   = SPI_SR_MODF,      /* Mode fault */
    OVR_Flag    = SPI_SR_OVR,       /* Overrun flag */
    BSY_Flag    = SPI_SR_BSY,       /* Busy flag */
    FRE_Flag    = SPI_SR_FRE        /* Frame format error flag */
} Spi_StatusFlag;

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
uint16_t Spi_SendReceiveData(SPI_TypeDef *pSpiHandle, uint16_t TxBuff);

/*!****************************************************************************
 * @brief			Gets the status of a given flag from Status Register.
 * @details		   	Returns if either the flag is set or nor set, based on the
 *                  flag value given by the user.
 * @param[in]      	pSpiHandle  SPI Data Structure, holds Status Register.
 * @param[in]       flag        Flag to be tested. Possible values: use values
 *                              from Spi_StatusFlag type.
 * @return         	flagStatus  Flag is set or not set.
 ******************************************************************************/
__STATIC_INLINE uint8_t Spi_GetFlagStatus(SPI_TypeDef *pSpiHandle, Spi_StatusFlag flag)
{
    uint8_t flagStatus = SPI_FLAG_NOT_SET;
    if(flag == (pSpiHandle->SR & flag))
    {
        flagStatus = SPI_FLAG_SET;
    }
    return flagStatus;
}

/*!****************************************************************************
 * @brief			Enables Interrupt depending on input.
 * @details		   	Sets the interrupt based on the input given.
 * @param[in]      	pSpiHandle  SPI Data Structure, holds Control Register 2.
 * @param[in]       flag        Flag to be tested. Possible values: Use values
 *                              from Spi_InterruptFlag type.
 * @return         	flagStatus  Flag is set or not set.
 ******************************************************************************/
__STATIC_INLINE void Spi_SetInterruptFlag(SPI_TypeDef *pSpiHandle, Spi_InterruptFlag flag)
{
    pSpiHandle->CR2 |= flag;
}

/*!****************************************************************************
 * @brief			Disables Interrupt depending on input.
 * @details		   	Disables the interrupt based on the input given.
 * @param[in]      	pSpiHandle  SPI Data Structure, holds Status Register.
 * @param[in]       flag        Flag to be tested. Possible values. Use values
 *                              from Spi_InterruptFlag type.
 * @return         	flagStatus  Flag is set or not set.
 ******************************************************************************/
__STATIC_INLINE void Spi_ClearInterruptFlag(SPI_TypeDef *pSpiHandle, Spi_InterruptFlag flag)
{
    pSpiHandle->CR2 &= ~(uint32_t)(flag);
}

/*!****************************************************************************
 * @brief			Writes content to SPI's Data Register.
 * @details		   	Writes content to SPI's Data Register based of the input 
 *                  given by the user. Useful during Full-Duplex configuration.
 * @param[in]      	pSpiHandle  SPI Data Structure, holds Data Register (which
 *                              is connected to Tx Buffer).
 * @return         	void        No output parameters.
 ******************************************************************************/
__STATIC_INLINE void Spi_WriteDR(SPI_TypeDef *pSpiHandle, uint16_t dataRegVal)
{
    pSpiHandle->DR = dataRegVal;
}

/*!****************************************************************************
 * @brief			Reads content of Data Register.
 * @details		   	Reads content of Data Register. Useful during Full-Duplex
 *                  configuration.
 * @param[in]      	pSpiHandle  SPI Data Structure, holds Data Register (which
 *                              is connected to Rx Buffer).
 * @return         	dataRegVal  Value read from Data Register (Rx Buffer)
 ******************************************************************************/
__STATIC_INLINE uint16_t Spi_ReadDR(SPI_TypeDef *pSpiHandle)
{
    return (pSpiHandle->DR);
}

/*!****************************************************************************
 * @brief			Reads from SPI's Data Register.
 * @details		   	Reads data from SPI's Data Register once the RXNE flag is
 *                  set, which indicates that Rx Buffer is full and we can read 
 *                  the Data Register.
 * @param[in]      	pSpiHandle  SPI Data Structure, holds Data Register.
 * @return         	DataReg     Contents of the Data Register.
 ******************************************************************************/
__STATIC_INLINE uint16_t Spi_ReadData(SPI_TypeDef *pSpiHandle)
{
    while (SPI_FLAG_SET != (Spi_GetFlagStatus(pSpiHandle, RXNE_Flag)));
    return pSpiHandle->DR;
}

/*!****************************************************************************
 * @brief			Writes to SPI's Data Register.
 * @details		   	Writes data to SPI's Data Register and waits until TXE flag
 *                  is set, which indicates that Tx Buffer is empty and we can
 *                  contents have been transfered to Data Register.
 * @param[in]      	pSpiHandle  SPI Data Structure, holds Status Register.
 * @param[in]       flag        Flag to be tested. Possible values. Use values
 *                              from Spi_StatusFlag type.
 * @return         	flagStatus  Flag is set or not set.
 ******************************************************************************/
__STATIC_INLINE void Spi_WriteData(SPI_TypeDef *pSpiHandle, uint16_t data)
{
    while (SPI_FLAG_SET != (Spi_GetFlagStatus(pSpiHandle, TXE_Flag)));
    pSpiHandle->DR = data;
}
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

/* Sets the SSOE bit of the Configuration Register 2, of the given SPI
*  peripheral, specified by input 'x'. This enables SS Output.
*/
#define Spi_SetSSOE_Helper(x)       SPI##x->CR2 |= SPI_CR2_SSOE
#define Spi_SetSSOE(x)              Spi_SetSSOE_Helper(x)

/* Checks the BSY flag of the Status Register, of the given SPI peripheral, 
* specified by input 'x', which determines if SPI HW is busy.
*/
#define Spi_ReadBSYFlag_Helper(x)   ((SPI##x->SR >> SPI_SR_BSY_Pos) & 1u)
#define Spi_ReadBSYFlag(x)          Spi_ReadBSYFlag_Helper(x)

/* Sets the SPE bit of the Configuration Register 1, of the given SPI 
*  peripheral, specified by input 'x'. This will enable the SPI Peripheral.
*/
#define Spi_PeripheralEnable_Helper(x)      SPI##x->CR1 |= SPI_CR1_SPE
#define Spi_PeripheralEnable(x)             Spi_PeripheralEnable_Helper(x)

/* Clears the SPE bit of the Configuration Register 1, of the given SPI 
*  peripheral, specified by input 'x'. This will disable the SPI Peripheral.
*/
#define Spi_PeripheralDisable_Helper(x)     SPI##x->CR1 &= ~(uint32_t)(SPI_CR1_SPE)
#define Spi_PeripheralDisable(x)            Spi_PeripheralDisable_Helper(x)

/* LetraGis (TODO): following macros produce the same result as Write/ReadDR
API's. To evaluate which implementation suits better. */
/* Reads Data Register. Used for Full-Duplex dummy reads. */
#define Spi_ReadDataReg_Helper(x)           (SPI##x->DR)
#define Spi_ReadDataReg(x)                  Spi_ReadDataReg_Helper(x)

/* Writes Data Register. Used for Full-Duplex dummy writes. */
#define Spi_WriteDataReg_Helper(x, y)       (SPI##x->DR = y)
#define Spi_WriteDataReg(x, y)              Spi_WriteDataReg_Helper(x, y)
/******************************************************************************
End Of File
******************************************************************************/

#endif /* SPI_H_ */
