/*
 * i2c.h
 *
 *  Created on: March 25, 2024
 *      Author: LetraGis
 */

#ifndef I2C_H_
#define I2C_H_

/******************************************************************************
EXTERNAL DEPENDENCIES
******************************************************************************/
#include <stdint.h>
#include "stm32f446xx.h"

/******************************************************************************
DEFINITION OF CONSTANTS
******************************************************************************/
/* I2C base register address */
#define I2CX_BASE 			(I2C1_BASE)

/* I2C offset from one I2C peripheral to another */
#define I2CX_OFFSET		    (0x400u)
/* I2C CR1 offset */
#define I2C_CR1_OFFSET	    (0x00u)
/* I2C CR2 offset */
#define I2C_CR2_OFFSET	    (0x04u)
/* I2C OAR1 offset */
#define I2C_OAR1_OFFSET	    (0x08u)
/* I2C OAR2 offset */
#define I2C_OAR2_OFFSET	    (0x0Cu)
/* I2C DR offset */
#define I2C_DR_OFFSET       (0x10u)
/* I2C SR1 offset */
#define I2C_SR1_OFFSET	    (0x14u)
/* I2C SR2 offset */
#define I2C_SR2_OFFSET	    (0x18u)
/* I2C CCR offset */
#define I2C_CCR_OFFSET	    (0x1Cu)
/* I2C TRISE offset */
#define I2C_TRISE_OFFSET    (0x20u)
/* I2C FLTR offset */
#define I2C_FLTR_OFFSET	    (0x24u)

/******************************************************************************
DECLARATION OF TYPES
******************************************************************************/

/***********************  I2C CR1 Related Definitions  ***********************/
typedef enum
{
	i2c1 = 0,
    i2c2,
    i2c3
} i2c_peripheralNum;

typedef enum
{
    ack_disabled = 0,   /* Disables ACK */
    ack_enabled         /* Enables ACK */
} i2c_ackCtrl;

typedef enum
{
    i2c_disabled = 0,   /* Disables i2c peripheral */
    i2c_enabled         /* Enables i2c peripheral */
} i2c_peripheralCtrl;

/******************************************************************************
DECLARATION OF VARIABLES
******************************************************************************/

/******************************************************************************
DECLARATION OF CONSTANT DATA
******************************************************************************/

/******************************************************************************
DECLARATION OF FUNCTIONS
******************************************************************************/
void i2c_Init(void);
void i2c_MasterTx(void);
void i2c_MasterRx(void);
void i2c_SlaveTx(void);
void i2c_SlaveRx(void);

/******************************************************************************
DECLARATION OF FUNCTION-LIKE MACROS
******************************************************************************/
/*!****************************************************************************
 * @brief			Macro to access all i2c registers based on offsets.
 * @details		   	Using this macro we can easily access i2c registers based
 * 					on the provided i2cNum and offset between registers.
 ******************************************************************************/		
#define I2CX_REG(i2cNum, offset)		*((volatile uint32_t *const)(I2CX_BASE \
						   			+ (i2cNum * I2CX_OFFSET) + offset))

/* Control Register 1, enables peripheral, generates Start Condition, etc. */
#define I2CX_CR1(i2cNum)        I2CX_REG(i2cNum, I2C_CR1_OFFSET)
/* Control Register 2, configures input clock */
#define I2CX_CR2(i2cNum)	    I2CX_REG(i2cNum, I2C_CR2_OFFSET)
/* Own Address Register 1, holds i2c's device address */
#define I2CX_OAR1(i2cNum)	    I2CX_REG(i2cNum, I2C_OAR1_OFFSET)
/* Own Address Register 2, holds i2c's secondary device address (if ENDUAL=1) */
#define I2CX_OAR2(i2cNum)	    I2CX_REG(i2cNum, I2C_OAR2_OFFSET)
/* Input Data Register, reads from i2cNum */
#define I2CX_DR(i2cNum)		    I2CX_REG(i2cNum, I2C_DR_OFFSET)
/* Output Data Register, writes to i2cNum */
#define I2CX_SR1(i2cNum)	    I2CX_REG(i2cNum, I2C_SR1_OFFSET)
/* Bit Set Reset Register, for atomic bit set/reset */
#define I2CX_SR2(i2cNum)	    I2CX_REG(i2cNum, I2C_SR2_OFFSET)
/* Clock Control Register, for SCL master clock generation */
#define I2CX_CCR(i2cNum)	    I2CX_REG(i2cNum, I2C_CCR_OFFSET)
/* Alternate Function Register Low, configures alt function, pin1-pin7 */
#define I2CX_TRISE(i2cNum)	    I2CX_REG(i2cNum, I2C_TRISE_OFFSET)
/* Alternate Function Register High, configures alt function, pin8-pin15 */
#define I2CX_FLTR(i2cNum)	    I2CX_REG(i2cNum, I2C_FLTR_OFFSET)

/******************************************************************************
DECLARATION OF INLINE FUNCTIONS
******************************************************************************/
__STATIC_INLINE void i2c_PeripheralInit(i2c_peripheralNum peripheral)
{
    RCC->APB1ENR |= (RCC_APB1ENR_I2C1EN_Msk + peripheral);
}

__STATIC_INLINE void i2c_StartCondition(i2c_peripheralNum peripheral)
{
    I2CX_CR1(peripheral) |= I2C_CR1_START;
}

__STATIC_INLINE void i2c_StopCondition(i2c_peripheralNum peripheral)
{
    I2CX_CR1(peripheral) |= I2C_CR1_STOP;
}

__STATIC_INLINE void i2c_AckCtrl(i2c_peripheralNum peripheral, 
                                 i2c_ackCtrl ackCtrl)
{
    if(ack_disabled == ackCtrl)
    {
        I2CX_CR1(peripheral) &= ~(uint32_t)(I2C_CR1_ACK);
    }
    else
    {
        I2CX_CR1(peripheral) |= I2C_CR1_ACK;
    }
}

__STATIC_INLINE void i2c_PeripheralCtrl(i2c_peripheralNum peripheral, 
                                        i2c_peripheralCtrl peripheralCtrl)
{
    if(i2c_disabled == peripheralCtrl)
    {
        I2CX_CR1(peripheral) &= ~(uint32_t)(I2C_CR1_PE);
    }
    else
    {
        I2CX_CR1(peripheral) |= I2C_CR1_PE;
    }
}

__STATIC_INLINE void i2c_PeripheralRst(i2c_peripheralNum peripheral)
{
    I2CX_CR1(peripheral) |= I2C_CR1_SWRST;
    /* Right after peripheral, we need to clear the bit.
    TODO: check if necessary. */
    I2CX_CR1(peripheral) &= ~(uint32_t)(I2C_CR1_SWRST);
}



/******************************************************************************
End Of File
******************************************************************************/

#endif /* I2C_H_ */
