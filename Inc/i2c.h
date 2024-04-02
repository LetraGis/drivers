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

/* Maximum APB1 Frequency - 45 MHz. I2C's are connected to APB1 Bus.
Minimum is 2 MHz, maximum is 45 MHz and intrinsic maximum is 50 MHz. */
#define I2C_PERICLKFREQ_45MHZ   (45u)

#define I2C_STATIC_CLKFREQ  TRUE

/* Spi_GetFlagStatus Return values */
#define I2C_FLAG_NOT_SET    0u
#define I2C_FLAG_SET        1u

#define I2C_TIM_SCL_RISE       (1000u)
#define I2C_TIM_SCL_CLK_HIGH   (4000u)
#define I2C_TIM_HIGH           (I2C_TIM_SCL_RISE + I2C_TIM_SCL_CLK_HIGH)

/******************************************************************************
DECLARATION OF TYPES
******************************************************************************/

/***********************  I2C CR1 Related Definitions  ***********************/
typedef uint8_t i2c_peripheralClkFreq;

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

/***********************  I2C OAR1 Related Definitions  **********************/
typedef enum
{
    i2c_7bitAddr = 0,   /* Disables i2c peripheral */
    i2c_10bitAddr       /* Enables i2c peripheral */
} i2c_slaveAddrMode;

/***********************  I2C DR Related Definitions  ************************/
typedef uint8_t i2c_data;

/*********************** I2C SR1 Related Definitions  ************************/
typedef enum
{
    SB_flag = I2C_SR1_SB,               /* Start bit. Set when start cond. */
    ADDR_flag = I2C_SR1_ADDR,           /* Addr. sent (mtr)/matched (slv) */        
    BTF_flag = I2C_SR1_BTF,             /* Byte transfer finished */
    ADD10_flag = I2C_SR1_ADD10,         /*  */
    STOPF_flag = I2C_SR1_STOPF,         /*  */
    RxNE_flag = I2C_SR1_RXNE,           /*  */
    TxE_flag = I2C_SR1_TXE,             /*  */
    BERR_flag = I2C_SR1_BERR,           /*  */
    ARLO_flag = I2C_SR1_ARLO,           /*  */
    AF_flag = I2C_SR1_AF,               /*  */
    OVR_flag = I2C_SR1_OVR,             /*  */
    PECERR_flag = I2C_SR1_PECERR,       /*  */
    TIMEOUT_flag = I2C_SR1_TIMEOUT,     /*  */
    SMBALERT_flag = I2C_SR1_SMBALERT    /*  */
} i2c_statusFlags1;

/*********************** I2C SR2 Related Definitions  ************************/
typedef enum
{
    MSL_flag = I2C_SR2_MSL,
    BUSY_flag = I2C_SR2_BUSY,
    TRA_flag = I2C_SR2_TRA,
    GENCALL_flag = I2C_SR2_GENCALL,
    SMBDEFAULT_flag = I2C_SR2_SMBDEFAULT,
    SMBHOST_flag = I2C_SR2_SMBHOST,
    DUALF_flag = I2C_SR2_DUALF,
} i2c_statusFlags2;

/*********************** I2C CCR Related Definitions  ************************/
typedef enum
{
    i2c_standardMode = 0,
    i2c_fastMode
} i2c_opMode;

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
/* TODO: add header description for API's */
/*!****************************************************************************
 * @brief			
 * @details		   	
 * @param[in]      	
 * @return         	
 ******************************************************************************/
__STATIC_INLINE void i2c_PeripheralInit(i2c_peripheralNum peripheral)
{
    RCC->APB1ENR |= (RCC_APB1ENR_I2C1EN_Msk + peripheral);
}

/*!****************************************************************************
 * @brief			
 * @details		   	
 * @param[in]      	
 * @return         	
 ******************************************************************************/
__STATIC_INLINE void i2c_StartCondition(i2c_peripheralNum peripheral)
{
    I2CX_CR1(peripheral) |= I2C_CR1_START;
}

/*!****************************************************************************
 * @brief			
 * @details		   	
 * @param[in]      	
 * @return         	
 ******************************************************************************/
__STATIC_INLINE void i2c_StopCondition(i2c_peripheralNum peripheral)
{
    I2CX_CR1(peripheral) |= I2C_CR1_STOP;
}

/*!****************************************************************************
 * @brief			
 * @details		   	
 * @param[in]      	
 * @return         	
 ******************************************************************************/
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

/*!****************************************************************************
 * @brief			
 * @details		   	
 * @param[in]      	
 * @return         	
 ******************************************************************************/
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

/*!****************************************************************************
 * @brief			
 * @details		   	
 * @param[in]      	
 * @return         	
 ******************************************************************************/
__STATIC_INLINE void i2c_PeripheralRst(i2c_peripheralNum peripheral)
{
    I2CX_CR1(peripheral) |= I2C_CR1_SWRST;
    /* Right after peripheral, we need to clear the bit.
    TODO: check if necessary. */
    I2CX_CR1(peripheral) &= ~(uint32_t)(I2C_CR1_SWRST);
}

/*!****************************************************************************
 * @brief			
 * @details		   	
 * @param[in]      	
 * @return         	
 ******************************************************************************/
#if  (I2C_STATIC_CLKFREQ != TRUE)
__STATIC_INLINE void i2c_PeripheralClkFreq(i2c_peripheralNum peripheral, 
                                           i2c_peripheralClkFreq clkFreq)
{
    I2CX_CR2(peripheral) |= (clkFreq && I2C_CR2_FREQ);
}
#else
__STATIC_INLINE void i2c_PeripheralClkFreq(i2c_peripheralNum peripheral)
{
    I2CX_CR2(peripheral) |= (I2C_PERICLKFREQ_45MHZ & I2C_CR2_FREQ);
}
#endif

/*!****************************************************************************
 * @brief			
 * @details		   	
 * @param[in]      	
 * @return         	
 ******************************************************************************/
__STATIC_INLINE void i2c_AddrModeCfg(i2c_peripheralNum peripheral, i2c_slaveAddrMode mode)
{
    if(i2c_7bitAddr == mode)
    {
        I2CX_OAR1(peripheral) &= ~(uint32_t)(I2C_OAR1_ADDMODE);
    }
    else
    {
        I2CX_OAR1(peripheral) |= I2C_OAR1_ADDMODE;
    }
}

/*!****************************************************************************
 * @brief			
 * @details		   	
 * @param[in]      	
 * @return         	
 ******************************************************************************/
__STATIC_INLINE void i2c_WriteData(i2c_peripheralNum peripheral, i2c_data data)
{
    I2CX_DR(peripheral) = data;
}

/*!****************************************************************************
 * @brief			
 * @details		   	
 * @param[in]      	
 * @return         	
 ******************************************************************************/
__STATIC_INLINE i2c_data i2c_ReadData(i2c_peripheralNum peripheral)
{
    return I2CX_DR(peripheral);
}

/*!****************************************************************************
 * @brief			
 * @details		   	
 * @param[in]      	
 * @return         	
 ******************************************************************************/
__STATIC_INLINE void i2c_ReadDataRef(i2c_peripheralNum peripheral, 
                                     i2c_data *data)
{
    *data = I2CX_DR(peripheral);
}

/*!****************************************************************************
 * @brief			
 * @details		   	
 * @param[in]      	
 * @return         	
 ******************************************************************************/
__STATIC_INLINE uint8_t i2c_GetStatusFlag_SR1(i2c_peripheralNum peripheral, 
                                          i2c_statusFlags1 flag)
{
    uint8_t flagStatus = I2C_FLAG_NOT_SET;
    if(flag == (I2CX_SR1(peripheral) & flag))
    {
        flagStatus = I2C_FLAG_SET;
    }
    return flagStatus;
}

/*!****************************************************************************
 * @brief			
 * @details		   	
 * @param[in]      	
 * @return         	
 ******************************************************************************/
__STATIC_INLINE uint8_t i2c_GetStatusFlag_SR2(i2c_peripheralNum peripheral, 
                                          i2c_statusFlags2 flag)
{
    uint8_t flagStatus = I2C_FLAG_NOT_SET;
    if(flag == (I2CX_SR1(peripheral) & flag))
    {
        flagStatus = I2C_FLAG_SET;
    }
    return flagStatus;
}

/*!****************************************************************************
 * @brief			
 * @details		   	
 * @param[in]      	
 * @return         	
 ******************************************************************************/
__STATIC_INLINE void i2c_OpModeCfg(i2c_peripheralNum peripheral, 
                                   i2c_opMode mode)
{
    if(i2c_standardMode == mode)
    {
        I2CX_CCR(peripheral) &= ~(uint32_t)(I2C_CCR_FS);
    }
    else
    {
        I2CX_CCR(peripheral) |= I2C_CCR_FS;
    }
}

/*!****************************************************************************
 * @brief			
 * @details		   	
 * @param[in]      	
 * @return         	
 ******************************************************************************/
__STATIC_INLINE void i2c_ClockCfg(i2c_peripheralNum peripheral, i2c_opMode mode)
{
    uint16_t ccr_value;

    /* TODO: to be implemented */
    I2CX_CCR(peripheral) |= (I2C_CCR_CCR & ccr_value);
}

/* TODO: Add API's to configure all fields on a register at once, using a mask
and macros, as follows:

--- This should be defined in configuration file (i2c_cfg.h file) ---
#define I2C_REG_MASK    ((BIT_0 << BIT_0_POS) |   \
                         (BIT_1 << BIT_1_POS) |    \ 
                         (BIT_2 << BIT_2_POS))

--- This can be defined in application level code ---
uint16_t i2c_reg_mask = I2C_REG_MASK;
I2C_CfgReg(i2c_reg_mask);

void I2C_CfgReg(uint16_t mask)
{
    I2C_Reg = mask;
}

end of TODO  */

/******************************************************************************
End Of File
******************************************************************************/

#endif /* I2C_H_ */
