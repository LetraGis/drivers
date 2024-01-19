/*
 * gpio.h
 *
 *  Created on: Jun 1, 2022
 *      Author: LetraGis
 */

#ifndef GPIO_H_
#define GPIO_H_

/******************************************************************************
EXTERNAL DEPENDENCIES
******************************************************************************/
#include <stdint.h>
#include "stm32f446xx.h"

/******************************************************************************
DEFINITION OF CONSTANTS
******************************************************************************/
/* GPIO base register address */
#define GPIOX_BASE 			(GPIOA_BASE)

/* GPIO offset from one GPIO port to another */
#define GPIOX_OFFSET		(0x400u)
/* GPIO MODER offset */
#define GPIO_MODER_OFFSET	(0x00u)
/* GPIO OTYPER offset */
#define GPIO_OTYPER_OFFSET	(0x04u)
/* GPIO OSPEEDR offset */
#define GPIO_OSPEEDR_OFFSET	(0x08u)
/* GPIO PUPDR offset */
#define GPIO_PUPDR_OFFSET	(0x0Cu)
/* GPIO IDR offset */
#define GPIO_IDR_OFFSET		(0x10u)
/* GPIO ODR offset */
#define GPIO_ODR_OFFSET		(0x14u)
/* GPIO BSRR offset */
#define GPIO_BSRR_OFFSET	(0x18u)
/* GPIO LCK offset */
#define GPIO_LCKR_OFFSET	(0x1Cu)
/* GPIO AFRL offset */
#define GPIO_AFRL_OFFSET	(0x20u)
/* GPIO AFRH offset */
#define GPIO_AFRH_OFFSET	(0x24u)

/* General defines */
#define GPIO_HIGH 		(1u)
#define GPIO_LOW 		(0u)
#define OK 				(0u)
#define N_OK 			(1u)
#define RET_ERROR 		(255u)

/* Maximum number of ports */
#define GPIO_PORT_NUM_MAX 		(8u)
/* Maximum number of pins per port */
#define GPIO_PIN_NUM_MAX 		(16u)
/* Maximum number of pull up/down configurations */
#define GPIO_PULL_MODE_MAX 		(3u)
/* Maximum number of pin modes */
#define GPIO_PIN_MODE_MAX 		(4u)
/* Maximum number of output type modes */
#define GPIO_OTYPE_MODE_MAX		(2u)
/* Maximum number of output speed modes */
#define GPIO_OSPEED_MODE_MAX 	(4u)
/* Maximum number of internal resistor modes */
#define GPIO_RES_MODE_MAX		(3u)
/* Maximum number of pin states */
#define GPIO_PIN_STATE_MAX		(2u)
/* Bit Reset Offset on Bit Set/Reset Register */
#define GPIO_BSRR_BR_PIN_OFFSET	(16u)
/* Bit Reset Offset on Bit Set/Reset Register */
#define GPIO_LCK_KEY			(16u)
/* Maximum number of alternate function configurations */
#define GPIO_ALT_FNC_MAX		(16u)
/* Number of bits used for alternate function:
	(2^4 = 16 possible combinations) */
#define GPIO_ALT_FIELD_LENGTH	(4u)
/* Mask for reading lower nibble */
#define GPIO_ALT_FIELD_MASK		(0x0Fu)

/******************************************************************************
DECLARATION OF TYPES
******************************************************************************/

typedef uint16_t lockMask;
typedef enum
{
	porta = 0,
	portb,
	portc,
	portd,
	porte,
	portf,
	portg,
	porth,
} portNumber;

typedef enum
{
	pin0 = 0,
	pin1,
	pin2,
	pin3,
	pin4,
	pin5,
	pin6,
	pin7,
	pin8,
	pin9,
	pin10,
	pin11,
	pin12,
	pin13,
	pin14,
	pin15,
} pinNumber;

typedef enum
{
	input = 0,
	output,
	alternate,
	analog,
} portMode;

typedef enum
{
	pushPull = 0,
	openDrain,
} outputType;

typedef enum
{
	lowSpeed = 0,
	mediumSpeed,
	fastSpeed,
	highSpeed,
} outputSpeed;

typedef enum
{
	none = 0,
	pullUp,
	pullDown,
} pullMode;

typedef enum
{
	low 	= 0,
	high 	= 1,
} pinState;

typedef enum
{
	altFun0 = 0,
	altFun1,
	altFun2,
	altFun3,
	altFun4,
	altFun5,
	altFun6,
	altFun7,
	altFun8,
	altFun9,
	altFun10,
	altFun11,
	altFun12,
	altFun13,
	altFun14,
	altFun15,
} altFunction;

/******************************************************************************
DECLARATION OF VARIABLES
******************************************************************************/

/******************************************************************************
DECLARATION OF CONSTANT DATA
******************************************************************************/

/******************************************************************************
DECLARATION OF FUNCTIONS
******************************************************************************/

void Gpio_Init(void);
void Gpio_InitCallback(void); /* Needs to be defined by the user to initialize
								 GPIO ports, set pin modes, etc. */
uint8_t Gpio_PortInit(portNumber port);
uint8_t Gpio_PortDeinit(portNumber port);
uint8_t Gpio_PinMode(portNumber port, pinNumber pin, portMode mode);
uint8_t Gpio_OutputType(portNumber port, pinNumber pin, outputType type);
uint8_t Gpio_OutputSpeed(portNumber port, pinNumber pin, outputSpeed speed);
uint8_t Gpio_PullMode(portNumber port, pinNumber pin, pullMode mode);
uint8_t Gpio_GetPinStateRef(portNumber port, pinNumber pin, pinState *state);
pinState Gpio_GetPinStateVal(portNumber port, pinNumber pin);
uint8_t Gpio_SetPinState(portNumber port, pinNumber pin, pinState state);
uint8_t Gpio_TogglePinState(portNumber port, pinNumber pin);
uint8_t Gpio_SetPinStateAtomic(portNumber port, pinNumber pin, pinState state);
uint8_t Gpio_LockCfg(portNumber port, lockMask mask);
uint8_t Gpio_ConfigAltFnc(portNumber port, pinNumber pin, altFunction func);

/******************************************************************************
DECLARATION OF FUNCTION-LIKE MACROS
******************************************************************************/
/*!****************************************************************************
 * @brief			Macro to access all GPIO registers based on offsets.
 * @details		   	Using this macro we can easily access GPIO registers based
 * 					on the provided port and offset between registers.
 ******************************************************************************/		
#define GPIOX_REG(port, offset)		*((volatile uint32_t *const)(GPIOX_BASE \
						   			+ (port * GPIOX_OFFSET) + offset))

/* Mode Register, configures pin as: in/out/alternate/analog */
#define GPIOX_MODER(port) 		GPIOX_REG(port, GPIO_MODER_OFFSET)
/* Output Type Register, configures output pin as: push-pull/open-drain */
#define GPIOX_OTYPER(port)		GPIOX_REG(port, GPIO_OTYPER_OFFSET)
/* Output Speed Register, configures pin speed as: low, med, fast, highspeed */
#define GPIOX_OSPEEDR(port)		GPIOX_REG(port, GPIO_OSPEEDR_OFFSET)
/* Pull-up Pull-down Register, configures pin internal resistors */
#define GPIOX_PUPDR(port)		GPIOX_REG(port, GPIO_PUPDR_OFFSET)
/* Input Data Register, reads from port */
#define GPIOX_IDR(port)			GPIOX_REG(port, GPIO_IDR_OFFSET)
/* Output Data Register, writes to port */
#define GPIOX_ODR(port)			GPIOX_REG(port, GPIO_ODR_OFFSET)
/* Bit Set Reset Register, for atomic bit set/reset */
#define GPIOX_BSRR(port)		GPIOX_REG(port, GPIO_BSRR_OFFSET)
/* Configuration Lock Register, for locking GPIO configuration */
#define GPIOX_LCKR(port)		GPIOX_REG(port, GPIO_LCKR_OFFSET)
/* Alternate Function Register Low, configures alt function, pin1-pin7 */
#define GPIOX_AFRL(port)		GPIOX_REG(port, GPIO_AFRL_OFFSET)
/* Alternate Function Register High, configures alt function, pin8-pin15 */
#define GPIOX_AFRH(port)		GPIOX_REG(port, GPIO_AFRH_OFFSET)

/******************************************************************************
End Of File
******************************************************************************/

#endif /* GPIO_H_ */
