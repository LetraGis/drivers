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

/******************************************************************************
DEFINITION OF CONSTANTS
******************************************************************************/
/* RCC register to enable GPIO ports */
#define RCC_AHB1ENR_ADDRESS 0x40023830u
/* GPIO base register address */
#define GPIOX_REG_BASE_ADDRESS 0x40020000u

#define HIGH (1u)
#define LOW (0)
#define OK (0u)
#define N_OK (1u)
#define RET_ERROR (255u)

/* Maximum number of ports */
#define PORT_NUM_MAX (8u)
/* Maximum number of pin modes */
#define PIN_MODE_MAX (4u)
/* Maximum number of pins per port */
#define PIN_NUM_MAX (16u)
/* Maximum number of pull up/down configurations */
#define PULL_MODE_MAX (3u)

/******************************************************************************
DECLARATION OF TYPES
******************************************************************************/
typedef enum
{
	low = 0,
	high = 1,
} pinState;

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
	none = 0,
	pullup,
	pulldown,
} pullMode;

typedef enum
{
	input = 0,
	output,
	alternate,
	analog,
} portMode;

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
uint8_t Gpio_PinMode(portNumber port, pinNumber pin, portMode mode);
uint8_t Gpio_SetPinState(portNumber port, pinNumber pin, pinState state);
uint8_t Gpio_TogglePinState(portNumber port, pinNumber pin);
uint8_t Gpio_GetPinStateRef(portNumber port, pinNumber pin, pinState *state);
uint8_t Gpio_GetPinStateVal(portNumber port, pinNumber pin);
uint8_t Gpio_PullMode(portNumber port, pinNumber pin, pullMode mode);

/******************************************************************************
DECLARATION OF FUNCTION-LIKE MACROS
******************************************************************************/
#define RCC_AHB1ENR (*(volatile uint32_t *const)(RCC_AHB1ENR_ADDRESS))
#define GPIOX_MODER(port) (*(volatile uint32_t *const)(GPIOX_REG_BASE_ADDRESS \
						   + (port * 0x400u)))

#define GPIOX_PUPDR(port) (*(volatile uint32_t *const)(GPIOX_REG_BASE_ADDRESS \
						   + (port * 0x400u) + 0x0Cu))

#define GPIOX_IDR(port) (*(volatile uint32_t *const)(GPIOX_REG_BASE_ADDRESS \
						   + (port * 0x400u) + 0x10u))

#define GPIOX_ODR(port) (*(volatile uint32_t *const)(GPIOX_REG_BASE_ADDRESS \
						   + (port * 0x400u) + 0x14u))

/******************************************************************************
End Of File
******************************************************************************/

#endif /* GPIO_H_ */
