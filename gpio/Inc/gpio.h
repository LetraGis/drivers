/*
 * gpio.h
 *
 *  Created on: Jun 3, 2022
 *      Author: LARL
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
#define TRUE (1u)
#define FALSE (0u)

/******************************************************************************
DECLARATION OF TYPES
******************************************************************************/

typedef uint32_t 	uint32;
typedef uint16_t 	uint16;
typedef uint8_t 	uint8;
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
	input = 0,
	output,
	alternate,
	analog,
} portMode;

typedef struct
{
	uint32 GPIOAEN 		: 1;
	uint32 GPIOBEN 		: 1;
	uint32 GPIOCEN 		: 1;
	uint32 GPIODEN 		: 1;
	uint32 GPIOEEN 		: 1;
	uint32 GPIOFEN 		: 1;
	uint32 GPIOGEN 		: 1;
	uint32 GPIOHEN 		: 1;
	uint32 RESERVED1 	: 4;
	uint32 CRCEN 		: 1;
	uint32 RESERVED2 	: 5;
	uint32 BKPSRAMEN 	: 1;
	uint32 RESERVED3 	: 2;
	uint32 DMA1EN 		: 1;
	uint32 DMA2EN 		: 1;
	uint32 RESERVED4 	: 6;
	uint32 OTGHSEN 		: 1;
	uint32 OTGHSULPIEN 	: 1;
	uint32 RESERVED5 	: 1;
} RCC_AHB1ENR_Type;

typedef struct
{
	uint16 pin0 	: 1;
	uint16 pin1 	: 1;
	uint16 pin2 	: 1;
	uint16 pin3 	: 1;
	uint16 pin4 	: 1;
	uint16 pin5 	: 1;
	uint16 pin6 	: 1;
	uint16 pin7 	: 1;
	uint16 pin8 	: 1;
	uint16 pin9 	: 1;
	uint16 pin10 	: 1;
	uint16 pin11 	: 1;
	uint16 pin12 	: 1;
	uint16 pin13 	: 1;
	uint16 pin14 	: 1;
	uint16 pin15 	: 1;
} GPIOX_ODR_Type;

typedef struct
{
	uint16 pin0 	: 1;
	uint16 pin1 	: 1;
	uint16 pin2 	: 1;
	uint16 pin3 	: 1;
	uint16 pin4 	: 1;
	uint16 pin5 	: 1;
	uint16 pin6 	: 1;
	uint16 pin7 	: 1;
	uint16 pin8 	: 1;
	uint16 pin9 	: 1;
	uint16 pin10 	: 1;
	uint16 pin11 	: 1;
	uint16 pin12 	: 1;
	uint16 pin13 	: 1;
	uint16 pin14 	: 1;
	uint16 pin15 	: 1;
} GPIOX_IDR_Type;

typedef struct
{
	uint32 pin0 	: 2;
	uint32 pin1 	: 2;
	uint32 pin2 	: 2;
	uint32 pin3 	: 2;
	uint32 pin4 	: 2;
	uint32 pin5 	: 2;
	uint32 pin6 	: 2;
	uint32 pin7 	: 2;
	uint32 pin8 	: 2;
	uint32 pin9 	: 2;
	uint32 pin10 	: 2;
	uint32 pin11 	: 2;
	uint32 pin12 	: 2;
	uint32 pin13 	: 2;
	uint32 pin14 	: 2;
	uint32 pin15 	: 2;
} GPIOX_MODER_Type;

typedef struct
{
	uint32 pin0 	: 2;
	uint32 pin1 	: 2;
	uint32 pin2 	: 2;
	uint32 pin3 	: 2;
	uint32 pin4 	: 2;
	uint32 pin5 	: 2;
	uint32 pin6 	: 2;
	uint32 pin7 	: 2;
	uint32 pin8 	: 2;
	uint32 pin9 	: 2;
	uint32 pin10 	: 2;
	uint32 pin11 	: 2;
	uint32 pin12 	: 2;
	uint32 pin13 	: 2;
	uint32 pin14 	: 2;
	uint32 pin15 	: 2;
} GPIOX_PUPDR_Type;

extern volatile RCC_AHB1ENR_Type * const RCC_AHB1ENR;
extern volatile GPIOX_MODER_Type * const GPIOA_MODER;
extern volatile GPIOX_MODER_Type * const GPIOC_MODER;
extern volatile GPIOX_ODR_Type * const GPIOA_ODR;
extern volatile GPIOX_IDR_Type * const GPIOC_IDR;

/******************************************************************************
End Of File
******************************************************************************/

#endif /* GPIO_H_ */
