/*
 * gpio.c
 *
 *  Created on: Jun 3, 2022
 *      Author: LARL
 */

#include <gpio.h>

volatile RCC_AHB1ENR_Type * const RCC_AHB1ENR = (RCC_AHB1ENR_Type *)(RCC_AHB1ENR_ADDRESS);
volatile GPIOX_MODER_Type * const GPIOA_MODER = (GPIOX_MODER_Type *)(GPIOX_REG_BASE_ADDRESS + (porta * 0x400u));
volatile GPIOX_MODER_Type * const GPIOC_MODER = (GPIOX_MODER_Type *)(GPIOX_REG_BASE_ADDRESS + (portc * 0x400u));
volatile GPIOX_ODR_Type * const GPIOA_ODR = (GPIOX_ODR_Type *)(GPIOX_REG_BASE_ADDRESS + (porta * 0x400u) + 0x14u);
volatile GPIOX_IDR_Type * const GPIOC_IDR = (GPIOX_IDR_Type *)(GPIOX_REG_BASE_ADDRESS + (portc * 0x400u) + 0x10u);
