/*
 * gpio.c
 *
 *  Created on: Jun 1, 2022
 *      Author: LetraGis
 */

#include "gpio.h"
#include "stddef.h"

/*!****************************************************************************
 * @details		   	Initializes the indicated port.
 * @param[in]      	port    Holds the port number.
 * @return         	retVal
 *                 	OK:     port has been enabled
 *                 	N_OK:   port has not been enabled due to invalid input
 ******************************************************************************/
uint8 Gpio_PortInit(portNumber port)
{
	uint8 retVal = N_OK;
	/* GPIO port enable */
	if(port < PORT_MAX)
	{
		RCC_AHB1ENR |= (1u << port);
		retVal = OK;
	}
	return retVal;
}

/*!****************************************************************************
 * @details		   	Sets the port mode as output, input, alternate or analog.
 * @param[in]      	port    Holds the port number.
 * @param[in]      	pin     Holds the pin number.
 * @param[in]      	mode    Holds the mode: out/in/alternate/analog
 * @return         	retVal
 *                 	OK:     request successful
 *                 	N_OK:   Request was not successful
 ******************************************************************************/
uint8 Gpio_PinMode(portNumber port, pinNumber pin, portMode mode)
{
	uint8 retVal = N_OK;
	if((port < PORT_MAX) && (mode < PIN_MODE_MAX ) && (pin < PIN_NUM_MAX))
	{
		/* Clear bits before setting them to mode */
		GPIOX_MODER(port) &= (~(uint32)(1u << (pin * 2u)) | ~(uint32)(1u << ((pin * 2u) + 1u)));
		/* Setting pin to mode */
		GPIOX_MODER(port) |= ((uint32)(mode << (pin * 2)));
		retVal = OK;
	}
	return retVal;
}

/*!****************************************************************************
 * @details		   	Sets the given pin of the given port to the state given.
 * @param[in]      	port    Holds the port number.
 * @param[in]      	pin	    Holds the pin number.
 * @param[in]      	state   Holds the state to be set on the pin: HIGH/LOW
 * @return         	retVal
 *                 	OK:     request successful
 *                 	N_OK:   Request was not successful
 ******************************************************************************/
uint8 Gpio_SetPinState(portNumber port, pinNumber pin, pinState state)
{
	uint8 retVal = N_OK;
	if((port < PORT_MAX) && (pin < PIN_NUM_MAX))
	{
		if(state == high)
		{
			GPIOX_ODR(port) |= (uint32)(1u << pin);
			retVal = OK;
		}
		else if(state == low)
		{
			GPIOX_ODR(port) &= ~(uint32)(1u << pin);
			retVal = OK;
		}
	}
	return retVal;
}

/*!****************************************************************************
 * @details		   	Initializes the indicated port.
 * @param[in]      	port    Holds the port number.
 * @param[in]      	pin     Holds the pin number.
 * @return         	retVal
 *                 	OK:     request successful
 *                 	N_OK:   Request was not successful
 ******************************************************************************/
uint8 Gpio_TogglePinState(portNumber port, pinNumber pin)
{
	uint8 retVal = N_OK;
	if((port < PORT_MAX) && (pin < PIN_NUM_MAX))
	{
		GPIOX_ODR(port) ^= (uint32)(1u << pin);
		retVal = OK;
	}
	return retVal;
}

/*!****************************************************************************
 * @details			Gets the status of the given pin of the given port.
 * @param[in]      	port    Holds the port number.
 * @param[in]      	pin     Holds the pin number.
 * @param[out]     	state   Returns the state of the given pin.
 * @return         	retVal
 *                 	OK:     request successful
 *                 	N_OK:   Request was not successful
 ******************************************************************************/
uint8 Gpio_GetPinStateRef(portNumber port, pinNumber pin, pinState *state)
{
	uint8 retVal = N_OK;
	if((port < PORT_MAX) && (pin < PIN_NUM_MAX) && (state != NULL))
	{
		*state = ((GPIOX_IDR(port))>>pin)&1u;
		retVal = OK;
	}
	return retVal;
}

/*!****************************************************************************
 * @details			Gets the status of the given pin of the given port.
 * @param[in]      	port    Holds the port number.
 * @param[in]      	pin     Holds the pin number.
 * @param[out]     	state   Returns the state of the given pin.
 * @return         	retVal
 *                 	OK:     request successful
 *                 	N_OK:   Request was not successful
 ******************************************************************************/
uint8 Gpio_GetPinStateVal(portNumber port, pinNumber pin)
{
	pinState retVal = RET_ERROR;
	if((port < PORT_MAX) && (pin < PIN_NUM_MAX))
	{
		retVal = ((GPIOX_IDR(port))>>pin)&1u;
	}
	return retVal;
}

/*!****************************************************************************
 * @details		   	Sets the port mode as output, input, alternate or analog.
 * @param[in]      	port    Holds the port number.
 * @param[in]      	pin     Holds the pin number.
 * @param[in]      	mode    Holds the mode: out/in/alternate/analog
 * @return         	retVal
 *                 	OK:     request successful
 *                 	N_OK:   Request was not successful
 ******************************************************************************/
uint8 Gpio_PullMode(portNumber port, pinNumber pin, pullMode mode)
{
	uint8 retVal = N_OK;
	if((port < PORT_MAX) && (mode < PIN_MODE_MAX ) && (pin < PIN_NUM_MAX))
	{
		/* Clear bits before setting them to mode */
		GPIOX_PUPDR(port) &= (~(uint32)(1u << (pin * 2u)) | ~(uint32)(1u << ((pin * 2u) + 1u)));
		/* Setting pin to mode */
		GPIOX_PUPDR(port) |= ((uint32)(mode << (pin * 2)));
		retVal = OK;
	}
	return retVal;
}
