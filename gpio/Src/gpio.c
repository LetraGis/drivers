/*
 * gpio.c
 *
 *  Created on: Jun 1, 2022
 *      Author: LetraGis
 */

/******************************************************************************
EXTERNAL DEPENDENCIES
******************************************************************************/

#include "gpio.h"
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
 * @brief			Calls callback function to perform Gpio initialization.
 * @details		   	Initializes Gpio by calling function Gpio_InitCallback. The
 * 					implementation needs to be designed by the developer.
 * @param[in]      	void    No input parameters.
 * @return         	void    No output parameters.
 ******************************************************************************/
void Gpio_Init(void)
{
    Gpio_InitCallback();
}

/*!****************************************************************************
 * @brief			Initialize port.
 * @details		   	Initializes the indicated port.
 * @param[in]      	port    Holds the port number.
 * @return         	OK      Request successful.
 *                 	N_OK    Request was not successful.
 ******************************************************************************/
uint8_t Gpio_PortInit(portNumber port)
{
	uint8_t retVal = N_OK;
	/* GPIO port enable */
	if(port < PORT_NUM_MAX)
	{
		RCC_AHB1ENR |= (1u << port);
		retVal = OK;
	}
	return (retVal);
}

/*!****************************************************************************
 * @brief			Configure port mode.
 * @details		   	Sets the port mode as output, input, alternate or analog.
 * @param[in]      	port    Holds the port number.
 * @param[in]      	pin     Holds the pin number.
 * @param[in]      	mode    Holds the mode: out/in/alternate/analog.
 * @return         	OK      Request successful.
 *                 	N_OK    Request was not successful.
 ******************************************************************************/
uint8_t Gpio_PinMode(portNumber port, pinNumber pin, portMode mode)
{
	uint8_t retVal = N_OK;
	if((port < PORT_NUM_MAX) && (mode < PIN_MODE_MAX ) && (pin < PIN_NUM_MAX))
	{
		/* Clear bits before setting them to mode */
		GPIOX_MODER(port) &= 
			(~(uint32_t)(1u << (pin * 2u)) | ~(uint32_t)(1u << ((pin * 2u) + 1u)));
		/* Setting pin to mode */
		GPIOX_MODER(port) |= ((uint32_t)(mode << (pin * 2)));
		retVal = OK;
	}
	return (retVal);
}

/*!****************************************************************************
 * @brief			Set pin state.
 * @details		   	Sets the given pin of the given port to the given state.
 * @param[in]      	port    Holds the port number.
 * @param[in]      	pin	    Holds the pin number.
 * @param[in]      	state   Holds the state to be set on the pin: HIGH/LOW.
 * @return         	OK      Request successful.
 *                 	N_OK    Request was not successful.
 ******************************************************************************/
uint8_t Gpio_SetPinState(portNumber port, pinNumber pin, pinState state)
{
	uint8_t retVal = N_OK;
	if((port < PORT_NUM_MAX) && (pin < PIN_NUM_MAX))
	{
		if(state == high)
		{
			GPIOX_ODR(port) |= (uint32_t)(1u << pin);
		}
		else if(state == low)
		{
			GPIOX_ODR(port) &= ~(uint32_t)(1u << pin);
		}
		else
		{
			/* Should not reach */
		}
		retVal = OK;
	}
	return (retVal);
}

/*!****************************************************************************
 * @brief			Toggle pin state.
 * @details		   	Toggles the status of the given pin of the given port.
 * @param[in]      	port    Holds the port number.
 * @param[in]      	pin     Holds the pin number.
 * @return         	OK      Request successful.
 *                 	N_OK    Request was not successful.
 ******************************************************************************/
uint8_t Gpio_TogglePinState(portNumber port, pinNumber pin)
{
	uint8_t retVal = N_OK;
	if((port < PORT_NUM_MAX) && (pin < PIN_NUM_MAX))
	{
		GPIOX_ODR(port) ^= (uint32_t)(1u << pin);
		retVal = OK;
	}
	return (retVal);
}

/*!****************************************************************************
 * @brief			Get pin state by reference.
 * @details			Gets the state by reference of given pin and given port.
 * @param[in]      	port    Holds the port number.
 * @param[in]      	pin     Holds the pin number.
 * @param[out]     	state   Returns the state of the given pin.
 * @return         	OK      Request successful.
 *                 	N_OK    Request was not successful.
 ******************************************************************************/
uint8_t Gpio_GetPinStateRef(portNumber port, pinNumber pin, pinState *state)
{
	uint8_t retVal = N_OK;
	if((port < PORT_NUM_MAX) && (pin < PIN_NUM_MAX) && (state != NULL))
	{
		*state = ((GPIOX_IDR(port))>>pin)&1u;
		retVal = OK;
	}
	return (retVal);
}

/*!****************************************************************************
 * @brief			Get pin state by value.
 * @details			Gets the state by value of given pin and given port.
 * @param[in]      	port    Holds the port number.
 * @param[in]      	pin     Holds the pin number.
 * @param[out]     	state   Returns the state of the given pin.
 * @return         	retVal  Pin state (HIGH/LOW).
 ******************************************************************************/
uint8_t Gpio_GetPinStateVal(portNumber port, pinNumber pin)
{
	pinState retVal = RET_ERROR;
	if((port < PORT_NUM_MAX) && (pin < PIN_NUM_MAX))
	{
		retVal = ((GPIOX_IDR(port))>>pin)&1u;
	}
	return (retVal);
}

/*!****************************************************************************
 * @brief			Set pull mode for pin resistors. 
 * @details		   	Sets the port mode as output, input, alternate or analog.
 * @param[in]      	port    Holds the port number.
 * @param[in]      	pin     Holds the pin number.
 * @param[in]      	mode    Holds the mode: out/in/alternate/analog.
 * @return         	OK      Request successful.
 *                 	N_OK    Request was not successful.
 ******************************************************************************/
uint8_t Gpio_PullMode(portNumber port, pinNumber pin, pullMode mode)
{
	uint8_t retVal = N_OK;
	if((port < PORT_NUM_MAX) && (mode < PIN_MODE_MAX ) && (pin < PIN_NUM_MAX))
	{
		/* Clear bits before setting them to mode */
		GPIOX_PUPDR(port) &= 
			(~(uint32_t)(1u << (pin * 2u)) | ~(uint32_t)(1u << ((pin * 2u) + 1u)));
		/* Setting pin to mode */
		GPIOX_PUPDR(port) |= ((uint32_t)(mode << (pin * 2)));
		retVal = OK;
	}
	return (retVal);
}

/******************************************************************************
End Of File
******************************************************************************/
