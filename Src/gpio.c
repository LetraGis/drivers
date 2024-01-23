/**
 ******************************************************************************
 * @file           : gpio.c
 * @author         : Letra Gis
 * @brief          : This files contains API's for using GPIO Peripheral. A
 * 					 brief description of each API capabilities can be found on
 * 					 in its header.
 ******************************************************************************
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
	if(port < GPIO_PORT_NUM_MAX)
	{
		RCC->AHB1ENR |= (1u << port);
		retVal = OK;
	}
	return (retVal);
}

/*!****************************************************************************
 * @brief			Resets all GPIO related registers to its reset state.
 * @details		   	By means of setting the correpsonding bit on the RCC AHB1
 * 					Reset Register, all GPIO registers will be resetted to its
 * 					initial values.
 * @param[in]      	port    Holds the port number
 * @return         	OK      Request successful.
 *                 	N_OK    Request was not successful.
 ******************************************************************************/
uint8_t Gpio_PortDeinit(portNumber port)
{
	uint8_t retVal = N_OK;
	/* GPIO port enable */
	if(port < GPIO_PORT_NUM_MAX)
	{
		RCC->AHB1RSTR |= (1u << port);
		/* Right after resetting GPIOX registers, we need to clear the bit. */
		RCC->AHB1RSTR &= (~(uint32_t)(1u << port));
		retVal = OK;
	}
	return (retVal);
}

/*!****************************************************************************
 * @brief			Configure pin mode.
 * @details		   	Sets the pin mode as output, input, alternate or analog.
 * @param[in]      	port    Holds the port number.
 * @param[in]      	pin     Holds the pin number.
 * @param[in]      	mode    Holds the mode: out/in/alternate/analog.
 * @return         	OK      Request successful.
 *                 	N_OK    Request was not successful.
 ******************************************************************************/
uint8_t Gpio_PinMode(portNumber port, pinNumber pin, portMode mode)
{
	uint8_t retVal = N_OK;
	if((port < GPIO_PORT_NUM_MAX) && (mode < GPIO_PIN_MODE_MAX ) && (pin < GPIO_PIN_NUM_MAX))
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
 * @brief			Set pin output configuration. 
 * @details		   	Sets the pin as push-pull or open-drain configuration. When
 * 					open-drain, pin states are either low or floating. When
 * 					push-pull, pin states are either low or high.
 * @param[in]      	port    Holds the port number.
 * @param[in]      	pin     Holds the pin number.
 * @param[in]      	type    Holds the output type: push-pull or open-drain.
 * @return         	OK      Request successful.
 *                 	N_OK    Request was not successful.
 ******************************************************************************/
uint8_t Gpio_OutputType(portNumber port, pinNumber pin, outputType type)
{
	uint8_t retVal = N_OK;
	if((port < GPIO_PORT_NUM_MAX) && (pin < GPIO_PIN_NUM_MAX) && (type < GPIO_OTYPE_MODE_MAX))
	{
		if(type == openDrain)
		{
			GPIOX_OTYPER(port) |= (uint32_t)(1u << pin);
			retVal = OK;
		}
		else if(type == pushPull)
		{
			GPIOX_OTYPER(port) &= ~(uint32_t)(1u << pin);
			retVal = OK;
		}
		else
		{
			/* Should not reach */
		}
	}
	return (retVal);
}

/*!****************************************************************************
 * @brief			Set pin output speed. 
 * @param[in]      	port    Holds the port number.
 * @param[in]      	pin     Holds the pin number.
 * @param[in]      	speed   Holds the speed: low, med, fast, high.
 * @return         	OK      Request successful.
 *                 	N_OK    Request was not successful.
 ******************************************************************************/
uint8_t Gpio_OutputSpeed(portNumber port, pinNumber pin, outputSpeed speed)
{
	uint8_t retVal = N_OK;
	if((port < GPIO_PORT_NUM_MAX) && (speed < GPIO_OSPEED_MODE_MAX) && (pin < GPIO_PIN_NUM_MAX))
	{
		/* Clear bits before setting them to speed */
		GPIOX_OSPEEDR(port) &= 
			(~(uint32_t)(1u << (pin * 2u)) | ~(uint32_t)(1u << ((pin * 2u) + 1u)));
		/* Setting pin to speed */
		GPIOX_OSPEEDR(port) |= ((uint32_t)(speed << (pin * 2)));
		retVal = OK;
	}
	return (retVal);
}

/*!****************************************************************************
 * @brief			Set pull mode for pin resistors. 
 * @details		   	Sets the pin resistors as none, pull-up or pull-down.
 * @param[in]      	port    Holds the port number.
 * @param[in]      	pin     Holds the pin number.
 * @param[in]      	mode    Holds the mode: none, pull-up or pull-down.
 * @return         	OK      Request successful.
 *                 	N_OK    Request was not successful.
 ******************************************************************************/
uint8_t Gpio_PullMode(portNumber port, pinNumber pin, pullMode mode)
{
	uint8_t retVal = N_OK;
	if((port < GPIO_PORT_NUM_MAX) && (mode < GPIO_PULL_MODE_MAX) && (pin < GPIO_PIN_NUM_MAX))
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
	if((port < GPIO_PORT_NUM_MAX) && (pin < GPIO_PIN_NUM_MAX) && (state != NULL))
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
 * @return         	retVal  Pin state (HIGH/LOW).
 ******************************************************************************/
pinState Gpio_GetPinStateVal(portNumber port, pinNumber pin)
{
	pinState retVal = RET_ERROR;
	if((port < GPIO_PORT_NUM_MAX) && (pin < GPIO_PIN_NUM_MAX))
	{
		retVal = ((GPIOX_IDR(port))>>pin)&1u;
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
	if((port < GPIO_PORT_NUM_MAX) && (pin < GPIO_PIN_NUM_MAX) && (state < GPIO_PIN_STATE_MAX))
	{
		if(state == high)
		{
			GPIOX_ODR(port) |= (uint32_t)(1u << pin);
			retVal = OK;
		}
		else if(state == low)
		{
			GPIOX_ODR(port) &= ~(uint32_t)(1u << pin);
			retVal = OK;
		}
		else
		{
			/* Should not reach */
		}
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
	if((port < GPIO_PORT_NUM_MAX) && (pin < GPIO_PIN_NUM_MAX))
	{
		GPIOX_ODR(port) ^= (uint32_t)(1u << pin);
		retVal = OK;
	}
	return (retVal);
}

/*!****************************************************************************
 * @brief			Set pin state using atomic operation (BSRR Register).
 * @details		   	Sets the given pin of the given port to the given state.
 * 					This is done atomically.
 * @param[in]      	port    Holds the port number.
 * @param[in]      	pin	    Holds the pin number.
 * @param[in]      	state   Holds the state to be set on the pin: HIGH/LOW.
 * @return         	OK      Request successful.
 *                 	N_OK    Request was not successful.
 ******************************************************************************/
uint8_t Gpio_SetPinStateAtomic(portNumber port, pinNumber pin, pinState state)
{
	uint8_t retVal = N_OK;
	if((port < GPIO_PORT_NUM_MAX) && (pin < GPIO_PIN_NUM_MAX) && (state < GPIO_PIN_STATE_MAX))
	{
		GPIOX_BSRR(port) |= (uint32_t)((state << pin)
							| ((!state) << (pin + GPIO_BSRR_BR_PIN_OFFSET)));
		retVal = OK;
	}
	return (retVal);
}

/*!****************************************************************************
 * @brief			Locks GPIO configuration
 * @details		   	Locks GPIO configuration for the given port according to
 * 					the mask value (each bit corresponds to a pin-port). 
 * 					Registers to be locked: MODER, OTYPER, OSPEEDR, PUPDR, AFRL 
 * 					and AFRH. Needs a special write sequence to lock
 * @param[in]      	port    Holds the port number.
 * @param[in]      	mask	Holds the pins to be locked, each bit represents a
 * 							pin-port (0 to 15).
 * @return         	OK      Request successful.
 *                 	N_OK    Request was not successful.
 ******************************************************************************/
uint8_t Gpio_LockCfg(portNumber port, lockMask mask)
{
	uint8_t retVal = N_OK;
	uint32_t temp;
	if((port < GPIO_PORT_NUM_MAX))
	{
		GPIOX_LCKR(port) = (uint32_t)((1u << GPIO_LCK_KEY) | mask);
		GPIOX_LCKR(port) = (uint32_t)(mask);
		GPIOX_LCKR(port) = (uint32_t)((1u << GPIO_LCK_KEY) | mask);
		/* Dummy read, part of special write sequence. */
		(void)GPIOX_LCKR(port);
		temp = GPIOX_LCKR(port);
		/* Check for Lock Key bit. If 1, locked was successful. */
		if(temp & (1u << GPIO_LCK_KEY))
		{
			retVal = OK;
		}
	}
	return (retVal);
}

/*!****************************************************************************
 * @brief			Configures Alternate Fuction for a pin.
 * @details		   	Configures Alternate Function for a given pin of a given
 * 					port with up to 16 possible configurations. Check datasheet
 * 					for all pin configurations available.
 * @param[in]      	port    Holds the port number.
 * @param[in]      	pin	    Holds the pin number.
 * @param[in]      	func	Holds the alternate function number.
 * @return         	OK      Request successful.
 *                 	N_OK    Request was not successful.
 ******************************************************************************/
uint8_t Gpio_ConfigAltFnc(portNumber port, pinNumber pin, altFunction func)
{
	uint8_t retVal = N_OK;
	if((port < GPIO_PORT_NUM_MAX) && (pin < GPIO_PIN_NUM_MAX) && (func < GPIO_ALT_FNC_MAX))
	{
		/* If target pin is minor than 8 pin, use Alternate Function Low Reg */
		if(pin < pin8)
		{
			GPIOX_AFRL(port) |= 
				((func & GPIO_ALT_FIELD_MASK) << (pin * GPIO_ALT_FIELD_LENGTH));
			retVal = OK;
		}
		/* Else, use Alternate Function High Reg */
		else
		{
			GPIOX_AFRH(port) |= 
				((func & GPIO_ALT_FIELD_MASK) << (pin * GPIO_ALT_FIELD_LENGTH));
			retVal = OK;
		}
	}
	return (retVal);
}

/******************************************************************************
End Of File
******************************************************************************/
