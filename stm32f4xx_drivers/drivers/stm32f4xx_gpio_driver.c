/*
 * stm32f4xx_gpio_driver.c
 *
 *  Created on: Oct 25, 2024
 *      Author: athul-muralidhar
 */


#include "stm32f4xx_gpio_driver.h"



// peripheral clock
/**
 * @fn					GPIO_PClockControl
 *
 * @brief				Controls the peripheral clock for a GPIO port
 *
 * @param[in] pGPIOx	Pointer to a GPIO_RegDef_t structure that contains
 * 						the configuration information for the specified GPIO port
 * @param[in] EnOrDi	ENABLE or DISABLE macros to enable or disable the clock
 *
 * @return				none
 *
 * @note				Peripheral clock should be enabled before using the GPIO port
 */
void GPIO_PClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi) {

}


// initialization and de-initialization
/**
 * @fn						GPIO_Init
 *
 * @brief					Initializes the GPIO port according to the specified parameters in the GPIO_Handle_t
 *
 * @param[in] pGPIOHandle	Pointer to a GPIO_Handle_t structure that contains
 * 							the configuration information for the specified GPIO pin
 *
 * @return					none
 *
 * @note					This function should be called before using the GPIO pin
 */
void GPIO_Init(GPIO_Handle_t  *pGPIOHandle);

/**
 * @fn					GPIO_DeInit
 *
 * @brief				De-initializes the GPIO port, resetting it to its default state
 *
 * @param[in] pGPIOx	Pointer to a GPIO_RegDef_t structure that contains
 * 						the configuration information for the specified GPIO port
 *
 * @return				none
 *
 * @note				This function resets all the GPIO port registers to their default values
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// data read / write
/**
 * @fn					GPIO_ReadFromInputPin
 *
 * @brief				Reads the current state of the specified GPIO input pin
 *
 * @param[in] pGPIOx	Pointer to a GPIO_RegDef_t structure that contains
 * 						the configuration information for the specified GPIO port
 * @param[in] PinNumber	The pin number (0-15) of the GPIO port to read from
 *
 * @return				The current state of the input pin (0 or 1)
 *
 * @note				Ensure that the specified pin is configured as an input before calling this function
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/**
 * @fn					GPIO_ReadFromInputPort
 *
 * @brief				Reads the current state of all pins of the specified GPIO port
 *
 * @param[in] pGPIOx	Pointer to a GPIO_RegDef_t structure that contains
 * 						the configuration information for the specified GPIO port
 *
 * @return				The current state of all pins of the GPIO port (16-bit value)
 *
 * @note				The returned value represents the state of all 16 pins, where each bit
 * 						corresponds to a pin (bit 0 for pin 0, bit 1 for pin 1, etc.)
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

/**
 * @fn					GPIO_WriteToOutputPort
 *
 * @brief				Writes a 16-bit value to the output of the specified GPIO port
 *
 * @param[in] pGPIOx	Pointer to a GPIO_RegDef_t structure that contains
 * 						the configuration information for the specified GPIO port
 * @param[in] value		16-bit value to be written to the GPIO port
 *
 * @return				none
 *
 * @note				Each bit in the value corresponds to a pin on the port
 * 						(bit 0 for pin 0, bit 1 for pin 1, etc.)
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {
    // Function implementation goes here
}

/**
 * @fn					GPIO_WriteToOutputPin
 *
 * @brief				Writes a value to the specified pin of the GPIO port
 *
 * @param[in] pGPIOx	Pointer to a GPIO_RegDef_t structure that contains
 * 						the configuration information for the specified GPIO port
 * @param[in] PinNumber	The pin number (0-15) of the GPIO port to write to
 * @param[in] value		Value to be written to the pin (0 or 1)
 *
 * @return				none
 *
 * @note				Ensure that the specified pin is configured as an output before calling this function
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value) {
    // Function implementation goes here
}

/**
 * @fn					GPIO_ToggleOutputPin
 *
 * @brief				Toggles the state of the specified pin of the GPIO port
 *
 * @param[in] pGPIOx	Pointer to a GPIO_RegDef_t structure that contains
 * 						the configuration information for the specified GPIO port
 * @param[in] PinNumber	The pin number (0-15) of the GPIO port to toggle
 *
 * @return				none
 *
 * @note				This function changes the state of the pin from high to low or low to high
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
    // Function implementation goes here
}


// interrupt handling and configuration
/**
 * @fn						GPIO_IRQConfig
 *
 * @brief					Configures the interrupt for a GPIO pin
 *
 * @param[in] IRQNumber	  	The IRQ (Interrupt Request) number for the GPIO pin
 * @param[in] IRQPriority 	The priority of the interrupt (0-255, with 0 being the highest priority)
 * @param[in] EnOrDi	  	ENABLE or DISABLE macros to enable or disable the interrupt
 *
 * @return					none
 *
 * @note					This function configures the interrupt controller to enable or disable
 * 							the specified GPIO interrupt and sets its priority
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi) {
    // Function implementation goes here
}

/**
 * @fn						GPIO_IRQHandler
 *
 * @brief					Handles the interrupt for a specific GPIO pin
 *
 * @param[in] PinNumber		The pin number (0-15) of the GPIO port that triggered the interrupt
 *
 * @return					none
 *
 * @note					This function should be called in the interrupt service routine (ISR)
 * 							for the GPIO pin. It typically clears the interrupt flag and performs
 * 							any necessary interrupt handling operations.
 */
void GPIO_IRQHandler(uint8_t PinNumber) {
    // Function implementation goes here
}
