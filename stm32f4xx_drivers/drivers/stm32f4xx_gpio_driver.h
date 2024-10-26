/*
 * stm32f4xx_gpio_driver.h
 *
 *  Created on: Oct 25, 2024
 *      Author: athul-muralidhar
 */

#ifndef STM32F4XX_GPIO_DRIVER_H_
#define STM32F4XX_GPIO_DRIVER_H_

#include "stmf407xx.h"

// GPIO pin possible modes
#define GPIO_MODE_INPUT     0  // Configures the GPIO pin as an input for reading external signals
#define GPIO_MODE_OUTPUT    1  // Configures the GPIO pin as an output for driving external devices
#define GPIO_MODE_ALT_FN    2  // Sets the GPIO pin for alternate function (e.g., UART, SPI, I2C)
#define GPIO_MODE_ANALOG    3  // Configures the GPIO pin for analog functionality (ADC or DAC)
#define GPIO_MODE_IT_FT     4  // Sets the GPIO pin to generate an interrupt on falling edge
#define GPIO_MODE_IT_RT     5  // Sets the GPIO pin to generate an interrupt on rising edge
#define GPIO_MODE_IT_RFT    6  // Configures the GPIO pin for interrupt on both rising and falling edges

// GPIO pin possible output types
#define GPIO_OP_TYPE_PP    0  // Configures the GPIO pin as Push-Pull output type
#define GPIO_OP_TYPE_OD    1  // Configures the GPIO pin as Open-Drain output type



typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOpType;
	uint8_t GPIO_PinAltFunMode;

} GPIO_PinConfig_t;

// a handler structure for GPIO pin
typedef struct {
	GPIO_RegDef_t *pGPIOx;// this pointer holds the base address of the GPIO peripheral
	GPIO_PinConfig_t GPIO_PinConfig; // this holds the GPIO pin configuration settings
} GPIO_Handle_t;

// ===================================  APIs supported by this driver =========================================

// peripheral clock
void GPIO_PClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

// initialization and de-initialization
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// data read / write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WrtiteToOutoutPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_WrtiteToOutoutPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// interrupt handling and configuration
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi);
void GPIO_IRQHandler(uint8_t PinNumber);

// ================================ Errors ========================================
uint32_t errorCode = 0;

#define	ERR_INVALID_GPIO_PORT  -1

#endif /* STM32F4XX_GPIO_DRIVER_H_ */
