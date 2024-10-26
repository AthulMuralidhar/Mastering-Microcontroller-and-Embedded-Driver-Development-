/*
 * stm32f4xx_gpio_driver.h
 *
 *  Created on: Oct 25, 2024
 *      Author: athul-muralidhar
 */

#ifndef STM32F4XX_GPIO_DRIVER_H_
#define STM32F4XX_GPIO_DRIVER_H_

#include "stmf407xx.h"



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
	GPIO_RegDef_t *pGPIOx;				// this pointer holds the base address of the GPIO peripheral
	GPIO_PinConfig_t  GPIO_PinConfig;   // this holds the GPIO pin configuration settings
} GPIO_Handle_t;


// ===================================  APIs supported by this driver =========================================

// peripheral clock
void GPIO_PClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);


// initialization and de-initialization
void GPIO_Init(GPIO_Handle_t  *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// data read / write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WrtiteToOutoutPort(GPIO_RegDef_t *pGPIOx,uint16_t value);
void GPIO_WrtiteToOutoutPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// interrupt handling and configuration
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi);
void GPIO_IRQHandler(uint8_t PinNumber);


#endif /* STM32F4XX_GPIO_DRIVER_H_ */
