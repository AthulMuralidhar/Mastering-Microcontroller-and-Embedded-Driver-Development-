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




} GPIO_Handle_t;


#endif /* STM32F4XX_GPIO_DRIVER_H_ */
