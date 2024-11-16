/*
 * led_btn.c
 *
 *  Created on: Oct 27, 2024
 *      Author: athul-muralidhar
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

#define HIGH		 1
#define BTN_PRESSED  HIGH

void delay(void) {
	for (uint32_t i = 0; i < 500000; i++)
		;
}

int main(void) {
	uint32_t whileLoopProfiler = 0;

	GPIO_Handle_t GpioLed, GpioBtn;

	// push pull output type  configuration for LED
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP; // push pull output type
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);

	// button configuration
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioBtn);

	while (1) {
		if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED) {
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
			delay();

		}
		whileLoopProfiler++;

	}

	return 0;

}
