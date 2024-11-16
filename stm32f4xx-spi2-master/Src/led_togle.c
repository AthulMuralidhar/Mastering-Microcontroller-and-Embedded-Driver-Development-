/*
 * led_togle.c
 *
 *  Created on: Oct 27, 2024
 *      Author: athul-muralidhar
 */


#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"



void delay(void) {
	for(uint32_t i=0; i < 500000; i++);
}

int main(void) {

	GPIO_Handle_t GpioLed;

	// push pull output type  configuration
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;     // push pull output type
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;



//	// open drain type  configuration
//  what happens in this dicfui is that the pull up resistor is a high value resists ~40kOhm, so
//	little current flows and the LED blinking is not visible
//	GpioLed.pGPIOx = GPIOD;
//	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
//	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
//	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;
//	GpioLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_OD;     // open drain output type
//	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
//

	GPIO_PClockControl(GPIOD, ENABLE);

	GPIO_Init(&GpioLed);

	while(1) {
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();
	}


	return 0;

}
