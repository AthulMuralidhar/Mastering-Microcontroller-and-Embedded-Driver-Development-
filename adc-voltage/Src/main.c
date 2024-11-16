

#include <stdint.h>
#include "stdio.h"
#include "stm32f407xx.h"
//#include "stm32f407xx_gpio_driver.h"
#include "printer.h"

#define RCC_APB2ENR_ADC1EN  (1 << 8)
#define RCC_APB2ENR_ADC2EN  (1 << 9)
#define RCC_APB2ENR_ADC3EN  (1 << 10)

#define ADC_CR2_ADON        (1 << 0)
#define ADC_CR2_SWSTART     (1 << 30)
#define ADC_SR_EOC			(1 << 1)


void delay(void) {
	for (uint32_t i = 0; i < 500000; i++)
		;
}

void ADC_Init(void) {
	GPIO_Handle_t GpioA;

	// push pull output type  configuration for GPIOs PA0,PA1 and PA2
	GpioA.pGPIOx = GPIOA;
	GpioA.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioA.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
	GpioA.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioA.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP; // push pull output type
	GpioA.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GpioA);

	GpioA.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	GPIO_Init(&GpioA);

	GpioA.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&GpioA);



    // Enable ADC clocks
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;   // ADC1
    RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;   // ADC2
    RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;   // ADC3

    // Configure ADC1
    ADC1->CR1 = 0;
    ADC1->CR2 = 0;
    ADC1->SQR1 = 0;
    ADC1->SQR3 = 0;  // Channel 0 (PA0)


    // Configure ADC2
    ADC2->CR1 = 0;
    ADC2->CR2 = 0;
    ADC2->SQR1 = 0;
    ADC2->SQR3 = 1;  // Channel 1 (PA1)


    // Configure ADC3
    ADC3->CR1 = 0;
    ADC3->CR2 = 0;
    ADC3->SQR1 = 0;
    ADC3->SQR3 = 2;  // Channel 2 (PA2)


    // Enable ADC
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC2->CR2 |= ADC_CR2_ADON;
    ADC3->CR2 |= ADC_CR2_ADON;
}


uint16_t ADC1_Read(void) {
    // Start conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;

    // Wait for conversion to complete
    while (!(ADC1->SR & ADC_SR_EOC));

    // Return the converted value
    return ADC1->DR;
}

uint16_t ADC2_Read(void) {
    ADC2->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC2->SR & ADC_SR_EOC));
    return ADC2->DR;
}

uint16_t ADC3_Read(void) {
    ADC3->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC3->SR & ADC_SR_EOC));
    return ADC3->DR;
}

float ConvertToVoltage(uint16_t adcValue) {
    // Assuming 12-bit ADC and 3.3V reference voltage
    float voltage = (float)adcValue * (3.3 / 4095.0);
    return voltage;
}


int main(void)
{

	  ADC_Init();

	    while (1) {
	        uint16_t adcValue1 = ADC1_Read();
	        uint16_t adcValue2 = ADC2_Read();
	        uint16_t adcValue3 = ADC3_Read();

	        char buffer[50];
	        delay();

	        float voltage1 = ConvertToVoltage(adcValue1);
	        delay();
	        float voltage2 = ConvertToVoltage(adcValue2);
	        delay();
	        float voltage3 = ConvertToVoltage(adcValue3);
	        delay();

	        snprintf(buffer, sizeof(buffer), "voltage is %f", (voltage1 + voltage2 + voltage3 ));
	        ITMPrint(buffer);


	        delay();
	    }


	return 0;
}
