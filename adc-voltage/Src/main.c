

#include <stdint.h>
#include "stdio.h"
#include "stm32f407xx.h"
//#include "stm32f407xx_gpio_driver.h"
#include "printer.h"

#define RCC_APB2ENR_ADC1EN  (1 << 8)
#define ADC_CR2_ADON        (1 << 0)
#define ADC_CR2_SWSTART     (1 << 30)
#define ADC_SR_EOC			(1 << 1)


void delay(void) {
	for (uint32_t i = 0; i < 500000; i++)
		;
}

void ADC_Init(void) {
	GPIO_Handle_t GpioA;

	// push pull output type  configuration for LED
	GpioA.pGPIOx = GPIOD;
	GpioA.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioA.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
	GpioA.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioA.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP; // push pull output type
	GpioA.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioA);



    // Enable ADC clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // Configure ADC1
    ADC1->CR1 = 0;
    ADC1->CR2 = 0;
    ADC1->SQR1 = 0;
    ADC1->SQR3 = 0;  // Channel 0 (PA0)

    // Set ADC clock prescaler
//    ADC->CCR &= ~ADC_CCR_ADCPRE;
//    ADC->CCR |= ADC_CCR_ADCPRE_0;  // PCLK2 divided by 4

    // Enable ADC
    ADC1->CR2 |= ADC_CR2_ADON;
}


uint16_t ADC_Read(void) {
    // Start conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;

    // Wait for conversion to complete
    while (!(ADC1->SR & ADC_SR_EOC));

    // Return the converted value
    return ADC1->DR;
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
	        uint16_t adcValue = ADC_Read();
	        char buffer[50];


	        delay();

	        float voltage = ConvertToVoltage(adcValue);
	        snprintf(buffer, sizeof(buffer), "voltage is %f", voltage);
	        ITMPrint(buffer);


	        delay();
	    }


	return 0;
}
