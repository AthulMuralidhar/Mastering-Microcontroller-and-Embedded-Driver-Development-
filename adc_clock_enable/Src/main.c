/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
#warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define ADC_BASE_ADDR		0x40012000U
#define ADC_CR1_REG_OFFSET	0x04U
#define ADC_CR1_REG_ADDR	((ADC_BASE_ADDR) + (ADC_CR1_REG_OFFSET))

#define RCC_APB2ENR_OFFSET	0x44U
#define RCC_BASE_ADDR		0x40023800U

int main(void) {
	// 1. enable the clock for ADC1
	uint32_t *pRCC_APB1_Addr = (uint32_t*) (RCC_BASE_ADDR + RCC_APB2ENR_OFFSET);
	*pRCC_APB1_Addr |= (1 << 8);   // Enable 8th bit for ADC1

	// 2. Modify the ADC CR1 register
	uint32_t *pADC_CR1_Reg = (uint32_t*) ADC_CR1_REG_ADDR;
	*pADC_CR1_Reg |= (1 << 8);   // set SCAN to 1

	/* Loop forever */
	for (;;)
		;
}