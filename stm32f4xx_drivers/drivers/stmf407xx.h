/*
 * stmf407xx.h
 *
 *  Created on: Oct 23, 2024
 *      Author: athul-muralidhar
 */

#ifndef STMF407XX_H_
#define STMF407XX_H_

#include<stdint.h>

// base addresses of FLASH and SRAM memories
#define FLASH_BASE_ADDR 							0x08000000U
#define SRAM1_BASE_ADDR								0x20000000U  						// 112 Kb
#define SRAM1_SIZE									(uint32_t)((112) * (1024))  		// bytes in hex is 1C000
#define SRAM1_BASE_ADDR								SRAM1_BASE_ADDR + 1C000  			// 16 Kb
#define ROM											0x1FFF0000U
#define SRAM										SRAM1_BASE_ADDR

// AHBx and APBxperipheral base addresses
#define PERIPHERAL_BASE_ADDR						0x40000000U
#define APB1_PERIPHERAL_BASE_ADDR					PERIPHERAL_BASE_ADDR
#define APB2_PERIPHERAL_BASE_ADDR 					0x40010000U
#define AHB1_PERIPHERAL_BASE_ADDR					0x40020000U
#define AHB2_PERIPHERAL_BASE_ADDR					0x50000000U

// base addresses of peripherals on AHB1 bus
#define GPIOA_BASE_ADDR								(AHB1_PERIPHERAL_BASE_ADDR + 0x0000)
#define GPIOB_BASE_ADDR								(AHB1_PERIPHERAL_BASE_ADDR + 0x0400)
#define GPIOC_BASE_ADDR								(AHB1_PERIPHERAL_BASE_ADDR + 0x0800)
#define GPIOD_BASE_ADDR								(AHB1_PERIPHERAL_BASE_ADDR + 0x0C00)
#define GPIOE_BASE_ADDR								(AHB1_PERIPHERAL_BASE_ADDR + 0x1000)
#define GPIOF_BASE_ADDR								(AHB1_PERIPHERAL_BASE_ADDR + 0x1400)
#define GPIOG_BASE_ADDR								(AHB1_PERIPHERAL_BASE_ADDR + 0x1800)
#define GPIOI_BASE_ADDR								(AHB1_PERIPHERAL_BASE_ADDR + 0x2000)
#define GPIOJ_BASE_ADDR								(AHB1_PERIPHERAL_BASE_ADDR + 0x2400)
#define GPIOK_BASE_ADDR								(AHB1_PERIPHERAL_BASE_ADDR + 0x2800)

// base addresses of peripherals on APB1 bus
#define SPI2_BASE_ADDR								(APB1_PERIPHERAL_BASE_ADDR + 0x3800)
#define SPI3_BASE_ADDR								(APB1_PERIPHERAL_BASE_ADDR + 0x3C00)
#define USART2_BASE_ADDR							(APB1_PERIPHERAL_BASE_ADDR + 0x4400)
#define USART3_BASE_ADDR							(APB1_PERIPHERAL_BASE_ADDR + 0x4800)
#define UART4_BASE_ADDR								(APB1_PERIPHERAL_BASE_ADDR + 0x4C00)
#define UART5_BASE_ADDR								(APB1_PERIPHERAL_BASE_ADDR + 0x5000)
#define I2C1_BASE_ADDR								(APB1_PERIPHERAL_BASE_ADDR + 0x5400)
#define I2C2_BASE_ADDR								(APB1_PERIPHERAL_BASE_ADDR + 0x5800)
#define I2C3_BASE_ADDR								(APB1_PERIPHERAL_BASE_ADDR + 0x5C00)

// base addresses of peripherals on APB2 bus
#define USART1_BASE_ADDR							(APB2_PERIPHERAL_BASE_ADDR + 0x1000)
#define USART6_BASE_ADDR							(APB2_PERIPHERAL_BASE_ADDR + 0x1400)
#define SPI1_BASE_ADDR								(APB2_PERIPHERAL_BASE_ADDR + 0x3000)
#define SYSCFG_BASE_ADDR							(APB2_PERIPHERAL_BASE_ADDR + 0x3800)
#define EXTI_BASE_ADDR								(APB2_PERIPHERAL_BASE_ADDR + 0x3C00)



// =====================================  Peripheral register definition structures ====================================================================================

// general registers definition for GPIOS
typedef struct {
	uint32_t MODER;			// Offset 0x00
	uint32_t OTYPER;			// Offset 0x04
	uint32_t OSPEEDR;			// Offset 0x08
	uint32_t PUPDR;			// Offset 0x0C
	uint32_t IDR;			// Offset 0x10
	uint32_t ODR;			// Offset 0x14
	uint32_t BSRR;			// Offset 0x18
	uint32_t LCKR;			// Offset 0x1C
	uint32_t AFRL;			// Offset 0x20
	uint32_t AFRH;			// Offset 0x24
} GPIO_RegDef_t;




#endif /* STMF407XX_H_ */
