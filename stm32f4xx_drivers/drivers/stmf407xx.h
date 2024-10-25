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
#define SRAM2_BASE_ADDR								(SRAM1_BASE_ADDR + 1C000)  			// 16 Kb
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
#define RCC_BASE_ADDR								(AHB1_PERIPHERAL_BASE_ADDR + 0x3800)

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
	volatile uint32_t MODER;				// Offset 0x00
	volatile uint32_t OTYPER;				// Offset 0x04
	volatile uint32_t OSPEEDR;				// Offset 0x08
	volatile uint32_t PUPDR;				// Offset 0x0C
	volatile uint32_t IDR;					// Offset 0x10
	volatile uint32_t ODR;					// Offset 0x14
	volatile uint32_t BSRR;					// Offset 0x18
	volatile uint32_t LCKR;					// Offset 0x1C
	volatile uint32_t AFR[2];				// Offset 0x20-0x24
} GPIO_RegDef_t;

typedef struct {
	volatile uint32_t CR;					// Offset 0x00
	volatile uint32_t PLLCFGR;				// Offset 0x04
	volatile uint32_t CFGR;					// Offset 0x08
	volatile uint32_t CIR;					// Offset 0x0C
	volatile uint32_t AHB1RSTR;				// Offset 0x10
	volatile uint32_t AHB2RSTR;				// Offset 0x14
	volatile uint32_t AHB3RSTR;				// Offset 0x18
	uint32_t RESERVED0;						// Offset 0x1C
	volatile uint32_t APB1RSTR;				// Offset 0x20
	volatile uint32_t APB2RSTR;				// Offset 0x24
	uint32_t RESERVED1[2];					// Offset 0x28-0x2C
	volatile uint32_t AHB1ENR;				// Offset 0x30
	volatile uint32_t AHB2ENR;				// Offset 0x34
	volatile uint32_t AHB3ENR;				// Offset 0x38
	uint32_t RESERVED2;						// Offset 0x3C
	volatile uint32_t APB1ENR;				// Offset 0x40
	volatile uint32_t APB2ENR;				// Offset 0x44
	uint32_t RESERVED3[2];					// Offset 0x48-0x4C
	volatile uint32_t AHB1LPENR;			// Offset 0x50
	volatile uint32_t AHB2LPENR;			// Offset 0x54
	volatile uint32_t AHB3LPENR;			// Offset 0x58
	uint32_t RESERVED4;						// Offset 0x5C
	volatile uint32_t APB1LPENR;			// Offset 0x60
	volatile uint32_t APB2LPENR;			// Offset 0x64
	uint32_t RESERVED5[2];					// Offset 0x68-0x6C
	volatile uint32_t RCC_BDCR;				// Offset 0x70
	volatile uint32_t RCC_CSR;				// Offset 0x74
	uint32_t RESERVED6[2];					// Offset 0x78-0x7C
	volatile uint32_t RCC_SSCGR;			// Offset 0x80
	volatile uint32_t PLLI2SCFGR;			// Offset 0x84
} RCC_RegDef_t;


// peripheral definitions
#define GPIOA   	((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB   	((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC	   	((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD   	((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE   	((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOF   	((GPIO_RegDef_t*)GPIOF_BASE_ADDR)
#define GPIOG   	((GPIO_RegDef_t*)GPIOG_BASE_ADDR)
#define GPIOH   	((GPIO_RegDef_t*)GPIOH_BASE_ADDR)
#define GPIOI   	((GPIO_RegDef_t*)GPIOI_BASE_ADDR)

// RCC
#define RCC			((RCC_RegDef_t*)RCC_BASE_ADDR)

// clock enable macros for GPIOx Peripherals
#define GPIOA_PCLK_EN()			(RCC->AHB1ENR |=  (1 << 0))
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR |=  (1 << 1))
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR |=  (1 << 2))
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR |=  (1 << 3))
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR |=  (1 << 4))
#define GPIOF_PCLK_EN()			(RCC->AHB1ENR |=  (1 << 5))
#define GPIOG_PCLK_EN()			(RCC->AHB1ENR |=  (1 << 6))
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR |=  (1 << 7))
#define GPIOI_PCLK_EN()			(RCC->AHB1ENR |=  (1 << 8))

// clock enable macros for I2Cx Peripherals
#define I2C1_PCLK_EN()			(RCC->APB1ENR |=  (1 << 21))
#define I2C2_PCLK_EN()			(RCC->APB1ENR |=  (1 << 22))
#define I2C3_PCLK_EN()			(RCC->APB1ENR |=  (1 << 23))


// clock enable macros for SPIx Peripherals
#define SPI1_PCLK_EN()			(RCC->APB2ENR |=  (1 << 12))
#define SPI2_PCLK_EN()			(RCC->APB1ENR |=  (1 << 14))
#define SPI3_PCLK_EN()			(RCC->APB1ENR |=  (1 << 15))
#define SPI4_PCLK_EN()			(RCC->APB2ENR |=  (1 << 13))


// clock enable macros for USARTx Peripherals
#define USART1_PCLK_EN()			(RCC->APB2ENR |=  (1 << 4))
#define USART2_PCLK_EN()			(RCC->APB1ENR |=  (1 << 17))
#define USART3_PCLK_EN()			(RCC->APB1ENR |=  (1 << 16))
#define UART4_PCLK_EN()				(RCC->APB1ENR |=  (1 << 19))
#define UART5_PCLK_EN()				(RCC->APB1ENR |=  (1 << 20))
#define USART6_PCLK_EN()			(RCC->APB2ENR |=  (1 << 5))


// sys cfg clock
#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |=  (1 << 14))












#endif /* STMF407XX_H_ */
