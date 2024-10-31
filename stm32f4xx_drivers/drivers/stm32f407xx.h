/*
 * stmf407xx.h
 *
 *  Created on: Oct 23, 2024
 *      Author: athul-muralidhar
 */

#ifndef STM32F407XX_H_
#define STM32F407XX_H_

#include<stdint.h>

// ============================================== Processor specific details ======================================================================

// IRQ numbers for peripherals - this is the position in the vector table
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI_10			40

#define  NUM_PR_BITS_IMPLEMENTED				4

// IRQ priorities levels
#define NVIC_IRQ_PRIO0		0
// TODO
#define NVIC_IRQ_PRIO15		15

// ARM Cortex Mx processors NVIC ISERx register addresses
#define NVIC_ISER0		((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1		((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2		((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3		((volatile uint32_t*)0xE000E10C)

// ARM Cortex Mx processors NVIC ICERx register addresses
#define NVIC_ICER0		((volatile uint32_t*)0XE000E180)
#define NVIC_ICER1		((volatile uint32_t*)0XE000E184)
#define NVIC_ICER2		((volatile uint32_t*)0XE000E188)
#define NVIC_ICER3		((volatile uint32_t*)0XE000E18C)

// ARM Cortex Mx processors NVIC ICERx register base address
#define NVIC_IPR_BASE_ADDR		((volatile uint32_t*)0xE000E400)

// ========================================================= Vendor specific details =====================================================================
// generic macros
#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET 		RESET

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
#define GPIOH_BASE_ADDR								(AHB1_PERIPHERAL_BASE_ADDR + 0x1C00)
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
#define SPI4_BASE_ADDR								(APB2_PERIPHERAL_BASE_ADDR + 0x3400)
#define SPI5_BASE_ADDR								(APB2_PERIPHERAL_BASE_ADDR + 0x5000)
#define SPI6_BASE_ADDR								(APB2_PERIPHERAL_BASE_ADDR + 0x5400)

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

// EXTI
#define EXTI		((EXTI_RegDef_t *)EXTI_BASE_ADDR)

// SYSCFG
#define SYSCFG		((SYSCFG_RegDef_t *)SYSCFG_BASE_ADDR)

// SPI
#define SPI1		((SPI_RegDef_t *)SPI1_BASE_ADDR)
#define SPI2		((SPI_RegDef_t *)SPI2_BASE_ADDR)
#define SPI3		((SPI_RegDef_t *)SPI3_BASE_ADDR)
#define SPI4		((SPI_RegDef_t *)SPI4_BASE_ADDR)

// clock enable macros for GPIOx Peripherals
#define GPIOA_PCLK_EN()				(RCC->AHB1ENR |=  (1 << 0))
#define GPIOB_PCLK_EN()				(RCC->AHB1ENR |=  (1 << 1))
#define GPIOC_PCLK_EN()				(RCC->AHB1ENR |=  (1 << 2))
#define GPIOD_PCLK_EN()				(RCC->AHB1ENR |=  (1 << 3))
#define GPIOE_PCLK_EN()				(RCC->AHB1ENR |=  (1 << 4))
#define GPIOF_PCLK_EN()				(RCC->AHB1ENR |=  (1 << 5))
#define GPIOG_PCLK_EN()				(RCC->AHB1ENR |=  (1 << 6))
#define GPIOH_PCLK_EN()				(RCC->AHB1ENR |=  (1 << 7))
#define GPIOI_PCLK_EN()				(RCC->AHB1ENR |=  (1 << 8))
// clock disable macros for GPIOx Peripherals
#define GPIOA_PCLK_DI()				(RCC->AHB1ENR &=  ~(1 << 0))
#define GPIOB_PCLK_DI()				(RCC->AHB1ENR &=  ~(1 << 1))
#define GPIOC_PCLK_DI()				(RCC->AHB1ENR &=  ~(1 << 2))
#define GPIOD_PCLK_DI()				(RCC->AHB1ENR &=  ~(1 << 3))
#define GPIOE_PCLK_DI()				(RCC->AHB1ENR &=  ~(1 << 4))
#define GPIOF_PCLK_DI()				(RCC->AHB1ENR &=  ~(1 << 5))
#define GPIOG_PCLK_DI()				(RCC->AHB1ENR &=  ~(1 << 6))
#define GPIOH_PCLK_DI()				(RCC->AHB1ENR &=  ~(1 << 7))
#define GPIOI_PCLK_DI()				(RCC->AHB1ENR &=  ~(1 << 8))
// GPIOx register reset
#define GPIOA_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &=  ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &=  ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &=  ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &=  ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &=  ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &=  ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &=  ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &=  ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &=  ~(1 << 8)); }while(0)

#define GPIO_BASE_ADDR_TO_EXTI_CODE(x) ((x) == GPIOA ? 0 : \
                                        (x) == GPIOB ? 1 : \
                                        (x) == GPIOC ? 2 : \
                                        (x) == GPIOD ? 3 : \
                                        (x) == GPIOE ? 4 : \
                                        (x) == GPIOF ? 5 : \
                                        (x) == GPIOG ? 6 : \
                                        (x) == GPIOH ? 7 : \
                                        (x) == GPIOI ? 8 : -1)

// clock enable macros for I2Cx Peripherals
#define I2C1_PCLK_EN()				(RCC->APB1ENR |=  (1 << 21))
#define I2C2_PCLK_EN()				(RCC->APB1ENR |=  (1 << 22))
#define I2C3_PCLK_EN()				(RCC->APB1ENR |=  (1 << 23))

// clock enable macros for SPIx Peripherals
#define SPI1_PCLK_EN()				(RCC->APB2ENR |=  (1 << 12))
#define SPI2_PCLK_EN()				(RCC->APB1ENR |=  (1 << 14))
#define SPI3_PCLK_EN()				(RCC->APB1ENR |=  (1 << 15))
#define SPI4_PCLK_EN()				(RCC->APB2ENR |=  (1 << 13))
// SPIx register reset
// TODO: the rest of the resets
#define SPI1_REG_RESET()			do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &=  ~(1 << 12)); }while(0)

// clock enable macros for USARTx Peripherals
#define USART1_PCLK_EN()			(RCC->APB2ENR |=  (1 << 4))
#define USART2_PCLK_EN()			(RCC->APB1ENR |=  (1 << 17))
#define USART3_PCLK_EN()			(RCC->APB1ENR |=  (1 << 16))
#define UART4_PCLK_EN()				(RCC->APB1ENR |=  (1 << 19))
#define UART5_PCLK_EN()				(RCC->APB1ENR |=  (1 << 20))
#define USART6_PCLK_EN()			(RCC->APB2ENR |=  (1 << 5))

// system config clock enable
#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |=  (1 << 14))

// =====================================  Peripheral register definition structures ====================================================================================
// Peripheral register definition structure for GPIOs
typedef struct {
    volatile uint32_t MODER;    // GPIO port mode register
    volatile uint32_t OTYPER;   // GPIO port output type register
    volatile uint32_t OSPEEDR;  // GPIO port output speed register
    volatile uint32_t PUPDR;    // GPIO port pull-up/pull-down register
    volatile uint32_t IDR;      // GPIO port input data register
    volatile uint32_t ODR;      // GPIO port output data register
    volatile uint32_t BSRR;     // GPIO port bit set/reset register
    volatile uint32_t LCKR;     // GPIO port configuration lock register
    volatile uint32_t AFR[2];   // GPIO alternate function low/high register
} GPIO_RegDef_t;

// Peripheral register definition structure for RCC
typedef struct {
    volatile uint32_t CR;           // Clock control register
    volatile uint32_t PLLCFGR;      // PLL configuration register
    volatile uint32_t CFGR;         // Clock configuration register
    volatile uint32_t CIR;          // Clock interrupt register
    volatile uint32_t AHB1RSTR;     // AHB1 peripheral reset register
    volatile uint32_t AHB2RSTR;     // AHB2 peripheral reset register
    volatile uint32_t AHB3RSTR;     // AHB3 peripheral reset register
    uint32_t RESERVED0;             // Reserved
    volatile uint32_t APB1RSTR;     // APB1 peripheral reset register
    volatile uint32_t APB2RSTR;     // APB2 peripheral reset register
    uint32_t RESERVED1[2];          // Reserved
    volatile uint32_t AHB1ENR;      // AHB1 peripheral clock enable register
    volatile uint32_t AHB2ENR;      // AHB2 peripheral clock enable register
    volatile uint32_t AHB3ENR;      // AHB3 peripheral clock enable register
    uint32_t RESERVED2;             // Reserved
    volatile uint32_t APB1ENR;      // APB1 peripheral clock enable register
    volatile uint32_t APB2ENR;      // APB2 peripheral clock enable register
    uint32_t RESERVED3[2];          // Reserved
    volatile uint32_t AHB1LPENR;    // AHB1 peripheral clock enable in low power mode register
    volatile uint32_t AHB2LPENR;    // AHB2 peripheral clock enable in low power mode register
    volatile uint32_t AHB3LPENR;    // AHB3 peripheral clock enable in low power mode register
    uint32_t RESERVED4;             // Reserved
    volatile uint32_t APB1LPENR;    // APB1 peripheral clock enable in low power mode register
    volatile uint32_t APB2LPENR;    // APB2 peripheral clock enable in low power mode register
    uint32_t RESERVED5[2];          // Reserved
    volatile uint32_t RCC_BDCR;     // Backup domain control register
    volatile uint32_t RCC_CSR;      // Clock control & status register
    uint32_t RESERVED6[2];          // Reserved
    volatile uint32_t RCC_SSCGR;    // Spread spectrum clock generation register
    volatile uint32_t PLLI2SCFGR;   // PLLI2S configuration register
} RCC_RegDef_t;

// Peripheral register definition structure for EXTI
typedef struct {
    volatile uint32_t IMR;    // Interrupt mask register
    volatile uint32_t EMR;    // Event mask register
    volatile uint32_t RTSR;   // Rising trigger selection register
    volatile uint32_t FTSR;   // Falling trigger selection register
    volatile uint32_t SWIER;  // Software interrupt event register
    volatile uint32_t PR;     // Pending register
} EXTI_RegDef_t;

// Peripheral register definition structure for SYSCFG
typedef struct {
    volatile uint32_t MEMRMP;     // Memory remap register
    volatile uint32_t PMC;        // Peripheral mode configuration register
    volatile uint32_t EXTICR[4];  // External interrupt configuration register 1-4
    volatile uint32_t CMPCR;      // Compensation cell control register
} SYSCFG_RegDef_t;

// Peripheral register definition structure for SPIx
typedef struct {
    volatile uint32_t CR1;      // SPI control register 1
    volatile uint32_t CR2;      // SPI control register 2
    volatile uint32_t SR;       // SPI status register
    volatile uint32_t DR;       // SPI data register
    volatile uint32_t CRCPR;    // SPI CRC polynomial register
    volatile uint32_t RXCRCR;   // SPI RX CRC register
    volatile uint32_t TXCRCR;   // SPI TX CRC register
    volatile uint32_t I2SCFGR;  // SPI_I2S configuration register
    volatile uint32_t I2SPR;    // SPI_I2S prescaler register
} SPI_RegDef_t;

#endif /* STM32F407XX_H_ */
