/*
 * driver.h
 *
 *  Created on: Nov 8, 2024
 *      Author: athul-muralidhar
 */

#ifndef DRIVER_H_
#define DRIVER_H_



#define AHB2PERIPH_BASEADDR						0x4800 0000U /*!< Base address for AHB1 peripherals */
#define GPIOA_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0000) /*!< Base address of GPIO Port A */

#include "driver.h"
#include<stdint.h>


#define __vo volatile


/* Generic macros */
#define ENABLE 				1 /*!< Enable state */
#define DISABLE 			0 /*!< Disable state */
#define SET 				ENABLE /*!< Set state (same as ENABLE) */
#define RESET 				DISABLE /*!< Reset state (same as DISABLE) */
#define GPIO_PIN_SET        SET /*!< GPIO pin set state */
#define GPIO_PIN_RESET      RESET /*!< GPIO pin reset state */
#define FLAG_RESET         RESET /*!< Flag reset state */
#define FLAG_SET 			SET /*!< Flag set state */



#define SPI_DEVICE_MODE_MASTER          1       // SPI device mode: Master
#define SPI_DEVICE_MODE_SLAVE           0       // SPI device mode: Slave

#define SPI_BUS_CONFIG_FD               1       // SPI bus configuration: Full-duplex
#define SPI_BUS_CONFIG_HD               2       // SPI bus configuration: Half-duplex
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY   3       // SPI bus configuration: Simplex RX only

#define SPI_SCLK_SPEED_DIV2             0       // SPI clock speed: Divide by 2
#define SPI_SCLK_SPEED_DIV4             1       // SPI clock speed: Divide by 4
#define SPI_SCLK_SPEED_DIV8             2       // SPI clock speed: Divide by 8
#define SPI_SCLK_SPEED_DIV16            3       // SPI clock speed: Divide by 16
#define SPI_SCLK_SPEED_DIV32            4       // SPI clock speed: Divide by 32
#define SPI_SCLK_SPEED_DIV64            5       // SPI clock speed: Divide by 64
#define SPI_SCLK_SPEED_DIV128           6       // SPI clock speed: Divide by 128
#define SPI_SCLK_SPEED_DIV256           7       // SPI clock speed: Divide by 256

#define SPI_DFF_8BITS                   0       // SPI data frame format: 8-bit
#define SPI_DFF_16BITS                  1       // SPI data frame format: 16-bit

#define SPI_CPOL_HIGH                   1       // SPI clock polarity: High when idle
#define SPI_CPOL_LOW                    0       // SPI clock polarity: Low when idle

#define SPI_CPHA_HIGH                   1       // SPI clock phase: Capture on second edge
#define SPI_CPHA_LOW                    0       // SPI clock phase: Capture on first edge

#define SPI_SSM_EN                      1       // SPI slave select management: Hardware
#define SPI_SSM_DI                      0       // SPI slave select management: Software

/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG    ( 1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG   ( 1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG   ( 1 << SPI_SR_BSY)


/*
 * SPI application states
 */
#define SPI_READY 					0
#define SPI_BUSY_IN_RX 				1
#define SPI_BUSY_IN_TX 				2

/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT   1
#define SPI_EVENT_RX_CMPLT   2
#define SPI_EVENT_OVR_ERR    3
#define SPI_EVENT_CRC_ERR    4



// configuration structure for SPIx peripheral
typedef struct {
    uint8_t SPI_DeviceMode;  // Specifies the SPI device mode (Master/Slave)
    uint8_t SPI_BusConfig;   // Defines the SPI bus configuration (Full-duplex/Half-duplex/Simplex)
    uint8_t SPI_SClkSpeed;   // Sets the SPI clock speed
    uint8_t SPI_DFF;         // Specifies the SPI data frame format (8-bit/16-bit)
    uint8_t SPI_CPOL;        // Defines the clock polarity (Idle High/Idle Low)
    uint8_t SPI_CPHA;        // Sets the clock phase (1st/2nd clock transition)
    uint8_t SPI_SSM;         // Enables/Disables software slave management
} SPI_Config_t;




typedef struct
{
    __vo uint32_t MODER;    /*!< GPIO port mode register,                    Address offset: 0x00      */
    __vo uint32_t OTYPER;   /*!< GPIO port output type register,             Address offset: 0x04      */
    __vo uint32_t OSPEEDR;  /*!< GPIO port output speed register,            Address offset: 0x08      */
    __vo uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,       Address offset: 0x0C      */
    __vo uint32_t IDR;      /*!< GPIO port input data register,              Address offset: 0x10      */
    __vo uint32_t ODR;      /*!< GPIO port output data register,             Address offset: 0x14      */
    __vo uint32_t BSRR;     /*!< GPIO port bit set/reset register,           Address offset: 0x18      */
    __vo uint32_t LCKR;     /*!< GPIO port configuration lock register,      Address offset: 0x1C      */
    __vo uint32_t AFR[2];   /*!< GPIO alternate function registers,          Address offset: 0x20-0x24 */
}GPIO_RegDef_t;



typedef struct
{
    __vo uint32_t CR1;        /*!< Control register 1,                   Address offset: 0x00 */
    __vo uint32_t CR2;        /*!< Control register 2,                   Address offset: 0x04 */
    __vo uint32_t SR;         /*!< Status register,                      Address offset: 0x08 */
    __vo uint32_t DR;         /*!< Data register,                        Address offset: 0x0C */
    __vo uint32_t CRCPR;      /*!< CRC polynomial register,              Address offset: 0x10 */
    __vo uint32_t RXCRCR;     /*!< RX CRC register,                      Address offset: 0x14 */
    __vo uint32_t TXCRCR;     /*!< TX CRC register,                      Address offset: 0x18 */
    __vo uint32_t I2SCFGR;    /*!< I2S configuration register,           Address offset: 0x1C */
    __vo uint32_t I2SPR;      /*!< I2S prescaler register,               Address offset: 0x20 */
} SPI_RegDef_t;

// Handler structure for SPIx
typedef struct {
    SPI_RegDef_t *pSPIx;    // Pointer to SPI peripheral register definition
    SPI_Config_t SPIConfig;  // SPI configuration settings
    uint8_t  *pTxBuffer;
    uint8_t  *pRxBuffer;
    uint32_t  TxLen;
    uint32_t  RxLen;
    uint32_t  TxState;
    uint32_t  RxState;
} SPI_Handle_t;



// General pin configuration structure
typedef struct {
	uint8_t GPIO_PinNumber;          // GPIO pin number from @GPIO_PIN_NUM

	uint8_t GPIO_PinMode;            // GPIO pin mode
									 // possible values from @GPIO_PIN_MODES

	uint8_t GPIO_PinSpeed;           // GPIO pin output speed
									 // possible values: GPIO_OP_SPEED_LOW, GPIO_OP_SPEED_MEDIUM,
									 //                  GPIO_OP_SPEED_FAST, GPIO_OP_SPEED_HIGH

	uint8_t GPIO_PinPuPdControl;     // GPIO pin pull-up/pull-down configuration
									 // possible values: GPIO_NO_PUPD, GPIO_PIN_PU, GPIO_PIN_PD

	uint8_t GPIO_PinOpType;          // GPIO pin output type
									 // possible values: GPIO_OP_TYPE_PP, GPIO_OP_TYPE_OD

	uint8_t GPIO_PinAltFunMode;      // GPIO pin alternate function mode

} GPIO_PinConfig_t;

// a handler structure for GPIO pin
typedef struct {
	GPIO_RegDef_t *pGPIOx; // this pointer holds the base address of the GPIO peripheral
	GPIO_PinConfig_t GPIO_PinConfig; // this holds the GPIO pin configuration settings
} GPIO_Handle_t;


// ===================================  APIs supported by this driver =========================================

// peripheral clock
void SPI_PClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

// initialization and de-initialization
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

// enable or disable SPI
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

// SSI config
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

// data read / write
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

// data read / write interrupt based
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

// interrupt handling and configuration
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQHandler(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);



#endif /* DRIVER_H_ */
