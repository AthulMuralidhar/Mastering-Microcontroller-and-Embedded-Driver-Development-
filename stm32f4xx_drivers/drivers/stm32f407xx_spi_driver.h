/*
 * stm32f4xx_spi_driver.h
 *
 *  Created on: Oct 31, 2024
 *      Author: athul-muralidhar
 */

#ifndef STM32F407XX_SPI_DRIVER_H_
#define STM32F407XX_SPI_DRIVER_H_

#include <stm32f407xx.h>


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

// Handler structure for SPIx
typedef struct {
    SPI_RegDef_t *pSPIx;    // Pointer to SPI peripheral register definition
    SPI_Config_t SPIConfig;  // SPI configuration settings
} SPI_Handle_t;


// ===================================  APIs supported by this driver =========================================

// peripheral clock
void SPI_PClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

// initialization and de-initialization
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


// data read / write
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

// interrupt handling and configuration
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQHandler(SPI_Handle_t *pSPIHandle);


#endif /* STM32F407XX_SPI_DRIVER_H_ */
