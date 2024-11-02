/*
 * stm32f4xx_spi_driver.c
 *
 *  Created on: Oct 31, 2024
 *      Author: athul-muralidhar
 */

#include "stm32f407xx_spi_driver.h"

// helper functions
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) {
	if (pSPIx->SR & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

// peripheral clock
/**
 * @fn                  SPI_PClockControl
 *
 * @brief               Controls the peripheral clock for an SPI port
 *
 * @param[in] pSPIx     Pointer to an SPI_RegDef_t structure that contains
 *                      the configuration information for the specified SPI port
 * @param[in] EnOrDi    ENABLE or DISABLE macros to enable or disable the clock
 *
 * @return              none
 *
 * @note                Peripheral clock should be enabled before using the SPI port
 *
 */
void SPI_PClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_EN();
		}
	}

	if (EnOrDi == DISABLE) {
		// TODO: do the disable bits
	}
}

// initialization and de-initialization
/**
 * @fn                  SPI_Init
 *
 * @brief               Initializes the SPI peripheral according to the specified parameters
 *                      in the SPI_Handle_t structure
 *
 * @param[in] pSPIHandle Pointer to an SPI_Handle_t structure that contains
 *                       the configuration information for the specified SPI peripheral
 *
 * @return              none
 *
 * @note                This function must be called before using the SPI peripheral
 */
void SPI_Init(SPI_Handle_t *pSPIHandle) {
	uint32_t tempRegister = 0;
	// configure the SPI CR1 register

	// 1.configure the device mode
	tempRegister |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	// 2.configure the bus
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		// full duplex
		//  BIDI mode should be cleared
		// TODO: change these bit positions to macros
		tempRegister &= ~(1 << 15);

	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		// half duplex
		// BIDI mode should be enabled
		// TODO: change these bit positions to macros
		tempRegister |= (1 << 15);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig
			== SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		// simplex receive only
		// BIDI mode should be cleared
		// TODO: change these bit positions to macros
		tempRegister &= ~(1 << 15);
		// RXONLY bit must be set
		tempRegister |= (1 << 10);
	}

	// 3.  configure the clock speed
	// TODO: change these bit positions to macros
	tempRegister |= pSPIHandle->SPIConfig.SPI_SClkSpeed << 3;

	// 4.  configure the clock polarity
	// TODO: change these bit positions to macros
	tempRegister |= pSPIHandle->SPIConfig.SPI_CPOL << 1;

	// 5.  configure the clock phase
	tempRegister |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	// 6.  configure the data frame
	// TODO: change these bit positions to macros
	tempRegister |= pSPIHandle->SPIConfig.SPI_DFF << 11;

	pSPIHandle->pSPIx->CR1 = tempRegister;

}

/**
 * @fn                  SPI_DeInit
 *
 * @brief               De-initializes the SPI peripheral registers to their default reset values
 *
 * @param[in] pSPIx     Pointer to an SPI_RegDef_t structure that contains
 *                      the configuration information for the specified SPI peripheral
 *
 * @return              none
 *
 * @note                This function should be called when the SPI peripheral is no longer needed
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx) {
	if (pSPIx == SPI1) {
		GPIOA_REG_RESET();
	} else if (pSPIx == SPI2) {
		GPIOB_REG_RESET();
	} else if (pSPIx == SPI3) {
		GPIOC_REG_RESET();
	} else if (pSPIx == SPI4) {
		GPIOD_REG_RESET();
	}
}

// data read / write
/**
 * @fn                  SPI_SendData
 *
 * @brief               Sends data through the SPI peripheral
 *
 * @param[in] pSPIx     Pointer to an SPI_RegDef_t structure that contains
 *                      the configuration information for the specified SPI peripheral
 * @param[in] pTxBuffer Pointer to the data buffer to be transmitted
 * @param[in] Len       Length of data to be transmitted in bytes
 *
 * @return              none
 *
 * @note                This function is blocking and will wait until all data is transmitted
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {
	while (Len > 0) {
		// 1.wait till TXE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET)
			;

		// 2.check the DFF bit in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			// 16 bit data
			// 1. load the data
			pSPIx->DR = (uint16_t*) pTxBuffer;
			Len--;
			Len--;
			(uint16_t*) pTxBuffer++;
		} else {
			// 8 bit data
			// 1. load the data
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

/**
 * @fn                  SPI_ReceiveData
 *
 * @brief               Receives data through the SPI peripheral
 *
 * @param[in] pSPIx     Pointer to an SPI_RegDef_t structure that contains
 *                      the configuration information for the specified SPI peripheral
 * @param[out] pRxBuffer Pointer to the buffer where received data will be stored
 * @param[in] Len       Length of data to be received in bytes
 *
 * @return              none
 *
 * @note                This function is blocking and will wait until all data is received
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

// interrupt handling and configuration
/**
 * @fn                  SPI_IRQPriorityConfig
 *
 * @brief               Configures the priority of the SPI interrupt
 *
 * @param[in] IRQNumber IRQ number of the SPI interrupt
 * @param[in] IRQPriority Priority to be set for the SPI interrupt
 *
 * @return              none
 *
 * @note                This function should be called before enabling the SPI interrupt
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/**
 * @fn                  SPI_IRQInterruptConfig
 *
 * @brief               Enables or disables the SPI interrupt
 *
 * @param[in] IRQNumber IRQ number of the SPI interrupt
 * @param[in] EnOrDi    ENABLE or DISABLE macros to enable or disable the interrupt
 *
 * @return              none
 *
 * @note                This function should be called after configuring the SPI interrupt priority
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);

/**
 * @fn                  SPI_IRQHandler
 *
 * @brief               Handles the SPI interrupt
 *
 * @param[in] pSPIHandle Pointer to an SPI_Handle_t structure that contains
 *                       the configuration information for the specified SPI peripheral
 *
 * @return              none
 *
 * @note                This function should be called in the SPI ISR
 */
void SPI_IRQHandler(SPI_Handle_t *pSPIHandle);

