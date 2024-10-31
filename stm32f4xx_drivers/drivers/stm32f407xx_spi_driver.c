/*
 * stm32f4xx_spi_driver.c
 *
 *  Created on: Oct 31, 2024
 *      Author: athul-muralidhar
 */

#include "stm32f407xx_spi_driver.h"

// configuration structure for SPIx peripheral

typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SClkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPI_Config_t;

// handler structure for SPIx
typedef struct {
	SPI_Reg_Def_t *pSPIx;
	SPI_Config_t SPIConfig;
}  SPI_Handler_t;
