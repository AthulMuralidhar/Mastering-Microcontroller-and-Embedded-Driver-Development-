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

#define FLASH_BASE_ADDR 		0x08000000U
#define SRAM1_BASE_ADDR			0x20000000U  						// 112 Kb
#define SRAM1_SIZE				(uint32_t)((112) * (1024))  		// bytes in hex is 1C00
#define SRAM1_BASE_ADDR			SRAM1_BASE_ADDR + 1C00  			//16 Kb
#define ROM						0x1FFF0000U

#define SRAM					SRAM1_BASE_ADDR



#endif /* STMF407XX_H_ */
