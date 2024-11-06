/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Nov 6, 2024
 *      Author: athul-muralidhar
 */


#include "stm32f407xx_i2c_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = { 2, 4 , 8, 16};

uint8_t RCC_GetPLLOutputClock() {
	return 1;
}

uint32_t RCC_GetPCLK1Value() {
	uint8_t clockSource, SystemClk, ahb1PreScalerReg,ahb1PreScaler,apb1PreScalerReg, apb1PreScaler;
	uint32_t pClock;

	clockSource = (RCC->CFGR >> 2) & 0x3; // read the SWS bit

	// determine the clock source
	if(clockSource == 0 )
	{
		SystemClk = (uint8_t)16000000;
	}else if(clockSource == 1)
	{
		SystemClk = (uint8_t) 8000000;
	}else if (clockSource == 2)
	{
		SystemClk = (uint8_t) RCC_GetPLLOutputClock();
	}

	// determine the AHB1 prescaler

	ahb1PreScalerReg = (RCC->CFGR >> 7) & 0xF; // 4 bits for AHB1 => 0xF mask
	if(ahb1PreScalerReg < 8)
		{
		ahb1PreScaler = 1;
		}else
		{
			ahb1PreScaler = AHB_PreScaler[ahb1PreScalerReg-8];
		}



	// determine the APB1 prescaler

	//apb1
	apb1PreScalerReg = ((RCC->CFGR >> 10 ) & 0x7);

	if(apb1PreScalerReg < 4)
	{
		apb1PreScaler = 1;
	}else
	{
		apb1PreScaler = APB1_PreScaler[apb1PreScalerReg-4];
	}

	pClock =  (SystemClk / ahb1PreScaler) /apb1PreScaler;



	return pClock;
}



static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_START);
}



static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); //SlaveAddr is Slave address + r/nw bit=0
	pI2Cx->DR = SlaveAddr;
}


static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle )
{
	uint32_t dummy_read;
	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
	{
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize  == 1)
			{
				//first disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);

				//clear the ADDR flag ( read SR1 , read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}

		}
		else
		{
			//clear the ADDR flag ( read SR1 , read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;

		}

	}
	else
	{
		//device is in slave mode
		//clear the ADDR flag ( read SR1 , read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}


}


/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi) {
	if(EnOrDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		//TODO
	}
}

/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle) {

	uint32_t tempReg = 0;


	// enable the clock for the i2cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// ACK control bit
	tempReg |= (pI2CHandle->I2C_Config.I2C_AckControl << 10);
	pI2CHandle->pI2Cx->CR1 = tempReg;

	// configure the FREQ field of CR2
	tempReg = 0;
	// get clock value
	tempReg |= RCC_GetPCLK1Value() /1000000U ;
	pI2CHandle->pI2Cx->CR2 =  (tempReg & 0x3F);

   //program the device own address
	tempReg = 0;
	tempReg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempReg |= ( 1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempReg;

	// CCR calculations

	   //program the device own address
	tempReg = 0;
	tempReg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempReg |= ( 1 << 14);
		pI2CHandle->pI2Cx->OAR1 = tempReg;

		//CCR calculations
		uint16_t ccr_value = 0;
		tempReg = 0;
		if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
		{
			//mode is standard mode
			ccr_value = (RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
			tempReg |= (ccr_value & 0xFFF);  // only 3 bits needed
		}else
		{
			//mode is fast mode
			tempReg |= ( 1 << 15);
			tempReg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
			if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
			{
				ccr_value = (RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
			}else
			{
				ccr_value = (RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
			}
			tempReg |= (ccr_value & 0xFFF);
		}
		pI2CHandle->pI2Cx->CCR = tempReg;

	// TRISE Configuration

}

void I2C_DeInit(I2C_RegDef_t *pI2Cx);


/*
 * Data Send and Receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr) {
	// 1. Generate the START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//2. confirm that start generation is completed by checking the SB flag in the SR1
		//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
		while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB)   );

		//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
		I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

		//4. Confirm that address phase is completed by checking the ADDR flag in teh SR1
		while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR)   );

		//5. clear the ADDR flag according to its software sequence
		//   Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
		I2C_ClearADDRFlag(pI2CHandle);


		//6. send the data until len becomes 0
		while(Len > 0)
		{
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE) ); //Wait till TXE is set
			pI2CHandle->pI2Cx->DR = *pTxbuffer;
			pTxbuffer++;
			Len--;
		}

		//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
		//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
		//   when BTF=1 SCL will be stretched (pulled to LOW)
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE) );
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF) );


		//8. Generate STOP condition and master need not to wait for the completion of stop condition.
		//   Note: generating STOP, automatically clears the BTF
		if(Sr == I2C_DISABLE_SR ) {
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);


void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);

/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);


/*
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);

/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);
