/*
 * stm32f44xx_i2c_driver.c
 *
 *  Created on: Jul 18, 2025
 *      Author: animeshsrivastava
 */

#include "stm32f44xx.h"
#include <stdio.h>
#include <strings.h>


static const uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
static const uint8_t APB1_PreScaler [4] = { 2, 4 , 8, 16};
uint8_t trise;

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle );
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle );


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr) {
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); //SlaveAddr is Slave address + r/nw bit=0
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr) {
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= (1); //SlaveAddr is Slave address + r/nw bit=0
	pI2Cx->DR = SlaveAddr;
}


static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx) {

	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void) dummyRead;

}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (I2C_CR1_STOP);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
    switch(AppEv)
    {
        case I2C_EV_TX_CMPLT:
            printf("I2C Event: Transmission Complete\n");
            // maybe turn on a green LED
            break;

        case I2C_EV_RX_CMPLT:
            printf("I2C Event: Reception Complete\n");
            // maybe signal main app with a flag
            break;

        case I2C_EV_STOP:
            printf("I2C Event: STOP Condition Detected\n");
            // slave mode only
            break;

        case I2C_EV_DATA_REQ:
            printf("I2C Event: Master Requests Data\n");
            // slave mode: master is requesting data
            // you can now write data into DR
            I2C_SlaveSendData(pI2CHandle->pI2Cx, 0xAB); // example byte
            break;

        case I2C_EV_DATA_RCV:
            printf("I2C Event: Data Received From Master\n");
            {
                uint8_t val = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
                printf("Received: 0x%02X\n", val);
            }
            break;

        case I2C_ERROR_BERR:
            printf("I2C Error: Bus Error\n");
            break;

        case I2C_ERROR_ARLO:
            printf("I2C Error: Arbitration Lost\n");
            break;

        case I2C_ERROR_AF:
            printf("I2C Error: ACK Failure\n");
            break;

        case I2C_ERROR_OVR:
            printf("I2C Error: Overrun/Underrun\n");
            break;

        case I2C_ERROR_TIMEOUT:
            printf("I2C Error: Timeout\n");
            break;

        default:
            printf("I2C Event: Unknown event code %d\n", AppEv);
            break;
    }
}


uint32_t RCC_GetPCLK1Value(void) {

	uint32_t pclk1,SystemClk;
	uint8_t clksrc,temp,ahbp,apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0) {
		SystemClk = 16000000;
	}
	else if(clksrc == 1){
		SystemClk = 8000000;
	}
	else if (clksrc == 2){

	}

	//ahb
	temp = ((RCC->CFGR >> 4 ) & 0xF);

	if (temp<8) {
		ahbp = 1;
	}
	else {
		ahbp = AHB_PreScaler[temp-8];
	}

	//apb1
	temp = ((RCC->CFGR >> 10 ) & 0x7);

	if (temp<4) {
		apb1p = 1;
	}
	else {
		apb1p = APB1_PreScaler [temp-4];
	}

	pclk1 = (SystemClk / ahbp) / apb1p;

	return pclk1;
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi) {
	if(EnOrDi == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
		//pI2cBaseAddress-›CR1 T= 12C_CR1_PE_Bit_Mask;
	}
	else {
		pI2Cx->CR1 &= ~(1 << 0);
	}
}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi )
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}

}

void I2C_Init(I2C_Handle_t *pI2CHandle) {
	uint32_t tempreg = 0 ;

	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE); // enables clock

	// ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_AckControl;
	pI2CHandle->pI2Cx->CR1 = tempreg;


	//configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() /1000000U ;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);


	//program the device own address
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= ( 1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config. I2C_SCLSpeed ));
		tempreg |= (ccr_value & 0xFFF);
	}

	else {
		tempreg |= ( 1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
			ccr_value = (RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}
		else {
			ccr_value = (RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}

		tempreg |= (ccr_value & 0xFFF);

	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	// TRISE Configuration

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {

		//standard mode

		trise = (RCC_GetPCLK1Value() /1000000U) + 1 ;
	}

	else {
		//fast mode
		tempreg = ( (RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

}
void I2C_DeInit(I2C_RegDef_t *pI2Cx) {

}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName) {
    if (pI2Cx->SR1 & FlagName)

        return FLAG_SET;

    return FLAG_RESET;
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr) {

	//generate start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB) );

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in teh SR1
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR) );

	//5. clear the ADDR flag according to its software sequence
	//Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	//6. send the data until len becomes 0
	while (len>0) {
		while(!I2C_GetFlagStatus (pI2CHandle->pI2Cx,I2C_FLAG_TXE)); //Wait till TXE is set
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		len--;
	}

	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	// when BTF=1 SCL will be stretched (pulled to LOW)
	while (!I2C_GetFlagStatus (pI2CHandle->pI2Cx,I2C_FLAG_TXE) );

	while (!I2C_GetFlagStatus (pI2CHandle->pI2Cx,I2C_FLAG_BTF) );

	if (Sr == 0)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);


}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t len, uint8_t SlaveAddr, uint8_t Sr) {

	//Start
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//confirm that start generation is completed by checking the SB flag in the SR1
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB) );

	//Send the address of the slave with r/nw bit set to R(1) (total 8 bits )
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

	//4. wait until address phase is completed by checking the ADDR flag in teh SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR) );

	//procedure to read only 1 byte from slave
	if (len == 1) {

		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE) );


		if (Sr==0)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	if (len>1) {

		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//read the data until Len becomes zero
		for (uint32_t i = len ; i > 0 ; i--) {

			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE) );

			if (i==2) {

				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

			}

			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address pRxBuffer++
			pRxBuffer++;

		}

	}

	//re-enable acking
	I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);

}


void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
	if (EnorDi == I2C_ACK_ENABLE) {
		pI2Cx->CR1 |= (I2C_CR1_ACK);
	}
	else {
		pI2Cx->CR1 &= ~(I2C_CR1_ACK) ;
	}
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle) {
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = 0;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
	}

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle) {
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(I2C_CR2_ITEVTEN);


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = 0;
	pI2CHandle->TxLen = 0;
}


void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data) {
	pI2C->DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C) {
    return (uint8_t) pI2C->DR;
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle ) {

	if(pI2CHandle->TxLen > 0)
	{
		//1. load the data in to DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2. decrement the TxLen
		pI2CHandle->TxLen--;

		//3. Increment the buffer address
		pI2CHandle->pTxBuffer++;

	}

}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle ) {
	//We have to do the data reception
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;

	}


	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			//clear the ack bit
			I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
		}

			//read DR
			*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
			pI2CHandle->pRxBuffer++;
			pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0 )
	{
		//close the I2C data reception and notify the application

		//1. generate the stop condition
		if(pI2CHandle->Sr == 0)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//2 . Close the I2C rx
		I2C_CloseReceiveData(pI2CHandle);

		//3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
	}
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle) {
	//Interrupt handling for both master and slave mode of a device

	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & I2C_CR2_ITEVTEN;
	temp2 = pI2CHandle->pI2Cx->CR2 & I2C_CR2_ITBUFEN ;

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB);
	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	if(temp1 && temp3)
	{
		//The interrupt is generated because of SB event
		//This block will not be executed in slave mode because for slave SB is always zero
		//In this block lets executed the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR);
	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	if(temp1 && temp3)
	{
		// interrupt is generated because of ADDR event
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF);
	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	if(temp1 && temp3)
	{
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//make sure that TXE is also set .
			if(pI2CHandle->pI2Cx->SR1 & ( I2C_SR1_TXE) )
			{
				//BTF, TXE = 1
				if(pI2CHandle->TxLen == 0 )
				{
					//1. generate the STOP condition
					if(pI2CHandle->Sr == 0)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					//2. reset all the member elements of the handle structure.
					I2C_CloseSendData(pI2CHandle);

					//3. notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);

				}
			}

		}else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
		{
			;
		}
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF);
	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	//The below code block will not be executed by the master since STOPF will not set in master mode
	if(temp1 && temp3)
	{
		//STOF flag is set
		//Clear the STOPF ( i.e 1) read SR1 2) Write to CR1 )

		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_STOP);
	}


	temp3  = pI2CHandle->pI2Cx->SR1 & (I2C_SR1_TXE);
	//5. Handle For interrupt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		//Check for device mode
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
			//TXE flag is set
			//We have to do the data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}else
		{
			//slave
			//make sure that the slave is really in transmitter mode
		    if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))
		    {
		    	I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ);
		    }
		}
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & (I2C_SR1_RXNE);
	//6. Handle For interrupt generated by RXNE event
	if(temp1 && temp2 && temp3)
	{
		//check device mode .
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
			//The device is master

			//RXNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);

			}

		}else
		{
			//slave
			//make sure that the slave is really in receiver mode
			if(!(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_RCV);
			}
		}
	}
}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle) {

    uint32_t temp1, temp2;

    // Check if error interrupts are enabled
    temp2 = (pI2CHandle->pI2Cx->CR2) & I2C_CR2_ITERREN;

    // Check for Bus error
    temp1 = (pI2CHandle->pI2Cx->SR1) & I2C_SR1_BERR;
    if (temp1 && temp2) {
        // Bus error
        pI2CHandle->pI2Cx->SR1 &= ~ I2C_SR1_BERR ;
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
    }

    // Check for Arbitration lost error
    temp1 = (pI2CHandle->pI2Cx->SR1) & (I2C_SR1_ARLO);
    if (temp1 && temp2) {
        // Arbitration lost error
        pI2CHandle->pI2Cx->SR1 &= ~(I2C_SR1_ARLO);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
    }

    // Check for ACK failure error
    temp1 = (pI2CHandle->pI2Cx->SR1) & (I2C_SR1_AF);
    if (temp1 && temp2) {
        // ACK failure error
        pI2CHandle->pI2Cx->SR1 &= ~(I2C_SR1_AF);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
    }

    // Check for Overrun/underrun error
    temp1 = (pI2CHandle->pI2Cx->SR1) & (I2C_SR1_OVR);
    if (temp1 && temp2) {
        // Overrun/underrun error
        pI2CHandle->pI2Cx->SR1 &= ~(I2C_SR1_OVR);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
    }

    // Check for Timeout error
    temp1 = (pI2CHandle->pI2Cx->SR1) & (I2C_SR1_TIMEOUT);
    if (temp1 && temp2) {
        // Timeout error
        pI2CHandle->pI2Cx->SR1 &= ~(I2C_SR1_TIMEOUT);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
    }
}


