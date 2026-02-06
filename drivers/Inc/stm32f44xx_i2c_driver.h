/*
 * stm32f44xx_i2c_driver.h
 *
 *  Created on: Jul 18, 2025
 *      Author: animeshsrivastava
 */

#ifndef INC_STM32F44XX_I2C_DRIVER_H_
#define INC_STM32F44XX_I2C_DRIVER_H_

#include "stm32f44xx.h"

//Config structure
typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_AckControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

//Handle structure
typedef struct
{
	I2C_RegDef_t 	*pI2Cx;
	I2C_Config_t 	I2C_Config;
	uint8_t 		*pTxBuffer; /* !< To store the app. Tx buffer address > */
	uint8_t 		*pRxBuffer;	/* !< To store the app. Rx buffer address > */
	uint32_t 		TxLen;		/* !< To store Tx len > */
	uint32_t 		RxLen;		/* !< To store Tx len > */
	uint8_t 		TxRxState;	/* !< To store Communication state > */
	uint8_t 		DevAddr;	/* !< To store slave/device address > */
    uint32_t        RxSize;		/* !< To store Rx size  > */
    uint8_t         Sr;			/* !< To store repeated start value  > */
}I2C_Handle_t;


#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM4K	400000
#define I2C_SCL_SPEED_FM2K  200000

#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

// I2C related status flags definitions
#define I2C_FLAG_TXE       I2C_SR1_TXE
#define I2C_FLAG_RXNE      I2C_SR1_RXNE
#define I2C_FLAG_SB 		(1 << I2C_SR1_SB)
#define I2C_FLAG_OVR 		(1 << I2C_SR1_OVR)
#define I2C_FLAG_AF 		(1 << 12C_SR1_AF)
#define I2C_FLAG_ARLO 		(1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR 		(1 << 12C_ 12C_SR1_BERR)
#define I2C_FLAG_STOPF 		(1 << I2C_SR1_STOPF)
#define I2C_FLAG_ADD10 		(1 << I2C_SR1_ADD10)
#define I2C_FLAG_BTF 		(1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR 		(1 << I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT 	(1 << I2C_SR1_TIMEOUT)

#define I2C_SR2_PEC_Pos          8            // PEC value position (bits 15:8)
#define I2C_SR2_PEC_Msk          (0xFF << I2C_SR2_PEC_Pos)  // PEC mask

#define I2C_CCR_CCR_Pos          0            // Clock control bits position (bits 11:0)
#define I2C_CCR_CCR_Msk          (0xFFF << I2C_CCR_CCR_Pos) // Mask for CCR bits

#define I2C_CCR_DUTY             (1U << 14)   // Fast mode duty cycle (0: 2, 1: 16/9)
#define I2C_CCR_FS               (1U << 15)   // I2C Master mode selection (0: Standard, 1: Fast)

#define I2C_TRISE_Pos      		 0            // Maximum rise time position (bits 5:0)
#define I2C_Msk      			 (0x3F << I2C_TRISE_TRISE_Pos) // Mask for TRISE bits

#define I2C_READY 					0
#define I2C_BUSY_IN_RX 				1
#define I2C_BUSY_IN_TX 				2


// APIs
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);

void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);


void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

uint8_t I2C_GetFlagStatus (I2C_RegDef_t *pI2Cx, uint32_t FlagName);

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);



#endif /* INC_STM32F44XX_I2C_DRIVER_H_ */
