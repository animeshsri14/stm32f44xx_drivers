/*
 * i2c_master_tx_testing.c
 *
 *  Created on: Jul 21, 2025
 *      Author: animeshsrivastava
 */

#include <string.h>
#include<stdio.h>
#include "stm32f44xx.h"

#define SLAVE_ADDR  0x68

void delay(void) {
    for (uint32_t i = 0; i < 500000; i++);
}

uint8_t some_data[] = "We are testing I2C master Tx\n";

I2C_Handle_t I2C1Handle;

void I2C1_GPIOInits(void) {
	GPIO_Handle_t I2CPins;
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//scl

	I2CPins. GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	// sda

	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&I2CPins);
}

void GPIO_ButtonInit(void) {
    GPIO_Handle_t GPIOBtn;
    GPIO_Handle_t GpioLed;

    GPIOBtn.pGPIOx = GPIOA;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_Init(&GPIOBtn);

    GpioLed.pGPIOx = GPIOD;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl (GPIOD, ENABLE) ;

    GPIO_Init (&GpioLed);
}

void I2C1_Inits(void) {

	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = 0x61;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);

}

int main() {

	// i2c pin inits
	I2C1_GPIOInits();

	// button and led inits
	GPIO_ButtonInit();

	// i2c peripheral configuration
	I2C1_Inits();

	// enable the i2c peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	while(1) {
		// wait for button press
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// simple debounce
		delay();

		// send data
		I2C_MasterSendData(&I2C1Handle, some_data, strlen((char*) some_data), SLAVE_ADDR,1);
	}
}
