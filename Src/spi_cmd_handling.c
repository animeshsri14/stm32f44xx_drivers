/*
 * spi_cmd_handling.c
 *
 *  Created on: Jul 6, 2025
 *      Author: animeshsrivastava
 */


// FOR SPI2
// PB15 MOSI
// PB14 MISO
// PB13 CLK
// PB12 NSS
// AF mode 5

#include <string.h>
#include "stm32f44xx.h"

#define COMMAND_LED_CTRL 		0x50
#define COMMAND_SENSOR_READ 	0x51
#define COMMAND_LED_READ 		0x52
#define COMMAND_PRINT 			0x53
#define COMMAND_ID_READ			0x54

#define LED_ON					1
#define LED_OFF					0

#define ANALOG_PIN0				0
#define ANALOG_PIN1				1
#define ANALOG_PIN2				2
#define ANALOG_PIN3				3
#define ANALOG_PIN4				4

#define LED_PIN 				9


void delay(void)
{
    for (uint32_t i = 0; i < 500000; i++);
}

void SPI2_GPIOInits(void)
{
    GPIO_Handle_t SPIPins;

    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    GPIO_PeriClockControl(GPIOB, ENABLE);

    // SCLK
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPIPins);

    // MISO
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    GPIO_Init(&SPIPins);

    // MOSI
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&SPIPins);

    // NSS
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&SPIPins);
}

void GPIO_ButtonInit(void)
{
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

void SPI2_Inits(void)
{
    SPI_Handle_t SPI2handle;

    SPI2handle.pSPIx = SPI2;
    SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;  // 2 MHz
    SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

    SPI_PeriClockControl(SPI2, ENABLE);
    SPI_Init(&SPI2handle);
}


uint8_t SPI_VerifyResponse(uint8_t ackbyte) {
	if (ackbyte == 0xF5) {
		return 1;
	}
	return 0;
}

int main(void)
{
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

    GPIO_ButtonInit();
    SPI2_GPIOInits();
    SPI2_Inits();

    // Enable hardware slave management
    SPI_SSOEConfig(SPI2, ENABLE);

    while (1)
    {
        // Wait for button press
        while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

        delay(); // Debounce

        SPI_PeripheralControl(SPI2, ENABLE);

        uint8_t commndcode = COMMAND_LED_CTRL;

        uint8_t ackbyte;

        uint8_t args[2];

        SPI_SendData(SPI2, &commndcode, 1);

        SPI_ReceiveData(SPI2,&dummy_read, 1);

        SPI_SendData(SPI2, &dummy_write, 1);

        SPI_ReceiveData(SPI2,&ackbyte, 1);

        if ( SPI_VerifyResponse(ackbyte) ) {
        //send arguments args [0]= LED_PIN; args [1]= LED_ON;
        	args[0] = LED_PIN;

        	args[1] = LED_ON;

        	SPI_SendData(SPI2,args,2);

        }

        while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

        delay();

        uint8_t commandcode = COMMAND_SENSOR_READ;

        SPI_SendData(SPI2, &commandcode, 1);

        SPI_ReceiveData (SPI2, &dummy_read, 1);

        SPI_SendData (SPI2, &dummy_write,1);

        SPI_ReceiveData(SPI2,&ackbyte, 1);

        if ( SPI_VerifyResponse(ackbyte) ) {
               //send arguments args [0]= LED_PIN; args [1]= LED_ON;
        	args[0] = ANALOG_PIN0;

            SPI_SendData(SPI2,args,1);

        }

        SPI_ReceiveData (SPI2, &dummy_read,1);

        delay();
        //Send some dummy bits (1 byte) fetch the response from the slave
        SPI_SendData(SPI2, &dummy_write,1);
        uint8_t analog_read;
        SPI_ReceiveData (SPI2, &analog_read, 1);

        // Wait until SPI is not busy
        while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

        SPI_PeripheralControl(SPI2, DISABLE);
    }

    return 0;
}
