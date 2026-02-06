/*
 * arduino_send_data.c
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

    GPIOBtn.pGPIOx = GPIOA;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_Init(&GPIOBtn);
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

int main(void)
{
    char user_data[] = "Hello world";

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

        uint8_t dataLen = strlen(user_data);

        // Send string length
        SPI_SendData(SPI2, &dataLen, 1);

        // Send string data
        SPI_SendData(SPI2, (uint8_t *)user_data, strlen(user_data));

        // Wait until SPI is not busy
        while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

        SPI_PeripheralControl(SPI2, DISABLE);
    }

    return 0;
}
