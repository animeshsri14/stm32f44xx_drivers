/*
 * stm32f44xx_gpio_driver.c
 *
 *  Created on: Jun 19, 2025
 *      Author: animeshsrivastava
 */


#include "stm32f44x_gpio_driver.h"

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t temp=0; //temp. register

    GPIO_PeriClockControl (pGPIOHandle->pGPIOx, ENABLE);

    //1. configure the mode of gpio pin
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
        //the non interrupt mode
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << pGPIOHandle-> GPIO_PinConfig.GPIO_PinNumber);
        pGPIOHandle->pGPIOx->MODER |= temp; //setting
    }
    else {
    	//this part will code later . ( interrupt mode)
    				if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_FT) {
    					EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

    					EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    				}
    				else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_RT) {
    					EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

    					EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    				}
    				else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT ) {
    					EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

    					EXTI->RTSR |= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    				}
    				//2. configure the GPIO port selection in SYSCFG_EXTICR
    				//3. enable the exti interrupt delivery using IMR
    				EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    }

    temp = 0;

    //2. configure the speed
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
    pGPIOHandle->pGPIOx->OSPEEDR |= temp; //setting
    temp = 0;

    //3. configure the pupd settings
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
    pGPIOHandle->pGPIOx->PUPDR |= temp; //setting
    temp = 0;

    //4. configure the optype
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
    pGPIOHandle->pGPIOx->OTYPER |= temp; //setting

    //5. configure the alt functionality
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
    {
        uint8_t temp1,temp2;
        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) ); //clearing
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ) );
    }
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if(pGPIOx == GPIOA)
{
GPIOA_REG_RESET();
}
    else if (pGPIOx == GPIOB)
{
GPIOB_REG_RESET();
}
    else if (pGPIOx == GPIOC)
{
GPIOC_REG_RESET() ;
}
    else if (pGPIOx == GPIOD)
{
GPIOD_REG_RESET();
}
    else if (pGPIOx == GPIOE)
{
GPIOE_REG_RESET();
}
    else if (pGPIOx == GPIOF)
{
GPIOF_REG_RESET();
}
    else if (pGPIOx == GPIOG)
{
GPIOG_REG_RESET();
}
    else if (pGPIOx == GPIOH)
{
GPIOH_REG_RESET();
}
    else if (pGPIOx == GPIOI)
{
GPIOI_REG_RESET();
}
}

//gpio clock setup

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        if (pGPIOx == GPIOA) { GPIOA_PCLK_EN(); }
        else if (pGPIOx == GPIOB) { GPIOB_PCLK_EN(); }
        else if (pGPIOx == GPIOC) { GPIOC_PCLK_EN(); }
        else if (pGPIOx == GPIOD) { GPIOD_PCLK_EN(); }
        else if (pGPIOx == GPIOE) { GPIOE_PCLK_EN(); }
        else if (pGPIOx == GPIOF) { GPIOF_PCLK_EN(); }
        else if (pGPIOx == GPIOG) { GPIOG_PCLK_EN(); }
        else if (pGPIOx == GPIOH) { GPIOH_PCLK_EN(); }
        else if (pGPIOx == GPIOI) { GPIOI_PCLK_EN(); }
    } else {
        // optionally: handle DISABLE
    }
}

//Read and Write

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
uint8_t value;
value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001 ) ;
return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == SET) {
		// Write 1 to the output data register at the bit field corresponding to the pin
		pGPIOx->ODR |= (1 << PinNumber);
	} else {
		// Write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {
	pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	pGPIOx->ODR ^= (1<<PinNumber);
}


//IRQ Configuration and ISR handling

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi) {

}
void GPIO_IRQHandling(uint8_t PinNumber) {

}


