#include "stm32f44xx.h"

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi )
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI2)
		{
			SPI3_PCLK_EN();
		}else if(pSPIx == SPI3)
		{
			SPI4_PCLK_EN();
		}
	}else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}

}


void SPI_Init(SPI_Handle_t *pSPIHandle) {
    uint32_t tempreg = 0;

    tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

    if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
        tempreg &= ~(1 << SPI_CR1_BIDIMODE);  // BIDIMODE = 0
    } else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
        tempreg |= (1 << SPI_CR1_BIDIMODE);   // BIDIMODE = 1
    } else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
        tempreg &= ~(1 << SPI_CR1_BIDIMODE);
        tempreg |= (1 << SPI_CR1_RXONLY);
    }

    tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;
    tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;
    tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;
    tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;
    tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

    pSPIHandle->pSPIx->CR1 = tempreg;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx) {
    // ToDo: Implement register reset macros if needed
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) {
    if (pSPIx->SR & FlagName)
        return FLAG_SET;
    return FLAG_RESET;
}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len) {
    while (len > 0) {
        while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

        if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
            // 16-bit
            pSPIx->DR = *((uint16_t *)pTxBuffer);
            pTxBuffer += 2;
            len -= 2;
        } else {
            // 8-bit
            pSPIx->DR = *pTxBuffer;
            pTxBuffer++;
            len--;
        }
    }
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len){
	 while (len > 0) {
	        while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

	        if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
	            // 16-bit
	        	*((uint16_t*)pRxBuffer) = pSPIx->DR ;
	        	pRxBuffer += 2;
	            len -= 2;
	        } else {
	            // 8-bit
	        	*((uint16_t*)pRxBuffer) = pSPIx->DR ;
	            pRxBuffer++;
	            len--;
	        }
	    }
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
    if (EnOrDi == ENABLE) {
        pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    } else {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
    if (EnOrDi == ENABLE) {
        pSPIx->CR1 |= (1 << SPI_CR1_SSI);
    } else {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
    }
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
    if (EnOrDi == ENABLE) {
        pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
    } else {
        pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
    }
}
