/*
 * stm32f44xx.h
 *
 *  Created on: Jun 19, 2025
 *      Author: animeshsrivastava
 */

#ifndef INC_STM32F44XX_H_
#define INC_STM32F44XX_H_

#include <stdint.h>

#define FLASH_BASEADDR    0x08000000U
#define SRAM1_BASEADDR    0x20000000U
#define SRAM2_BASEADDR	  0x2001C000U
#define ROM				  0x1FFF0000U // System memory
#define SRAM			  SRAM1_BASEADDR

//Buses base address

#define PERIPHERAL_BASE  0x4000000U
#define APB1_BASE 		 PERIPHERAL_BASE
#define APB2_BASE		 0x40010000U
#define AHB1_BASE		 0x40020000U
#define AHB2_BASE		 0x50000000U

// GPIO Base Addresses (AHB1)
#define GPIOA_BASEADDR   0x40020000U
#define GPIOB_BASEADDR   0x40020400U
#define GPIOC_BASEADDR   0x40020800U
#define GPIOD_BASEADDR   0x40020C00U
#define GPIOE_BASEADDR   0x40021000U
#define GPIOF_BASEADDR   0x40021400U
#define GPIOG_BASEADDR   0x40021800U
#define GPIOH_BASEADDR   0x40021C00U
#define GPIOI_BASEADDR   0x40022000U

// SPI Base Addresses
#define SPI1_BASEADDR    0x40013000U
#define SPI2_BASEADDR    0x40003800U
#define SPI3_BASEADDR    0x40003C00U
#define SPI4_BASEADDR    0x40013400U

// USART Base Addresses
#define USART1_BASEADDR  0x40011000U
#define USART2_BASEADDR  0x40004400U
#define USART3_BASEADDR  0x40004800U
#define UART4_BASEADDR   0x40004C00U
#define UART5_BASEADDR   0x40005000U
#define USART6_BASEADDR  0x40011400U

// I2C Base Addresses
#define I2C1_BASEADDR    0x40005400U
#define I2C2_BASEADDR    0x40005800U
#define I2C3_BASEADDR    0x40005C00U

// EXTI (External Interrupt/Event Controller)
#define EXTI_BASEADDR    0x40013C00U

// SYSCFG (System Configuration Controller)
#define SYSCFG_BASEADDR  0x40013800U

// DMA Base Addresses
#define DMA1_BASEADDR    0x40026000U
#define DMA2_BASEADDR    0x40026400U

// ADC Common Base Address (ADC1, ADC2, ADC3 share a common interface)
#define ADC_BASEADDR     0x40012000U

// TIM Base Addresses (General purpose and advanced timers)
#define TIM1_BASEADDR    0x40010000U
#define TIM2_BASEADDR    0x40000000U
#define TIM3_BASEADDR    0x40000400U
#define TIM4_BASEADDR    0x40000800U
#define TIM5_BASEADDR    0x40000C00U
#define TIM6_BASEADDR    0x40001000U
#define TIM7_BASEADDR    0x40001400U
#define TIM8_BASEADDR    0x40010400U
#define TIM9_BASEADDR    0x40014000U
#define TIM10_BASEADDR   0x40014400U
#define TIM11_BASEADDR   0x40014800U
#define TIM12_BASEADDR   0x40001800U
#define TIM13_BASEADDR   0x40001C00U
#define TIM14_BASEADDR   0x40002000U

// SDIO
#define SDMMC_BASEADDR   0x40012C00U

// CRC
#define CRC_BASEADDR     0x40023000U

// Power Interface
#define PWR_BASEADDR     0x40007000U

// Backup and RTC
#define RTC_BASEADDR     0x40002800U

#define EXTI 	((EXTI_RegDef_t*) EXTI_BASEADDR)

#define GPIOA 	((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB 	((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC 	((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD 	((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE 	((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF 	((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG 	((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH 	((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI 	((GPIO_RegDef_t*) GPIOI_BASEADDR)

#define SPI1 	((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2 	((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3 	((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4 	((SPI_RegDef_t*)SPI4_BASEADDR)

#define I2C1 ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2 ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3 ((I2C_RegDef_t*)I2C3_BASEADDR)

//Clock Enable Macros for GPIOx peripherals

#define GPIOA_PCLK_EN()    ( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()    ( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()    ( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()    ( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()    ( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()    ( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()    ( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN()    ( RCC->AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN()    ( RCC->AHB1ENR |= (1 << 8) )

//Clock Enable Macros for I2Cx peripherals
#define I2C1_PCLK_EN()     ( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()     ( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()     ( RCC->APB1ENR |= (1 << 23) )


//Clock Enable Macros for SPIx peripherals

#define SPI1_PCLK_EN()     ( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()     ( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()     ( RCC->APB1ENR |= (1 << 15) )
#define SPI4_PCLK_EN()     ( RCC->APB2ENR |= (1 << 13) )

//Clock Enable Macros for USARTx peripherals

#define USART1_PCLK_EN()   ( RCC->APB2ENR |= (1 << 4) )
#define USART2_PCLK_EN()   ( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN()   ( RCC->APB1ENR |= (1 << 18) )
#define UART4_PCLK_EN()    ( RCC->APB1ENR |= (1 << 19) )
#define UART5_PCLK_EN()    ( RCC->APB1ENR |= (1 << 20) )
#define USART6_PCLK_EN()   ( RCC->APB2ENR |= (1 << 5) )

//Clock Enable Macro for SYSCFG peripheral

#define SYSCFG_PCLK_EN()   ( RCC->APB2ENR |= (1 << 14) )

//Clock Disable Macros for GPIOx peripherals

#define GPIOA_PCLK_DI()    ( RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()    ( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()    ( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()    ( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()    ( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI()    ( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI()    ( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI()    ( RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI()    ( RCC->AHB1ENR &= ~(1 << 8) )


//Clock Disable Macros for I2Cx peripherals

#define I2C1_PCLK_DI()     ( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI()     ( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI()     ( RCC->APB1ENR &= ~(1 << 23) )


//Clock Disable Macros for SPIx peripherals

#define SPI1_PCLK_DI()     ( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI()     ( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI()     ( RCC->APB1ENR &= ~(1 << 15) )
#define SPI4_PCLK_DI()     ( RCC->APB2ENR &= ~(1 << 13) )


//Clock Disable Macros for USARTx peripherals

#define USART1_PCLK_DI()   ( RCC->APB2ENR &= ~(1 << 4) )
#define USART2_PCLK_DI()   ( RCC->APB1ENR &= ~(1 << 17) )
#define USART3_PCLK_DI()   ( RCC->APB1ENR &= ~(1 << 18) )
#define UART4_PCLK_DI()    ( RCC->APB1ENR &= ~(1 << 19) )
#define UART5_PCLK_DI()    ( RCC->APB1ENR &= ~(1 << 20) )
#define USART6_PCLK_DI()   ( RCC->APB2ENR &= ~(1 << 5) )


//Clock Disable Macro for SYSCFG peripheral

#define SYSCFG_PCLK_DI()   ( RCC->APB2ENR &= ~(1 << 14) )

//Reset GPIOx peripherals
#define GPIOA_REG_RESET()   do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()   do { (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()   do { (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()   do { (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()   do { (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET()   do { (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET()   do { (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET()   do { (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while(0)
#define GPIOI_REG_RESET()   do { (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); } while(0)


#define ENABLE   	 1
#define DISABLE   	 0
#define SET       	 ENABLE
#define RESET      	 DISABLE
#define FLAG_RESET	 RESET
#define FLAG_SET 	 SET

#define SPI_CR1_CPHA       0   // Clock Phase
#define SPI_CR1_CPOL       1   // Clock Polarity
#define SPI_CR1_MSTR       2   // Master Selection
#define SPI_CR1_BR         3   // Baud Rate Control (3:5)
#define SPI_CR1_SPE        6   // SPI Enable
#define SPI_CR1_LSBFIRST   7   // Frame Format (LSB/MSB first)
#define SPI_CR1_SSI        8   // Internal Slave Select
#define SPI_CR1_SSM        9   // Software Slave Management
#define SPI_CR1_RXONLY     10  // Receive Only
#define SPI_CR1_DFF        11  // Data Frame Format (8/16 bit)
#define SPI_CR1_CRCNEXT    12  // Transmit CRC Next
#define SPI_CR1_CRCEN      13  // Hardware CRC Enable
#define SPI_CR1_BIDIOE     14  // Output Enable in Bidirectional Mode
#define SPI_CR1_BIDIMODE   15  // Bidirectional Data Mode Enable

#define SPI_CR2_RXDMAEN    0   // Rx Buffer DMA Enable
#define SPI_CR2_TXDMAEN    1   // Tx Buffer DMA Enable
#define SPI_CR2_SSOE       2   // SS Output Enable
#define SPI_CR2_FRF        4   // Frame Format
#define SPI_CR2_ERRIE      5   // Error Interrupt Enable
#define SPI_CR2_RXNEIE     6   // RX buffer Not Empty Interrupt Enable
#define SPI_CR2_TXEIE      7   // Tx buffer Empty Interrupt Enable


// SPI Status Register Bit Positions
#define SPI_SR_RXNE     0   // Receive buffer not empty
#define SPI_SR_TXE      1   // Transmit buffer empty
#define SPI_SR_CHSIDE   2   // Channel side (I2S)
#define SPI_SR_UDR      3   // Underrun error (I2S)
#define SPI_SR_CRCERR   4   // CRC error flag (cleared by writing 0)
#define SPI_SR_MODF     5   // Mode fault
#define SPI_SR_OVR      6   // Overrun flag
#define SPI_SR_BSY      7   // Busy flag
#define SPI_SR_FRE      8   // Frame format error

#define I2C_CR1_PE               (1U << 0)   // Peripheral Enable
#define I2C_CR1_SMBUS            (1U << 1)   // SMBus mode
#define I2C_CR1_SMBTYPE          (1U << 3)   // SMBus type
#define I2C_CR1_ENARP            (1U << 4)   // ARP enable
#define I2C_CR1_ENPEC            (1U << 5)   // PEC enable
#define I2C_CR1_ENGC             (1U << 6)   // General call enable
#define I2C_CR1_NOSTRETCH        (1U << 7)   // Clock stretching disable
#define I2C_CR1_START            (1 << 8)   // Start generation
#define I2C_CR1_STOP             (1U << 9)   // Stop generation
#define I2C_CR1_ACK              (1U << 10)  // Acknowledge enable
#define I2C_CR1_POS              (1U << 11)  // Acknowledge/PEC position
#define I2C_CR1_PEC              (1U << 12)  // PEC request
#define I2C_CR1_ALERT            (1U << 13)  // SMBus alert
#define I2C_CR1_SWRST            (1U << 15)  // Software reset

#define I2C_CR2_FREQ_Pos         0
#define I2C_CR2_FREQ_Msk         (0x3F << I2C_CR2_FREQ_Pos) // Peripheral clock frequency
#define I2C_CR2_ITERREN          (1U << 8)   // Error interrupt enable
#define I2C_CR2_ITEVTEN          (1U << 9)   // Event interrupt enable
#define I2C_CR2_ITBUFEN          (1U << 10)  // Buffer interrupt enable
#define I2C_CR2_DMAEN            (1U << 11)  // DMA requests enable
#define I2C_CR2_LAST             (1U << 12)  // DMA last transfer

#define I2C_OAR1_ADD0            (1U << 0)   // Interface address bit 0 (7/10-bit)
#define I2C_OAR1_ADD71_Pos       1
#define I2C_OAR1_ADD71_Msk       (0x7F << I2C_OAR1_ADD71_Pos)  // Interface address bits [7:1]
#define I2C_OAR1_ADDMODE         (1U << 15)  // Addressing mode (0: 7-bit, 1: 10-bit)

#define I2C_OAR2_ENDUAL          (1U << 0)   // Dual addressing mode enable
#define I2C_OAR2_ADD2_Pos        1
#define I2C_OAR2_ADD2_Msk        (0x7F << I2C_OAR2_ADD2_Pos)   // Interface address 2

#define I2C_SR1_SB               (1U << 0)   // Start bit (Master mode)
#define I2C_SR1_ADDR             (1U << 1)   // Address sent/matched
#define I2C_SR1_BTF              (1U << 2)   // Byte transfer finished
#define I2C_SR1_ADD10            (1U << 3)   // 10-bit header sent
#define I2C_SR1_STOPF            (1U << 4)   // Stop detection
#define I2C_SR1_RXNE             (1U << 6)   // Data register not empty (receivers)
#define I2C_SR1_TXE              (1U << 7)   // Data register empty (transmitters)
#define I2C_SR1_BERR             (1U << 8)   // Bus error
#define I2C_SR1_ARLO             (1U << 9)   // Arbitration lost
#define I2C_SR1_AF               (1U << 10)  // Acknowledge failure
#define I2C_SR1_OVR              (1U << 11)  // Overrun/Underrun
#define I2C_SR1_PECERR           (1U << 12)  // PEC error in reception
#define I2C_SR1_TIMEOUT          (1U << 14)  // Timeout or Tlow error
#define I2C_SR1_SMBALERT         (1U << 15)  // SMBus alert

#define I2C_SR2_MSL              (1U << 0)   // Master/slave
#define I2C_SR2_BUSY             (1U << 1)   // Bus busy
#define I2C_SR2_TRA              (1U << 2)   // Transmitter/receiver
#define I2C_SR2_GENCALL          (1U << 4)   // General call address received
#define I2C_SR2_SMBDEFAULT       (1U << 5)   // SMBus device default address
#define I2C_SR2_SMBHOST          (1U << 6)   // SMBus host header
#define I2C_SR2_DUALF            (1U << 7)   // Dual flag (OAR2 matched)
#define I2C_SR2_PEC_Pos          8
#define I2C_SR2_PEC_Msk          (0xFF << I2C_SR2_PEC_Pos)   // Packet error checking

#define I2C_CCR_CCR_Pos          0
#define I2C_CCR_CCR_Msk          (0xFFF << I2C_CCR_CCR_Pos)   // Clock control bits
#define I2C_CCR_DUTY             (1U << 14)  // Fast mode duty cycle
#define I2C_CCR_FS               (1U << 15)  // I2C master mode selection (0: Standard, 1: Fast)

#define I2C_TRISE_TRISE_Pos      0
#define I2C_TRISE_TRISE_Msk      (0x3F << I2C_TRISE_TRISE_Pos)  // Maximum rise time in Fm/Sm mode

#define I2C_READY               0
#define I2C_BUSY_IN_RX          1
#define I2C_BUSY_IN_TX          2

#define I2C_DISABLE_SR          0
#define I2C_ENABLE_SR           1

#define I2C_ACK_ENABLE          1
#define I2C_ACK_DISABLE         0

#define I2C_SCL_SPEED_SM        100000  // standard mode 100kHz
#define I2C_SCL_SPEED_FM4K      400000  // fast mode 400kHz
#define I2C_SCL_SPEED_FM2K      200000  // fast mode 200kHz

#define I2C_FM_DUTY_2           0
#define I2C_FM_DUTY_16_9        1


#define I2C_EV_TX_CMPLT         1  // Master/slave transmit complete
#define I2C_EV_RX_CMPLT         2  // Master/slave receive complete
#define I2C_EV_STOP             3  // Stop condition detected (slave mode)
#define I2C_EV_DATA_REQ         4  // Master requests data (slave transmit)
#define I2C_EV_DATA_RCV         5  // Data received from master (slave receive)

#define I2C_ERROR_BERR          6  // Bus error
#define I2C_ERROR_ARLO          7  // Arbitration lost
#define I2C_ERROR_AF            8  // ACK failure
#define I2C_ERROR_OVR           9  // Overrun/underrun
#define I2C_ERROR_TIMEOUT       10 // Timeout error

#define ENABLE                  1
#define DISABLE                 0


//GPIO peripheral register definition structures

typedef struct
{
    volatile uint32_t MODER;    // GPIO port mode register          (offset: 0x00)
    volatile uint32_t OTYPER;   // GPIO port output type register   (offset: 0x04)
    volatile uint32_t OSPEEDR;  // GPIO port output speed register  (offset: 0x08)
    volatile uint32_t PUPDR;    // GPIO port pull-up/pull-down reg  (offset: 0x0C)
    volatile uint32_t IDR;      // GPIO port input data register    (offset: 0x10)
    volatile uint32_t ODR;      // GPIO port output data register   (offset: 0x14)
    volatile uint32_t BSRR;     // GPIO port bit set/reset register (offset: 0x18)
    volatile uint32_t LCKR;     // GPIO port configuration lock reg (offset: 0x1C)
    volatile uint32_t AFR[2];   // GPIO alternate function registers (low/high) (0x20, 0x24)
} GPIO_RegDef_t;

// RCC (Reset and Clock Control)
#define RCC     ((RCC_RegDef_t*)0x40023800U)

//RCC register definition structure

typedef struct
{
    volatile uint32_t CR;            // 0x00: Clock control register
    volatile uint32_t PLLCFGR;       // 0x04: PLL configuration register
    volatile uint32_t CFGR;          // 0x08: Clock configuration register
    volatile uint32_t CIR;           // 0x0C: Clock interrupt register
    volatile uint32_t AHB1RSTR;      // 0x10: AHB1 peripheral reset register
    volatile uint32_t AHB2RSTR;      // 0x14: AHB2 peripheral reset register
    volatile uint32_t AHB3RSTR;      // 0x18: AHB3 peripheral reset register
    uint32_t RESERVED0;     // 0x1C: Reserved
    volatile uint32_t APB1RSTR;      // 0x20: APB1 peripheral reset register
    volatile uint32_t APB2RSTR;      // 0x24: APB2 peripheral reset register
    uint32_t RESERVED1[2];  // 0x28–0x2C: Reserved
    volatile uint32_t AHB1ENR;       // 0x30: AHB1 peripheral clock enable register
    volatile uint32_t AHB2ENR;       // 0x34: AHB2 peripheral clock enable register
    volatile uint32_t AHB3ENR;       // 0x38: AHB3 peripheral clock enable register
    uint32_t RESERVED2;     // 0x3C: Reserved
    volatile uint32_t APB1ENR;       // 0x40: APB1 peripheral clock enable register
    volatile uint32_t APB2ENR;       // 0x44: APB2 peripheral clock enable register
    uint32_t RESERVED3[2];  // 0x48–0x4C: Reserved
    volatile uint32_t AHB1LPENR;     // 0x50: AHB1 low power enable register
    volatile uint32_t AHB2LPENR;     // 0x54: AHB2 low power enable register
    volatile uint32_t AHB3LPENR;     // 0x58: AHB3 low power enable register
    uint32_t RESERVED4;     // 0x5C: Reserved
    volatile uint32_t APB1LPENR;     // 0x60: APB1 low power enable register
    volatile uint32_t APB2LPENR;     // 0x64: APB2 low power enable register
    uint32_t RESERVED5[2];  // 0x68–0x6C: Reserved
    volatile uint32_t BDCR;          // 0x70: Backup domain control register
    volatile uint32_t CSR;           // 0x74: Clock control & status register
    uint32_t RESERVED6[2];  // 0x78–0x7C: Reserved
    volatile uint32_t SSCGR;         // 0x80: Spread spectrum clock generation register
    volatile uint32_t PLLI2SCFGR;    // 0x84: PLLI2S configuration register
    volatile uint32_t PLLSAICFGR;    // 0x88: PLLSAI configuration register
    volatile uint32_t DCKCFGR;       // 0x8C: Dedicated Clocks configuration register
    volatile uint32_t CKGATENR;      // 0x90: Clocks gated enable register
    volatile uint32_t DCKCFGR2;      // 0x94: Dedicated Clocks configuration register 2
} RCC_RegDef_t;

//EXTI peripheral register definition structures

typedef struct
{
    volatile uint32_t IMR;
    volatile uint32_t EMR;
    volatile uint32_t RTSR;
    volatile uint32_t FTSR;
    volatile uint32_t SWIER;
    volatile uint32_t PR;
} EXTI_RegDef_t;

//SPI peripheral register definition structure

typedef struct
{
    volatile uint32_t CR1;      // Control Register 1
    volatile uint32_t CR2;      // Control Register 2
    volatile uint32_t SR;       // Status Register
    volatile uint32_t DR;       // Data Register
    volatile uint32_t CRCPR;    // CRC Polynomial Register
    volatile uint32_t RXCRCR;   // RX CRC Register
    volatile uint32_t TXCRCR;   // TX CRC Register
    volatile uint32_t I2SCFGR;  // I2S Configuration Register
    volatile uint32_t I2SPR;    // I2S Prescaler Register
} SPI_RegDef_t;

// Structure representing the memory-mapped registers of an I2C peripheral
typedef struct
{
	volatile uint32_t CR1;    // Control Register 1: enables the peripheral, PE bit, ACK, STOP, START generation
	volatile uint32_t CR2;    // Control Register 2: peripheral clock frequency, interrupts, DMA
	volatile uint32_t OAR1;   // Own Address Register 1: primary device address
	volatile uint32_t OAR2;   // Own Address Register 2: secondary device address (if dual addressing)
	volatile uint32_t DR;     // Data Register: holds the data to be transmitted/received
	volatile uint32_t SR1;    // Status Register 1: event flags, error flags, start/stop condition detected
	volatile uint32_t SR2;    // Status Register 2: bus status, slave/host mode, general call, PEC
	volatile uint32_t CCR;    // Clock Control Register: sets the SCL clock speed
	volatile uint32_t TRISE;  // TRISE Register: maximum rise time configuration
	volatile uint32_t FLTR;   // Filter Register: digital and analog noise filter configuration
} I2C_RegDef_t;





#include "stm32f44x_gpio_driver.h"
#include "stm32f44xx_spi_driver.h"
#include "stm32f44xx_i2c_driver.h"

#endif /* INC_STM32F44XX_H_ */


