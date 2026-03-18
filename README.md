# STM32F446xx Peripheral Drivers

Bare-metal peripheral drivers for the STM32F446RE written from scratch in C — no HAL, no LL, no abstractions you didn't write yourself. The goal was to go all the way down to register level: read the reference manual, map the registers into structs, and build a clean driver API on top of that. Three peripherals are covered so far: **GPIO**, **SPI**, and **I2C**.

---

## Background

Most embedded courses and tutorials hand you STM32CubeHAL and call it a day. That's fine for shipping products, but it leaves a gap in understanding what's actually happening at the silicon level. This project started as a way to close that gap — to understand what `HAL_GPIO_WritePin` is *really doing*, why you need to clear the ADDR flag in I2C, what the difference between TXE and BTF is, and why SPI needs that dummy read after sending a command.

The target board is the **STM32F446RE Nucleo-64**. The reference manual for the STM32F446xx (RM0390) and the Cortex-M4 generic user guide (PM0214) are the primary sources of truth here.

---

## Development Environment

| Tool | Details |
|---|---|
| **IDE** | STM32CubeIDE 1.x (Eclipse-based, built-in GDB OpenOCD debug) |
| **Toolchain** | arm-none-eabi-gcc (bundled with CubeIDE) |
| **Debug** | ST-Link V2 on-board, OpenOCD, semi-hosting for printf over SWO |
| **Target MCU** | STM32F446RET6 — Cortex-M4 @ up to 180 MHz, 512 KB Flash, 128 KB SRAM |
| **Board** | NUCLEO-F446RE |
| **Linker scripts** | `STM32F446RETX_FLASH.ld` (default) and `STM32F446RETX_RAM.ld` (load to SRAM) |

Semi-hosting is used in the I2C receive test to get `printf` output over the debugger without a UART. To enable it: in STM32CubeIDE go to **Debug Configurations → Startup → Run Commands** and add `monitor arm semihosting enable`.

---

## Repository Layout

```
stm32f44xx_drivers/
├── drivers/
│   ├── Inc/
│   │   ├── stm32f44xx.h              # MCU header: base addresses, register structs, macros
│   │   ├── stm32f44x_gpio_driver.h   # GPIO driver API and config definitions
│   │   ├── stm32f44xx_spi_driver.h   # SPI driver API and config definitions
│   │   └── stm32f44xx_i2c_driver.h   # I2C driver API and config definitions
│   └── src/
│       ├── stm32f44xx_gpio_driver.c  # GPIO driver implementation
│       ├── stm32f44x_spi_driver.c    # SPI driver implementation
│       └── stm32f44xx_i2c_driver.c   # I2C driver implementation (interrupt-capable)
├── Src/
    ├── led_toggle.c                  # Basic GPIO output test
    ├── test_spi.c                    # SPI loopback / basic send
    ├── arduino_send_data.c           # SPI master → Arduino slave, sends a string
    ├── spi_cmd_handling.c            # SPI command protocol with ACK/NACK handshake
    ├── i2c_master_tx_testing.c       # I2C master transmit to a slave at 0x68
    ├── i2c_master_rx_testing.c       # I2C master send command → receive response
    ├── syscalls.c                    # Newlib syscall stubs (semi-hosting support)
    └── sysmem.c                      # Heap memory implementation

```

Only one `Src/` file should be active (included in the build) at a time — they each define their own `main()`. Exclude the others from the build in CubeIDE by right-clicking → **Resource Configurations → Exclude from Build**.

---

## MCU Header (`stm32f44xx.h`)

Everything that describes the hardware lives here. No CMSIS, no vendor headers — these were written by hand against the reference manual.

### Memory Map

```c
#define FLASH_BASEADDR    0x08000000U
#define SRAM1_BASEADDR    0x20000000U
#define SRAM2_BASEADDR    0x2001C000U
#define ROM               0x1FFF0000U  // System memory (bootloader)

#define APB1_BASE         0x40000000U
#define APB2_BASE         0x40010000U
#define AHB1_BASE         0x40020000U
#define AHB2_BASE         0x50000000U
```

All peripheral base addresses (GPIOx, SPIx, I2Cx, USARTx, TIMx, DMA, EXTI, SYSCFG, RTC, ADC, CRC, PWR) are defined here.

### Register Definition Structs

Peripherals are accessed through C structs mapped directly onto physical memory:

- `GPIO_RegDef_t` — MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2]
- `RCC_RegDef_t` — Full RCC register set (CR through DCKCFGR2)
- `EXTI_RegDef_t` — IMR, EMR, RTSR, FTSR, SWIER, PR
- `SPI_RegDef_t` — CR1, CR2, SR, DR, CRCPR, RXCRCR, TXCRCR, I2SCFGR, I2SPR
- `I2C_RegDef_t` — CR1, CR2, OAR1, OAR2, DR, SR1, SR2, CCR, TRISE, FLTR

Peripheral instances are accessed like:
```c
#define GPIOA   ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define SPI2    ((SPI_RegDef_t*)  SPI2_BASEADDR)
#define I2C1    ((I2C_RegDef_t*)  I2C1_BASEADDR)
#define RCC     ((RCC_RegDef_t*)  0x40023800U)
```

### Clock Control Macros

Clock enable/disable macros for every peripheral, operating on the RCC registers:

```c
// Enable
GPIOA_PCLK_EN()    // RCC->AHB1ENR |= (1 << 0)
SPI2_PCLK_EN()     // RCC->APB1ENR |= (1 << 14)
I2C1_PCLK_EN()     // RCC->APB1ENR |= (1 << 21)
SYSCFG_PCLK_EN()   // RCC->APB2ENR |= (1 << 14)

// Disable (same pattern with &= ~)
GPIOA_PCLK_DI()
SPI1_PCLK_DI()
I2C3_PCLK_DI()

// Peripheral reset (set then clear the reset bit — do-while(0) idiom)
GPIOA_REG_RESET()  // RCC->AHB1RSTR pulse on bit 0
```

---

## GPIO Driver

**Files:** `drivers/Inc/stm32f44x_gpio_driver.h`, `drivers/src/stm32f44xx_gpio_driver.c`

### Configuration Structures

```c
typedef struct {
    uint8_t GPIO_PinNumber;       // GPIO_PIN_NO_0 .. GPIO_PIN_NO_15
    uint8_t GPIO_PinMode;         // Input / Output / AltFn / Analog / Interrupt
    uint8_t GPIO_PinSpeed;        // Low / Medium / Fast / High
    uint8_t GPIO_PinPuPdControl;  // No pull / Pull-up / Pull-down
    uint8_t GPIO_PinOPType;       // Push-pull / Open-drain
    uint8_t GPIO_PinAltFunMode;   // AF0–AF15 (used when Mode = ALTFN)
} GPIO_PinConfig_t;

typedef struct {
    GPIO_RegDef_t    *pGPIOx;        // Pointer to GPIOx base (GPIOA–GPIOI)
    GPIO_PinConfig_t  GPIO_PinConfig;
} GPIO_Handle_t;
```

### Mode Options

| Macro | Value | Description |
|---|---|---|
| `GPIO_MODE_IN` | 0 | Digital input |
| `GPIO_MODE_OUT` | 1 | Digital output |
| `GPIO_MODE_ALTFN` | 2 | Alternate function (SPI, I2C, UART, etc.) |
| `GPIO_MODE_ANALOG` | 3 | Analog (ADC / DAC) |
| `GPIO_MODE_IT_FT` | 4 | Interrupt on falling edge |
| `GPIO_MODE_IT_RT` | 5 | Interrupt on rising edge |
| `GPIO_MODE_IT_RFT` | 6 | Interrupt on both edges |

### Output Type Options

| Macro | Description |
|---|---|
| `GPIO_OP_TYPE_PP` | Push-pull |
| `GPIO_OP_TYPE_OD` | Open-drain (required for I2C lines) |

### Speed Options

`GPIO_SPEED_LOW`, `GPIO_SPEED_MEDIUM`, `GPIO_SPEED_FAST`, `GPIO_SPEED_HIGH`

### Pull-up/Pull-down Options

`GPIO_NO_PUPD`, `GPIO_PIN_PU`, `GPIO_PIN_PD`

### API Reference

```c
// Initialization
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);  // Uses RCC reset register
```

`GPIO_Init` handles all five configuration steps internally: mode (including EXTI setup for interrupt modes), speed, pull-up/down, output type, and alternate function register (AFR[0] for pins 0–7, AFR[1] for pins 8–15). The peripheral clock is enabled automatically at the start of `GPIO_Init`.

```c
// Read
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

// Write
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);  // XOR on ODR

// Interrupt
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);  // Clears the EXTI pending register
```

### Usage Example — LED Toggle

```c
GPIO_Handle_t GpioLed;
GpioLed.pGPIOx = GPIOA;
GpioLed.GPIO_PinConfig.GPIO_PinNumber     = GPIO_PIN_NO_5;
GpioLed.GPIO_PinConfig.GPIO_PinMode       = GPIO_MODE_OUT;
GpioLed.GPIO_PinConfig.GPIO_PinSpeed      = GPIO_SPEED_FAST;
GpioLed.GPIO_PinConfig.GPIO_PinOPType     = GPIO_OP_TYPE_PP;
GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

GPIO_Init(&GpioLed);

while (1) {
    GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
    for (volatile int i = 0; i < 500000; i++);
}
```

---

## SPI Driver

**Files:** `drivers/Inc/stm32f44xx_spi_driver.h`, `drivers/src/stm32f44x_spi_driver.c`

Supports SPI1–SPI4. The driver handles full-duplex, half-duplex, and simplex (RX-only) configurations. Transfers are blocking (polling-based) — the interrupt hooks are declared but the full interrupt-driven path isn't wired yet.

### Configuration Structures

```c
typedef struct {
    uint8_t SPI_DeviceMode;  // Master or Slave
    uint8_t SPI_BusConfig;   // Full-duplex / Half-duplex / Simplex RX
    uint8_t SPI_SclkSpeed;   // Clock divide factor
    uint8_t SPI_DFF;         // 8-bit or 16-bit frames
    uint8_t SPI_CPOL;        // Clock polarity (idle low / idle high)
    uint8_t SPI_CPHA;        // Clock phase (sample on 1st / 2nd edge)
    uint8_t SPI_SSM;         // Software or hardware slave management
} SPI_Config_t;

typedef struct {
    SPI_RegDef_t *pSPIx;
    SPI_Config_t  SPIConfig;
} SPI_Handle_t;
```

### Configuration Options

**Device Mode**

| Macro | Description |
|---|---|
| `SPI_DEVICE_MODE_MASTER` | This device drives SCLK |
| `SPI_DEVICE_MODE_SLAVE` | SCLK is driven by the master |

**Bus Configuration**

| Macro | Description |
|---|---|
| `SPI_BUS_CONFIG_FD` | Full-duplex (MOSI + MISO simultaneously) |
| `SPI_BUS_CONFIG_HD` | Half-duplex (single bidirectional data line) |
| `SPI_BUS_CONFIG_SIMPLEX_RXONLY` | MISO only, receive-only mode |

**Clock Speed (baud rate prescaler)**

These divide the APB clock to derive SCLK. On APB2 (SPI1/SPI4) with 16 MHz HSI, `DIV8` gives 2 MHz.

| Macro | Divider |
|---|---|
| `SPI_SCLK_SPEED_DIV2` | ÷2 |
| `SPI_SCLK_SPEED_DIV4` | ÷4 |
| `SPI_SCLK_SPEED_DIV8` | ÷8 |
| `SPI_SCLK_SPEED_DIV16` | ÷16 |
| `SPI_SCLK_SPEED_DIV32` | ÷32 |
| `SPI_SCLK_SPEED_DIV64` | ÷64 |
| `SPI_SCLK_SPEED_DIV128` | ÷128 |
| `SPI_SCLK_SPEED_DIV256` | ÷256 |

**Data Frame Format:** `SPI_DFF_8BITS` / `SPI_DFF_16BITS`

**Clock Polarity (CPOL):** `SPI_CPOL_LOW` (idle = 0) / `SPI_CPOL_HIGH` (idle = 1)

**Clock Phase (CPHA):** `SPI_CPHA_LOW` (sample on leading edge) / `SPI_CPHA_HIGH` (sample on trailing edge)

**Slave Select Management:** `SPI_SSM_EN` (software SSM, use SSI bit) / `SPI_SSM_DI` (hardware NSS pin)

### Status Flags

```c
SPI_RXNE_FLAG   // Receive buffer not empty
SPI_TXE_FLAG    // Transmit buffer empty
SPI_BUSY_FLAG   // SPI peripheral is busy
SPI_OVR_FLAG    // Overrun error
SPI_MODF_FLAG   // Mode fault
SPI_CRCERR_FLAG // CRC error
SPI_FRE_FLAG    // Frame format error
```

### API Reference

```c
// Peripheral setup
void    SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void    SPI_Init(SPI_Handle_t *pSPIHandle);
void    SPI_DeInit(SPI_RegDef_t *pSPIx);

// Enable / disable SPE bit (must be set after init, cleared before reconfiguring)
void    SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

// Slave select control
void    SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);  // Hardware NSS output enable
void    SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);   // Software SSI bit

// Blocking data transfer
void    SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void    SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

// Status
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

// Interrupt configuration (for future interrupt-driven use)
void    SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void    SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void    SPI_IRQHandling(SPI_Handle_t *pHandle);
```

`SPI_SendData` polls `TXE` before each byte write. `SPI_ReceiveData` polls `RXNE` before each byte read. Neither returns until the full transfer is done.

### SPI Pin Mapping (SPI2 — used in test applications)

| SPI Signal | Pin |
|---|---|
| SCLK | PB13 |
| MISO | PB14 |
| MOSI | PB15 |
| NSS  | PB12 |
| Alternate Function | AF5 |

### Usage Example — Master Send

```c
SPI_Handle_t SPI2handle;
SPI2handle.pSPIx                    = SPI2;
SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
SPI2handle.SPIConfig.SPI_BusConfig  = SPI_BUS_CONFIG_FD;
SPI2handle.SPIConfig.SPI_SclkSpeed  = SPI_SCLK_SPEED_DIV8;
SPI2handle.SPIConfig.SPI_DFF        = SPI_DFF_8BITS;
SPI2handle.SPIConfig.SPI_CPOL       = SPI_CPOL_LOW;
SPI2handle.SPIConfig.SPI_CPHA       = SPI_CPHA_LOW;
SPI2handle.SPIConfig.SPI_SSM        = SPI_SSM_DI;  // Hardware NSS

SPI_PeriClockControl(SPI2, ENABLE);
SPI_Init(&SPI2handle);
SPI_SSOEConfig(SPI2, ENABLE);

// In main loop, after button press:
SPI_PeripheralControl(SPI2, ENABLE);
char msg[] = "Hello world";
uint8_t len = strlen(msg);
SPI_SendData(SPI2, &len, 1);
SPI_SendData(SPI2, (uint8_t*)msg, len);
while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
SPI_PeripheralControl(SPI2, DISABLE);
```

---

## I2C Driver

**Files:** `drivers/Inc/stm32f44xx_i2c_driver.h`, `drivers/src/stm32f44xx_i2c_driver.c`

This is the most complex driver in the project. It supports standard mode (100 kHz) and fast mode (up to 400 kHz), blocking master transmit and receive, slave transmit and receive, and interrupt-driven transfers with application callbacks. The CCR and TRISE values are computed at runtime from the actual APB1 clock, which itself is derived by reading and decoding the RCC_CFGR register.

### Configuration Structures

```c
typedef struct {
    uint32_t I2C_SCLSpeed;       // SCL clock speed in Hz
    uint8_t  I2C_DeviceAddress;  // Own 7-bit address (used in slave mode)
    uint8_t  I2C_AckControl;     // ACK enable/disable after receive
    uint16_t I2C_FMDutyCycle;    // Fast mode duty cycle (2 or 16/9)
} I2C_Config_t;

typedef struct {
    I2C_RegDef_t *pI2Cx;         // I2C1, I2C2, or I2C3
    I2C_Config_t  I2C_Config;
    uint8_t      *pTxBuffer;     // App Tx buffer pointer (interrupt mode)
    uint8_t      *pRxBuffer;     // App Rx buffer pointer (interrupt mode)
    uint32_t      TxLen;
    uint32_t      RxLen;
    uint8_t       TxRxState;     // I2C_READY / I2C_BUSY_IN_TX / I2C_BUSY_IN_RX
    uint8_t       DevAddr;       // Target slave address (interrupt mode)
    uint32_t      RxSize;
    uint8_t       Sr;            // Repeated start: I2C_ENABLE_SR / I2C_DISABLE_SR
} I2C_Handle_t;
```

### Configuration Options

**SCL Speed**

| Macro | Speed |
|---|---|
| `I2C_SCL_SPEED_SM` | 100,000 Hz (standard mode) |
| `I2C_SCL_SPEED_FM2K` | 200,000 Hz (fast mode) |
| `I2C_SCL_SPEED_FM4K` | 400,000 Hz (fast mode) |

**ACK Control:** `I2C_ACK_ENABLE` / `I2C_ACK_DISABLE`

**Fast Mode Duty Cycle**

| Macro | t_low / t_high ratio |
|---|---|
| `I2C_FM_DUTY_2` | 2:1 |
| `I2C_FM_DUTY_16_9` | 16:9 |

**Repeated Start:** `I2C_ENABLE_SR` (don't generate STOP — keep bus) / `I2C_DISABLE_SR` (generate STOP)

### Status Flags (SR1)

```c
I2C_FLAG_SB      // Start bit generated (master mode)
I2C_FLAG_ADDR    // Address sent (master) / matched (slave)
I2C_FLAG_BTF     // Byte transfer finished (both SR and DR empty/full)
I2C_FLAG_TXE     // Data register empty (transmitter)
I2C_FLAG_RXNE    // Data register not empty (receiver)
I2C_FLAG_STOPF   // Stop condition detected (slave mode)
I2C_FLAG_BERR    // Bus error
I2C_FLAG_ARLO    // Arbitration lost
I2C_FLAG_AF      // ACK failure
I2C_FLAG_OVR     // Overrun / underrun
I2C_FLAG_TIMEOUT // Timeout
```

### Application Event Codes (Callbacks)

These are passed to `I2C_ApplicationEventCallback` to notify the application layer:

| Code | Value | Meaning |
|---|---|---|
| `I2C_EV_TX_CMPLT` | 1 | Transmission complete |
| `I2C_EV_RX_CMPLT` | 2 | Reception complete |
| `I2C_EV_STOP` | 3 | STOP detected (slave mode) |
| `I2C_EV_DATA_REQ` | 4 | Master is requesting data from slave |
| `I2C_EV_DATA_RCV` | 5 | Slave received data from master |
| `I2C_ERROR_BERR` | 6 | Bus error |
| `I2C_ERROR_ARLO` | 7 | Arbitration lost |
| `I2C_ERROR_AF` | 8 | ACK failure |
| `I2C_ERROR_OVR` | 9 | Overrun / underrun |
| `I2C_ERROR_TIMEOUT` | 10 | Timeout |

### API Reference

```c
// Peripheral setup
void    I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void    I2C_Init(I2C_Handle_t *pI2CHandle);   // Computes CCR and TRISE from live APB1 clock
void    I2C_DeInit(I2C_RegDef_t *pI2Cx);
void    I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);  // PE bit

// Blocking master transfers
void    I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer,
                            uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void    I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
                               uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);

// Slave transfers (single byte, used inside callbacks)
void    I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);

// Interrupt-driven path — teardown helpers
void    I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void    I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

// Interrupt service routines (call from ISR)
void    I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);   // Event ISR
void    I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);   // Error ISR

// Interrupt configuration
void    I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void    I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

// Utility
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void    I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

// Application callback — implement this in your application code
void    I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);
```

### I2C Pin Mapping

| I2C Signal | Pin (I2C1) |
|---|---|
| SCL | PB6 |
| SDA | PB7 or PB9 |
| Alternate Function | AF4 |
| Output Type | **Open-drain** (required) |
| Pull | Pull-up (internal or external 4.7 kΩ) |

### I2C Initialization — What Actually Happens

`I2C_Init` does the following, in order:

1. Enables the peripheral clock via `I2C_PeriClockControl`.
2. Sets the ACK enable bit in CR1.
3. Programs CR2 with the peripheral clock frequency (APB1 / 1 MHz units), computed by `RCC_GetPCLK1Value()` which reads and decodes RCC_CFGR at runtime.
4. Programs OAR1 with the device's own address (shifted left by 1, bit 14 set per RM requirement).
5. Computes and writes CCR:
   - Standard mode: `CCR = PCLK1 / (2 × SCL_speed)`
   - Fast mode DUTY_2: `CCR = PCLK1 / (3 × SCL_speed)`
   - Fast mode DUTY_16_9: `CCR = PCLK1 / (25 × SCL_speed)`
6. Computes and writes TRISE:
   - Standard mode: `TRISE = (PCLK1_MHz) + 1`  → max rise time = 1000 ns
   - Fast mode: `TRISE = (PCLK1 × 300 / 1,000,000,000) + 1` → max rise time = 300 ns

### Usage Example — Master TX then RX with Repeated Start

```c
I2C_Handle_t I2C1Handle;
I2C1Handle.pI2Cx                      = I2C1;
I2C1Handle.I2C_Config.I2C_SCLSpeed    = I2C_SCL_SPEED_SM;
I2C1Handle.I2C_Config.I2C_DeviceAddress = 0x61;
I2C1Handle.I2C_Config.I2C_AckControl  = I2C_ACK_ENABLE;
I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;

I2C_Init(&I2C1Handle);
I2C_PeripheralControl(I2C1, ENABLE);
I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

// Send a command, then receive a response without releasing the bus in between
uint8_t cmd = 0x51;
I2C_MasterSendData(&I2C1Handle, &cmd, 1, 0x68, I2C_ENABLE_SR);

uint8_t len;
I2C_MasterReceiveData(&I2C1Handle, &len, 1, 0x68, I2C_ENABLE_SR);

uint8_t buf[32];
uint8_t data_cmd = 0x52;
I2C_MasterSendData(&I2C1Handle, &data_cmd, 1, 0x68, I2C_ENABLE_SR);
I2C_MasterReceiveData(&I2C1Handle, buf, len, 0x68, I2C_DISABLE_SR);
```

---

## Test Applications

### `led_toggle.c`

Basic GPIO output test. Toggles GPIOA pin 5 (the on-board LED on the Nucleo) in a software delay loop. Covers `GPIO_Init`, `GPIO_ToggleOutputPin`.

### `test_spi.c`

Minimal SPI send test, used during initial bringup to verify the SPI clock and data lines with a logic analyzer.

### `arduino_send_data.c`

SPI2 master sends a length-prefixed string to an Arduino running a compatible SPI slave sketch. The STM32 waits for a button press on PA0, pulls NSS low via SSOE hardware control, sends the length byte, then sends the data bytes, waits for BSY to clear, then disables SPI.

### `spi_cmd_handling.c`

A more realistic command protocol over SPI2, exchanging with an Arduino slave that implements an ACK/NACK handshake:

- Master sends a command byte.
- Master issues a dummy read to flush the shift register.
- Master sends a dummy byte to clock in the slave's ACK/NACK byte (`0xF5` = ACK).
- If ACK received, master sends command arguments.

Commands implemented:

| Command | Code | Arguments |
|---|---|---|
| `COMMAND_LED_CTRL` | `0x50` | Pin number, LED_ON/LED_OFF |
| `COMMAND_SENSOR_READ` | `0x51` | Analog pin number (0–4) |
| `COMMAND_LED_READ` | `0x52` | Pin number |
| `COMMAND_PRINT` | `0x53` | — |
| `COMMAND_ID_READ` | `0x54` | — |

### `i2c_master_tx_testing.c`

I2C1 master sends the string `"We are testing I2C master Tx\n"` to a slave at address `0x68` (e.g., an Arduino or RPi running a slave sketch) every time the button on PA0 is pressed. Uses a repeated start (`Sr = 1`) so the bus isn't released.

I2C1 pins: PB6 (SCL), PB9 (SDA), AF4, open-drain, internal pull-up.

### `i2c_master_rx_testing.c`

Two-phase read sequence from a slave at `0x68`:

1. Send command `0x51` → receive 1 byte (the length of data to follow).
2. Send command `0x52` → receive N bytes into a local buffer.
3. Print result via semi-hosting (`printf`).

Button on PC13 (the Nucleo user button). Semi-hosting enabled in debug config.

---

## Notes and Known Gaps

- The GPIO interrupt path (`GPIO_IRQConfig`, `GPIO_IRQHandling`) is declared and partially wired (EXTI FTSR/RTSR/IMR registers are set in `GPIO_Init`) but the NVIC-side configuration in `GPIO_IRQConfig` is not yet implemented.
- `I2C_DeInit` is declared but empty — peripheral reset via RCC is not yet connected.
- The SPI interrupt path (`SPI_IRQHandling`) is declared but not implemented.
- `RCC_GetPCLK1Value` only handles HSI (16 MHz) and HSE (8 MHz) clock sources. If PLL is used as the system clock, the PLL output calculation branch is left empty — this will return `SystemClk = 0` and produce wrong CCR/TRISE values for I2C.
- Only 7-bit I2C addressing is used. The `I2C_OAR1_ADDMODE` bit (10-bit addressing) is defined in `stm32f44xx.h` but not used anywhere.

---

## Reference

- STM32F446xx Reference Manual - RM0390
- STM32F446RE Datasheet
- ARM Cortex-M4 Generic User Guide — PM0214
- NUCLEO-F446RE User Manual — UM1724
