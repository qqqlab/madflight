/*==============================================================================
Generated on: 2025-03-18 18:40:43.357163
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: REVOLTOSD
Manufacturer ID: FLON

//copy this line to madflight.ino to use this flight controller
#include <madflight_board_FLON-REVOLTOSD.h>

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out.
==============================================================================*/

#define HAL_BOARD_NAME "BETAFLIGHT-FLON-REVOLTOSD"
#define HAL_MCU "STM32F405"

#define MADFLIGHT_BOARD R""(
imu_gizmo MPU6500 // #define USE_GYRO_SPI_MPU6500
// resource BEEPER 1 B04
pin_out0 PB0 // resource MOTOR 1 B00
pin_out1 PB1 // resource MOTOR 2 B01
pin_out2 PA3 // resource MOTOR 3 A03
pin_out3 PA2 // resource MOTOR 4 A02
// resource LED_STRIP 1 B06
pin_ser0_tx PA9 // resource SERIAL_TX 1 A09
pin_ser2_tx PB10 // resource SERIAL_TX 3 B10
pin_ser3_tx PA0 // resource SERIAL_TX 4 A00
pin_ser5_tx PC6 // resource SERIAL_TX 6 C06
pin_ser0_rx PA10 // resource SERIAL_RX 1 A10
pin_ser2_rx PB11 // resource SERIAL_RX 3 B11
pin_ser3_rx PA1 // resource SERIAL_RX 4 A01
pin_ser5_rx PC7 // resource SERIAL_RX 6 C07
pin_ser0_inv PC0 // resource INVERTER 1 C00
pin_i2c0_scl PB8 // resource I2C_SCL 1 B08
pin_i2c0_sda PB9 // resource I2C_SDA 1 B09
pin_led PB5 // resource LED 1 B05
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi2_sclk PC10 // resource SPI_SCK 3 C10
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PB14 // resource SPI_MISO 2 B14
pin_spi2_miso PC11 // resource SPI_MISO 3 C11
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi1_mosi PB15 // resource SPI_MOSI 2 B15
pin_spi2_mosi PC12 // resource SPI_MOSI 3 C12
// resource ESCSERIAL 1 C06
pin_bat_v PC2 // resource ADC_BATT 1 C02
bat_gizmo ADC
pin_bat_i PC1 // resource ADC_CURR 1 C01
bat_gizmo ADC
// resource FLASH_CS 1 B03
// resource OSD_CS 1 D02
pin_imu_int PC4 // resource GYRO_EXTI 1 C04
pin_imu_cs PA4 // resource GYRO_CS 1 A04
// resource USB_DETECT 1 C05
// set dshot_burst = ON
// set system_hse_mhz = 8
// set max7456_spi_bus = 2
// set dashboard_i2c_bus = 1
// set flash_spi_bus = 2
// set blackbox_device = SPIFLASH
// set gyro_1_bustype = SPI
imu_spi_bus 0 // set gyro_1_spibus = 1
)"" //end of MADFLIGHT_BOARD


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F405 (S405) 4.2.0 Feb  2 2020 / 16:57:58 (norevision) MSP API: 1.43

#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

board_name REVOLTOSD
manufacturer_id FLON

# resources
resource BEEPER 1 B04
resource MOTOR 1 B00
resource MOTOR 2 B01
resource MOTOR 3 A03
resource MOTOR 4 A02
resource LED_STRIP 1 B06
resource SERIAL_TX 1 A09
resource SERIAL_TX 3 B10
resource SERIAL_TX 4 A00
resource SERIAL_TX 6 C06
resource SERIAL_RX 1 A10
resource SERIAL_RX 3 B11
resource SERIAL_RX 4 A01
resource SERIAL_RX 6 C07
resource INVERTER 1 C00
resource I2C_SCL 1 B08
resource I2C_SDA 1 B09
resource LED 1 B05
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 C10
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 C11
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 C12
resource ESCSERIAL 1 C06
resource ADC_BATT 1 C02
resource ADC_CURR 1 C01
resource FLASH_CS 1 B03
resource OSD_CS 1 D02
resource GYRO_EXTI 1 C04
resource GYRO_CS 1 A04
resource USB_DETECT 1 C05

# timer
timer B09 AF3
# pin B09: TIM11 CH1 (AF3)
timer B00 AF3
# pin B00: TIM8 CH2N (AF3)
timer B01 AF3
# pin B01: TIM8 CH3N (AF3)
timer A03 AF1
# pin A03: TIM2 CH4 (AF1)
timer A02 AF1
# pin A02: TIM2 CH3 (AF1)
timer B06 AF2
# pin B06: TIM4 CH1 (AF2)

# dma
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin B00 0
# pin B00: DMA2 Stream 2 Channel 0
dma pin B01 0
# pin B01: DMA2 Stream 2 Channel 0
dma pin A03 0
# pin A03: DMA1 Stream 7 Channel 3
dma pin A02 0
# pin A02: DMA1 Stream 1 Channel 3
dma pin B06 0
# pin B06: DMA1 Stream 0 Channel 2

# feature
feature OSD

# master
set dshot_burst = ON
set system_hse_mhz = 8
set max7456_spi_bus = 2
set dashboard_i2c_bus = 1
set flash_spi_bus = 2
set blackbox_device = SPIFLASH
set gyro_1_bustype = SPI
set gyro_1_spibus = 1

*/
