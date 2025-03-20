/*==============================================================================
Generated on: 2025-03-18 18:40:43.314169
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: DREAMFLYF411
Manufacturer ID: DFRC

//copy this line to madflight.ino to use this flight controller
#include <madflight_board_DFRC-DREAMFLYF411.h>

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out.
==============================================================================*/

#define HAL_BOARD_NAME "BETAFLIGHT-DFRC-DREAMFLYF411"
#define HAL_MCU "STM32F411"

#define MADFLIGHT_BOARD R""(
imu_gizmo MPU6000 // #define USE_GYRO_SPI_MPU6000
imu_gizmo ICM42688P // #define USE_GYRO_SPI_ICM42688P
// resource BEEPER 1 C15
pin_out0 PB10 // resource MOTOR 1 B10
pin_out1 PB6 // resource MOTOR 2 B06
pin_out2 PB7 // resource MOTOR 3 B07
pin_out3 PB8 // resource MOTOR 4 B08
pin_rcl_ppm PA10 // resource PPM 1 A10
// resource LED_STRIP 1 A00
pin_ser0_tx PA9 // resource SERIAL_TX 1 A09
pin_ser1_tx PA2 // resource SERIAL_TX 2 A02
pin_ser0_rx PA10 // resource SERIAL_RX 1 A10
pin_ser1_rx PA3 // resource SERIAL_RX 2 A03
pin_led PC13 // resource LED 1 C13
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PB14 // resource SPI_MISO 2 B14
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi1_mosi PB15 // resource SPI_MOSI 2 B15
pin_bat_v PB0 // resource ADC_BATT 1 B00
bat_gizmo ADC
pin_bat_i PB1 // resource ADC_CURR 1 B01
bat_gizmo ADC
// resource FLASH_CS 1 A14
// resource OSD_CS 1 B12
pin_imu_int PA1 // resource GYRO_EXTI 1 A01
pin_imu_cs PA4 // resource GYRO_CS 1 A04
// set serialrx_provider = CRSF
// set blackbox_device = SPIFLASH
// set dshot_burst = AUTO
// set dshot_bitbang = OFF
// set current_meter = ADC
// set battery_meter = ADC
// set beeper_inversion = ON
// set beeper_od = OFF
// set system_hse_mhz = 8
// set max7456_spi_bus = 2
// set flash_spi_bus = 2
// set gyro_1_bustype = SPI
imu_spi_bus 0 // set gyro_1_spibus = 1
imu_align CW270 // set gyro_1_sensor_align = CW270
)"" //end of MADFLIGHT_BOARD


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F411 (S411) 4.3.0 Jun 14 2022 / 00:48:04 (229ac66) MSP API: 1.44

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_ICM42688P
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_ICM42688P
#define USE_ACCGYRO_BMI270
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

board_name DREAMFLYF411
manufacturer_id DFRC

# resources
resource BEEPER 1 C15
resource MOTOR 1 B10
resource MOTOR 2 B06
resource MOTOR 3 B07
resource MOTOR 4 B08
resource PPM 1 A10
resource LED_STRIP 1 A00
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 A02
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 A03
resource LED 1 C13
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource ADC_BATT 1 B00
resource ADC_CURR 1 B01
resource FLASH_CS 1 A14
resource OSD_CS 1 B12
resource GYRO_EXTI 1 A01
resource GYRO_CS 1 A04

# timer
timer B10 AF1
# pin B10: TIM2 CH3 (AF1)
timer B06 AF2
# pin B06: TIM4 CH1 (AF2)
timer B07 AF2
# pin B07: TIM4 CH2 (AF2)
timer B08 AF2
# pin B08: TIM4 CH3 (AF2)
timer A00 AF2
# pin A00: TIM5 CH1 (AF2)
timer A09 AF1
# pin A09: TIM1 CH2 (AF1)
timer A10 AF1
# pin A10: TIM1 CH3 (AF1)

# dma
dma ADC 1 0
# ADC 1: DMA2 Stream 0 Channel 0
dma pin B10 0
# pin B10: DMA1 Stream 1 Channel 3
dma pin B06 0
# pin B06: DMA1 Stream 0 Channel 2
dma pin B07 0
# pin B07: DMA1 Stream 3 Channel 2
dma pin B08 0
# pin B08: DMA1 Stream 7 Channel 2
dma pin A00 0
# pin A00: DMA1 Stream 2 Channel 6
dma pin A09 0
# pin A09: DMA2 Stream 6 Channel 0
dma pin A10 0
# pin A10: DMA2 Stream 6 Channel 0


# feature
feature LED_STRIP
feature OSD
feature RX_SERIAL

# serial
serial 0 64 115200 57600 0 115200

# master
set serialrx_provider = CRSF
set blackbox_device = SPIFLASH
set dshot_burst = AUTO
set dshot_bitbang = OFF
set current_meter = ADC
set battery_meter = ADC
set beeper_inversion = ON
set beeper_od = OFF
set system_hse_mhz = 8
set max7456_spi_bus = 2
set flash_spi_bus = 2
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW270

*/
