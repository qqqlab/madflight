/*==============================================================================
Generated on: 2025-04-16 00:58:15.324824
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: FLOWBOX
Manufacturer ID: NERC

//copy this line to madflight.ino to use this flight controller
#include <madflight_zzz_NERC-FLOWBOX.h>

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out, or use madflight_config to override.
==============================================================================*/

#define MF_BOARD_NAME "BETAFLIGHT-NERC-FLOWBOX"
#define MF_MCU_NAME "STM32F411"

const char* madflight_board = R""(
imu_bus_type SPI
// resource BEEPER 1 B01
pin_out0 PB10 // resource MOTOR 1 B10
pin_out1 PA0 // resource MOTOR 2 A00
pin_out2 PB6 // resource MOTOR 3 B06
pin_out3 PB7 // resource MOTOR 4 B07
pin_ser0_tx PA9 // resource SERIAL_TX 1 A09
pin_ser1_tx PA2 // resource SERIAL_TX 2 A02
pin_ser0_rx PA10 // resource SERIAL_RX 1 A10
pin_ser1_rx PA3 // resource SERIAL_RX 2 A03
pin_led PC13 // resource LED 1 C13
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi2_sclk PB3 // resource SPI_SCK 3 B03
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi2_miso PB4 // resource SPI_MISO 3 B04
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi2_mosi PB5 // resource SPI_MOSI 3 B05
// resource FLASH_CS 1 A15
pin_imu_int PA1 // resource GYRO_EXTI 1 A01
pin_imu_cs PA4 // resource GYRO_CS 1 A04
// set blackbox_device = SPIFLASH
// set beeper_frequency = 2185
// set beeper_inversion = ON
// set beeper_od = OFF
// set system_hse_mhz = 8
// set flash_spi_bus = 3
imu_spi_bus 0 // set gyro_1_spibus = 1
imu_align CW180 // set gyro_1_sensor_align = CW180
// set gyro_1_align_yaw = 1800
)""; //end of madflight_board


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F411 (S411) 4.3.0 May 20 2021 / 03:37:19 (e0ad71db2) MSP API: 1.44

board_name FLOWBOX
manufacturer_id NERC

# resources
resource BEEPER 1 B01
resource MOTOR 1 B10
resource MOTOR 2 A00
resource MOTOR 3 B06
resource MOTOR 4 B07
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 A02
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 A03
resource LED 1 C13
resource SPI_SCK 1 A05
resource SPI_SCK 3 B03
resource SPI_MISO 1 A06
resource SPI_MISO 3 B04
resource SPI_MOSI 1 A07
resource SPI_MOSI 3 B05
resource FLASH_CS 1 A15
resource GYRO_EXTI 1 A01
resource GYRO_CS 1 A04

# timer
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer B10 AF1
# pin B10: TIM2 CH3 (AF1)
timer B06 AF2
# pin B06: TIM4 CH1 (AF2)
timer B07 AF2
# pin B07: TIM4 CH2 (AF2)
timer A00 AF1
# pin A00: TIM2 CH1 (AF1)

# dma
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin B01 0
# pin B01: DMA1 Stream 2 Channel 5
dma pin B10 0
# pin B10: DMA1 Stream 1 Channel 3
dma pin B06 0
# pin B06: DMA1 Stream 0 Channel 2
dma pin B07 0
# pin B07: DMA1 Stream 3 Channel 2
dma pin A00 0
# pin A00: DMA1 Stream 5 Channel 3

# feature
feature -AIRMODE
feature RX_SERIAL

# serial
serial 0 64 115200 57600 0 115200

# master

set blackbox_device = SPIFLASH
set beeper_frequency = 2185
set beeper_inversion = ON
set beeper_od = OFF
set system_hse_mhz = 8
set flash_spi_bus = 3
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW180
set gyro_1_align_yaw = 1800

*/
