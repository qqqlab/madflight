/*==============================================================================
Generated on: 2025-04-16 00:58:15.315091
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: NBD_CRICKETF7V2
Manufacturer ID: NEBD

//copy this line to madflight.ino to use this flight controller
#include <madflight_zzz_NEBD-NBD_CRICKETF7V2.h>

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out, or use madflight_config to override.
==============================================================================*/

#define MF_BOARD_NAME "BETAFLIGHT-NEBD-NBD_CRICKETF7V2"
#define MF_MCU_NAME "STM32F7X2"

const char* madflight_board = R""(
imu_bus_type SPI
// resource BEEPER 1 C15
pin_out0 PB4 // resource MOTOR 1 B04
pin_out1 PB1 // resource MOTOR 2 B01
pin_out2 PB3 // resource MOTOR 3 B03
pin_out3 PB0 // resource MOTOR 4 B00
// resource LED_STRIP 1 A08
pin_ser1_tx PA2 // resource SERIAL_TX 2 A02
pin_ser2_tx PB10 // resource SERIAL_TX 3 B10
pin_ser3_tx PA0 // resource SERIAL_TX 4 A00
pin_ser5_tx PC6 // resource SERIAL_TX 6 C06
pin_ser0_rx PA10 // resource SERIAL_RX 1 A10
pin_ser1_rx PA3 // resource SERIAL_RX 2 A03
pin_ser2_rx PB11 // resource SERIAL_RX 3 B11
pin_ser3_rx PA1 // resource SERIAL_RX 4 A01
pin_ser4_rx PD2 // resource SERIAL_RX 5 D02
pin_ser5_rx PC7 // resource SERIAL_RX 6 C07
pin_led PA15 // resource LED 1 A15
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi2_sclk PC10 // resource SPI_SCK 3 C10
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PB14 // resource SPI_MISO 2 B14
pin_spi2_miso PC11 // resource SPI_MISO 3 C11
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi1_mosi PB15 // resource SPI_MOSI 2 B15
pin_spi2_mosi PC12 // resource SPI_MOSI 3 C12
pin_bat_v PC0 // resource ADC_BATT 1 C00
bat_gizmo ADC
pin_bat_i PC1 // resource ADC_CURR 1 C01
bat_gizmo ADC
// resource OSD_CS 1 C13
pin_imu_int PC4 // resource GYRO_EXTI 1 C04
// resource GYRO_EXTI 2 C09
pin_imu_cs PB12 // resource GYRO_CS 1 B12
// resource GYRO_CS 2 A04
// set gyro_to_use = BOTH
// set serialrx_provider = SBUS
// set current_meter = ADC
// set battery_meter = ADC
// set beeper_inversion = ON
// set beeper_od = OFF
// set max7456_spi_bus = 3
// set gyro_1_bustype = SPI
imu_spi_bus 1 // set gyro_1_spibus = 2
imu_align CW270 // set gyro_1_sensor_align = CW270
// set gyro_1_align_yaw = 2700
// set gyro_2_bustype = SPI
// set gyro_2_spibus = 1
// set gyro_2_sensor_align = CW0
// set gyro_2_align_yaw = 0
)""; //end of madflight_board


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
﻿# Betaflight / STM32F7X2 (S7X2) 4.3.0 Jun 14 2022 / 00:50:37 (229ac66) MSP API: 1.44

board_name NBD_CRICKETF7V2
manufacturer_id NEBD

#define USE_ACC
#define USE_GYRO
#define USE_ACCGYRO_BMI270

# resources
resource BEEPER 1 C15
resource MOTOR 1 B04
resource MOTOR 2 B01
resource MOTOR 3 B03
resource MOTOR 4 B00
resource LED_STRIP 1 A08
resource SERIAL_TX 2 A02
resource SERIAL_TX 3 B10
resource SERIAL_TX 4 A00
resource SERIAL_TX 6 C06
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 A03
resource SERIAL_RX 3 B11
resource SERIAL_RX 4 A01
resource SERIAL_RX 5 D02
resource SERIAL_RX 6 C07
resource LED 1 A15
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 C10
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 C11
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 C12
resource ADC_BATT 1 C00
resource ADC_CURR 1 C01
resource OSD_CS 1 C13
resource GYRO_EXTI 1 C04
resource GYRO_EXTI 2 C09
resource GYRO_CS 1 B12
resource GYRO_CS 2 A04

# timer
timer B04 AF2
# pin B04: TIM3 CH1 (AF2)
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer B03 AF1
# pin B03: TIM2 CH2 (AF1)
timer B00 AF2
# pin B00: TIM3 CH3 (AF2)
timer A08 AF1
# pin A08: TIM1 CH1 (AF1)

# dma
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin B04 0
# pin B04: DMA1 Stream 4 Channel 5
dma pin B01 0
# pin B01: DMA1 Stream 2 Channel 5
dma pin B03 0
# pin B03: DMA1 Stream 6 Channel 3
dma pin B00 0
# pin B00: DMA1 Stream 7 Channel 5
dma pin A08 0
# pin A08: DMA2 Stream 6 Channel 0

# feature
feature RX_SERIAL
feature OSD

# serial
serial 0 64 115200 57600 0 115200
serial 5 1 115200 57600 0 115200

# master
set gyro_to_use = BOTH
set serialrx_provider = SBUS
set current_meter = ADC
set battery_meter = ADC
set beeper_inversion = ON
set beeper_od = OFF
set max7456_spi_bus = 3
set gyro_1_bustype = SPI
set gyro_1_spibus = 2
set gyro_1_sensor_align = CW270
set gyro_1_align_yaw = 2700
set gyro_2_bustype = SPI
set gyro_2_spibus = 1
set gyro_2_sensor_align = CW0
set gyro_2_align_yaw = 0

*/
