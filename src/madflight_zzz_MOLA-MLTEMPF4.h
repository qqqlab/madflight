/*==============================================================================
Generated on: 2025-04-16 00:58:15.294675
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: MLTEMPF4
Manufacturer ID: MOLA

//copy this line to madflight.ino to use this flight controller
#include <madflight_zzz_MOLA-MLTEMPF4.h>

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out, or use madflight_config to override.
==============================================================================*/

#define MF_BOARD_NAME "BETAFLIGHT-MOLA-MLTEMPF4"
#define MF_MCU_NAME "STM32F405"

const char* madflight_board = R""(
imu_bus_type SPI
imu_gizmo MPU6000 // #define USE_GYRO_SPI_MPU6000
// resource BEEPER 1 B04
pin_out0 PB5 // resource MOTOR 1 B05
pin_out1 PB0 // resource MOTOR 2 B00
pin_out2 PB3 // resource MOTOR 3 B03
pin_out3 PB1 // resource MOTOR 4 B01
// resource LED_STRIP 1 A08
pin_ser0_tx PA9 // resource SERIAL_TX 1 A09
pin_ser1_tx PA2 // resource SERIAL_TX 2 A02
pin_ser3_tx PA0 // resource SERIAL_TX 4 A00
pin_ser4_tx PC12 // resource SERIAL_TX 5 C12
pin_ser5_tx PC6 // resource SERIAL_TX 6 C06
pin_ser0_rx PA10 // resource SERIAL_RX 1 A10
pin_ser1_rx PA3 // resource SERIAL_RX 2 A03
pin_ser3_rx PA1 // resource SERIAL_RX 4 A01
pin_ser4_rx PD2 // resource SERIAL_RX 5 D02
pin_ser5_rx PC7 // resource SERIAL_RX 6 C07
pin_led PC3 // resource LED 1 C03
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi2_sclk PC10 // resource SPI_SCK 3 C10
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PB14 // resource SPI_MISO 2 B14
pin_spi2_miso PC11 // resource SPI_MISO 3 C11
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi1_mosi PB15 // resource SPI_MOSI 2 B15
pin_spi2_mosi PC12 // resource SPI_MOSI 3 C12
// resource ESCSERIAL 1 B08
pin_bat_v PC0 // resource ADC_BATT 1 C00
bat_gizmo ADC
pin_bat_i PC1 // resource ADC_CURR 1 C01
bat_gizmo ADC
pin_bbx_cs PB12 // resource SDCARD_CS 1 B12
bbx_gizmo SDSPI
// resource SDCARD_DETECT 1 C13
// resource OSD_CS 1 A15
pin_imu_int PC5 // resource GYRO_EXTI 1 C05
pin_imu_cs PA4 // resource GYRO_CS 1 A04
// set blackbox_device = SDCARD
// set current_meter = ADC
// set battery_meter = ADC
// set ibata_scale = 140
// set beeper_inversion = ON
// set beeper_od = OFF
// set sdcard_detect_inverted = ON
// set sdcard_mode = SPI
bbx_spi_bus 1 // set sdcard_spi_bus = 2
// set system_hse_mhz = 8
// set max7456_spi_bus = 3
// set gyro_1_bustype = SPI
imu_spi_bus 0 // set gyro_1_spibus = 1
imu_align CW180 // set gyro_1_sensor_align = CW180
// set gyro_1_align_yaw = 1800
)""; //end of madflight_board


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F405 (S405) 4.1.0 Jul 27 2019 / 18:42:59 (f6870e418) MSP API: 1.42

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_MAX7456
#define USE_SDCARD

board_name MLTEMPF4
manufacturer_id MOLA

# resources
resource BEEPER 1 B04
resource MOTOR 1 B05
resource MOTOR 2 B00
resource MOTOR 3 B03
resource MOTOR 4 B01
resource LED_STRIP 1 A08
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 A02
resource SERIAL_TX 4 A00
resource SERIAL_TX 5 C12
resource SERIAL_TX 6 C06
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 A03
resource SERIAL_RX 4 A01
resource SERIAL_RX 5 D02
resource SERIAL_RX 6 C07
resource LED 1 C03
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 C10
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 C11
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 C12
resource ESCSERIAL 1 B08
resource ADC_BATT 1 C00
resource ADC_CURR 1 C01
resource SDCARD_CS 1 B12
resource SDCARD_DETECT 1 C13
resource OSD_CS 1 A15
resource GYRO_EXTI 1 C05
resource GYRO_CS 1 A04

# timer
timer B05 AF2
# pin B05: TIM3 CH2 (AF2)
timer B00 AF2
# pin B00: TIM3 CH3 (AF2)
timer B03 AF1
# pin B03: TIM2 CH2 (AF1)
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer A02 AF2
# pin A02: TIM5 CH3 (AF2)
timer A03 AF2
# pin A03: TIM5 CH4 (AF2)
timer A08 AF1
# pin A08: TIM1 CH1 (AF1)
timer B08 AF2
# pin B08: TIM4 CH3 (AF2)

# dma
dma SPI_TX 2 0
# SPI_TX 2: DMA1 Stream 4 Channel 0
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin B05 0
# pin B05: DMA1 Stream 5 Channel 5
dma pin B00 0
# pin B00: DMA1 Stream 7 Channel 5
dma pin B03 0
# pin B03: DMA1 Stream 6 Channel 3
dma pin B01 0
# pin B01: DMA1 Stream 2 Channel 5
dma pin A02 0
# pin A02: DMA1 Stream 0 Channel 6
dma pin A03 0
# pin A03: DMA1 Stream 1 Channel 6
dma pin A08 0
# pin A08: DMA2 Stream 6 Channel 0
dma pin B08 0
# pin B08: DMA1 Stream 7 Channel 2

# feature
feature OSD

# master
set blackbox_device = SDCARD
set current_meter = ADC
set battery_meter = ADC
set ibata_scale = 140
set beeper_inversion = ON
set beeper_od = OFF
set sdcard_detect_inverted = ON
set sdcard_mode = SPI
set sdcard_spi_bus = 2
set system_hse_mhz = 8
set max7456_spi_bus = 3
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW180
set gyro_1_align_yaw = 1800

*/
