/*==============================================================================
Generated on: 2025-03-18 18:40:43.631070
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: VGOODRCF411_DJI
Manufacturer ID: VGRC

//copy this line to madflight.ino to use this flight controller
#include <madflight_board_VGRC-VGOODRCF411_DJI.h>

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out.
==============================================================================*/

#define HAL_BOARD_NAME "BETAFLIGHT-VGRC-VGOODRCF411_DJI"
#define HAL_MCU "STM32F411"

#define MADFLIGHT_BOARD R""(
// resource BEEPER 1 B02
pin_out0 PB3 // resource MOTOR 1 B03
pin_out1 PB4 // resource MOTOR 2 B04
pin_out2 PB6 // resource MOTOR 3 B06
pin_out3 PB7 // resource MOTOR 4 B07
pin_rcl_ppm PB9 // resource PPM 1 B09
// resource LED_STRIP 1 A08
pin_ser0_tx PA9 // resource SERIAL_TX 1 A09
pin_ser1_tx PA2 // resource SERIAL_TX 2 A02
pin_ser0_rx PA10 // resource SERIAL_RX 1 A10
pin_ser1_rx PA3 // resource SERIAL_RX 2 A03
pin_ser0_inv PB10 // resource INVERTER 1 B10
pin_led PC13 // resource LED 1 C13
// resource LED 2 C14
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PB14 // resource SPI_MISO 2 B14
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi1_mosi PB15 // resource SPI_MOSI 2 B15
// resource ESCSERIAL 1 B09
pin_bat_v PA0 // resource ADC_BATT 1 A00
bat_gizmo ADC
// resource ADC_RSSI 1 B01
pin_bat_i PA1 // resource ADC_CURR 1 A01
bat_gizmo ADC
// resource OSD_CS 1 B12
pin_imu_int PB0 // resource GYRO_EXTI 1 B00
pin_imu_cs PA4 // resource GYRO_CS 1 A04
// set serialrx_provider = SBUS
// set current_meter = ADC
// set battery_meter = ADC
// set beeper_inversion = ON
// set max7456_spi_bus = 2
// set gyro_1_bustype = SPI
imu_spi_bus 0 // set gyro_1_spibus = 1
)"" //end of MADFLIGHT_BOARD


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F411 (S411) 4.1.7 May 28 2020 / 15:05:37 (9ba02a587) MSP API: 1.42

board_name VGOODRCF411_DJI
manufacturer_id VGRC

# resources
resource BEEPER 1 B02
resource MOTOR 1 B03
resource MOTOR 2 B04
resource MOTOR 3 B06
resource MOTOR 4 B07
resource PPM 1 B09
resource LED_STRIP 1 A08
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 A02
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 A03
resource INVERTER 1 B10
resource LED 1 C13
resource LED 2 C14
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource ESCSERIAL 1 B09
resource ADC_BATT 1 A00
resource ADC_RSSI 1 B01
resource ADC_CURR 1 A01
resource OSD_CS 1 B12
resource GYRO_EXTI 1 B00
resource GYRO_CS 1 A04

# timer
timer B09 AF3
# pin B09: TIM11 CH1 (AF3)
timer B03 AF1
# pin B03: TIM2 CH2 (AF1)
timer B04 AF2
# pin B04: TIM3 CH1 (AF2)
timer B06 AF2
# pin B06: TIM4 CH1 (AF2)
timer B07 AF2
# pin B07: TIM4 CH2 (AF2)
timer A15 AF1
# pin A15: TIM2 CH1 (AF1)
timer B08 AF3
# pin B08: TIM10 CH1 (AF3)
timer A08 AF1
# pin A08: TIM1 CH1 (AF1)

# dma
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin B03 0
# pin B03: DMA1 Stream 6 Channel 3
dma pin B04 0
# pin B04: DMA1 Stream 4 Channel 5
dma pin B06 0
# pin B06: DMA1 Stream 0 Channel 2
dma pin B07 0
# pin B07: DMA1 Stream 3 Channel 2
dma pin A15 0
# pin A15: DMA1 Stream 5 Channel 3
dma pin A08 0
# pin A08: DMA2 Stream 6 Channel 0

# feature
feature OSD

# serial
serial 0 64 115200 57600 0 115200

# master
set serialrx_provider = SBUS
set current_meter = ADC
set battery_meter = ADC
set beeper_inversion = ON
set max7456_spi_bus = 2
set gyro_1_bustype = SPI
set gyro_1_spibus = 1

*/
