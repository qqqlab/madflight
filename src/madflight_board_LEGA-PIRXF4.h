/*==============================================================================
Generated on: 2025-03-18 18:40:43.497696
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: PIRXF4
Manufacturer ID: LEGA

//copy this line to madflight.ino to use this flight controller
#include <madflight_board_LEGA-PIRXF4.h>

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out.
==============================================================================*/

#define HAL_BOARD_NAME "BETAFLIGHT-LEGA-PIRXF4"
#define HAL_MCU "STM32F405"

#define MADFLIGHT_BOARD R""(
imu_gizmo MPU6500 // #define USE_GYRO_SPI_MPU6500
// resource BEEPER 1 A15
pin_out0 PB4 // resource MOTOR 1 B04
pin_out1 PB5 // resource MOTOR 2 B05
pin_out2 PB8 // resource MOTOR 3 B08
pin_out3 PB9 // resource MOTOR 4 B09
pin_ser0_tx PB6 // resource SERIAL_TX 1 B06
pin_ser1_tx PA2 // resource SERIAL_TX 2 A02
pin_ser2_tx PB10 // resource SERIAL_TX 3 B10
pin_ser3_tx PA0 // resource SERIAL_TX 4 A00
pin_ser5_tx PC6 // resource SERIAL_TX 6 C06
pin_ser0_rx PB7 // resource SERIAL_RX 1 B07
pin_ser1_rx PA3 // resource SERIAL_RX 2 A03
pin_ser2_rx PB11 // resource SERIAL_RX 3 B11
pin_ser3_rx PA1 // resource SERIAL_RX 4 A01
pin_ser5_rx PC7 // resource SERIAL_RX 6 C07
pin_ser2_inv PB1 // resource INVERTER 3 B01
pin_ser5_inv PA8 // resource INVERTER 6 A08
pin_led PC13 // resource LED 1 C13
// resource LED 2 C14
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PB14 // resource SPI_MISO 2 B14
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi1_mosi PB15 // resource SPI_MOSI 2 B15
// resource ESCSERIAL 1 B09
pin_bat_v PC2 // resource ADC_BATT 1 C02
bat_gizmo ADC
// resource ADC_RSSI 1 C01
pin_bat_i PC3 // resource ADC_CURR 1 C03
bat_gizmo ADC
// resource SDCARD_DETECT 1 C15
// resource OSD_CS 1 B12
pin_imu_cs PC4 // resource GYRO_CS 1 C04
// resource USB_DETECT 1 A09
// set blackbox_device = NONE
// set dshot_burst = ON
// set current_meter = ADC
// set battery_meter = ADC
// set beeper_inversion = ON
// set beeper_od = OFF
// set system_hse_mhz = 8
// set max7456_spi_bus = 2
// set gyro_1_bustype = SPI
imu_spi_bus 0 // set gyro_1_spibus = 1
)"" //end of MADFLIGHT_BOARD


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F405 (S405) 4.2.0 Mar  5 2020 / 22:29:55 (c29b125a59) MSP API: 1.43

#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_MAX7456
#define USE_SDCARD

board_name PIRXF4
manufacturer_id LEGA

# resources
resource BEEPER 1 A15
resource MOTOR 1 B04
resource MOTOR 2 B05
resource MOTOR 3 B08
resource MOTOR 4 B09
resource SERIAL_TX 1 B06
resource SERIAL_TX 2 A02
resource SERIAL_TX 3 B10
resource SERIAL_TX 4 A00
resource SERIAL_TX 6 C06
resource SERIAL_RX 1 B07
resource SERIAL_RX 2 A03
resource SERIAL_RX 3 B11
resource SERIAL_RX 4 A01
resource SERIAL_RX 6 C07
resource INVERTER 3 B01
resource INVERTER 6 A08
resource LED 1 C13
resource LED 2 C14
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource ESCSERIAL 1 B09
resource ADC_BATT 1 C02
resource ADC_RSSI 1 C01
resource ADC_CURR 1 C03
resource SDCARD_DETECT 1 C15
resource OSD_CS 1 B12
resource GYRO_CS 1 C04
resource USB_DETECT 1 A09

# timer
timer B04 AF2
# pin B04: TIM3 CH1 (AF2)
timer B05 AF2
# pin B05: TIM3 CH2 (AF2)
timer B08 AF2
# pin B08: TIM4 CH3 (AF2)
timer B09 AF2
# pin B09: TIM4 CH4 (AF2)

# dma
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin B04 0
# pin B04: DMA1 Stream 4 Channel 5
dma pin B05 0
# pin B05: DMA1 Stream 5 Channel 5
dma pin B08 0
# pin B08: DMA1 Stream 7 Channel 2

# feature
feature OSD

# master
set blackbox_device = NONE
set dshot_burst = ON
set current_meter = ADC
set battery_meter = ADC
set beeper_inversion = ON
set beeper_od = OFF
set system_hse_mhz = 8
set max7456_spi_bus = 2
set gyro_1_bustype = SPI
set gyro_1_spibus = 1

*/
