/*==============================================================================
Generated on: 2025-04-16 00:58:15.405915
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: VGOODRCF405_DJI
Manufacturer ID: VGRC

//copy this line to madflight.ino to use this flight controller
#include <madflight_zzz_VGRC-VGOODRCF405_DJI.h>

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out, or use madflight_config to override.
==============================================================================*/

#define MF_BOARD_NAME "BETAFLIGHT-VGRC-VGOODRCF405_DJI"
#define MF_MCU_NAME "STM32F405"

const char* madflight_board = R""(
imu_bus_type SPI
imu_gizmo MPU6000 // #define USE_GYRO_SPI_MPU6000
// resource BEEPER 1 C13
pin_out0 PA9 // resource MOTOR 1 A09
pin_out1 PA8 // resource MOTOR 2 A08
pin_out2 PC9 // resource MOTOR 3 C09
pin_out3 PC8 // resource MOTOR 4 C08
pin_rcl_ppm PB9 // resource PPM 1 B09
// resource LED_STRIP 1 B03
pin_ser0_tx PB6 // resource SERIAL_TX 1 B06
pin_ser1_tx PA2 // resource SERIAL_TX 2 A02
pin_ser2_tx PB10 // resource SERIAL_TX 3 B10
pin_ser3_tx PA0 // resource SERIAL_TX 4 A00
pin_ser4_tx PC12 // resource SERIAL_TX 5 C12
pin_ser5_tx PC6 // resource SERIAL_TX 6 C06
pin_ser0_rx PB7 // resource SERIAL_RX 1 B07
pin_ser1_rx PA3 // resource SERIAL_RX 2 A03
pin_ser2_rx PB11 // resource SERIAL_RX 3 B11
pin_ser3_rx PA1 // resource SERIAL_RX 4 A01
pin_ser4_rx PD2 // resource SERIAL_RX 5 D02
pin_ser5_rx PC7 // resource SERIAL_RX 6 C07
pin_ser0_inv PC0 // resource INVERTER 1 C00
pin_led PC15 // resource LED 1 C15
// resource LED 2 C14
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi2_sclk PC10 // resource SPI_SCK 3 C10
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PB14 // resource SPI_MISO 2 B14
pin_spi2_miso PC11 // resource SPI_MISO 3 C11
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi1_mosi PB15 // resource SPI_MOSI 2 B15
pin_spi2_mosi PB5 // resource SPI_MOSI 3 B05
// resource ESCSERIAL 1 B09
// resource CAMERA_CONTROL 1 B08
pin_bat_v PC1 // resource ADC_BATT 1 C01
bat_gizmo ADC
// resource ADC_RSSI 1 C02
pin_bat_i PC3 // resource ADC_CURR 1 C03
bat_gizmo ADC
// resource PINIO 1 B00
// resource FLASH_CS 1 A15
// resource OSD_CS 1 B12
pin_imu_int PC4 // resource GYRO_EXTI 1 C04
pin_imu_cs PA4 // resource GYRO_CS 1 A04
// set serialrx_provider = SBUS
// set adc_device = 3
// set blackbox_device = SPIFLASH
// set current_meter = ADC
// set battery_meter = ADC
// set beeper_inversion = ON
// set max7456_spi_bus = 2
// set flash_spi_bus = 3
// set gyro_1_bustype = SPI
imu_spi_bus 0 // set gyro_1_spibus = 1
)""; //end of madflight_board


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F405 (S405) 4.1.7 May 28 2020 / 15:05:17 (9ba02a587) MSP API: 1.42

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

board_name VGOODRCF405_DJI
manufacturer_id VGRC

# resources
resource BEEPER 1 C13
resource MOTOR 1 A09
resource MOTOR 2 A08
resource MOTOR 3 C09
resource MOTOR 4 C08
resource PPM 1 B09
resource LED_STRIP 1 B03
resource SERIAL_TX 1 B06
resource SERIAL_TX 2 A02
resource SERIAL_TX 3 B10
resource SERIAL_TX 4 A00
resource SERIAL_TX 5 C12
resource SERIAL_TX 6 C06
resource SERIAL_RX 1 B07
resource SERIAL_RX 2 A03
resource SERIAL_RX 3 B11
resource SERIAL_RX 4 A01
resource SERIAL_RX 5 D02
resource SERIAL_RX 6 C07
resource INVERTER 1 C00
resource LED 1 C15
resource LED 2 C14
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 C10
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 C11
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 B05
resource ESCSERIAL 1 B09
resource CAMERA_CONTROL 1 B08
resource ADC_BATT 1 C01
resource ADC_RSSI 1 C02
resource ADC_CURR 1 C03
resource PINIO 1 B00
resource FLASH_CS 1 A15
resource OSD_CS 1 B12
resource GYRO_EXTI 1 C04
resource GYRO_CS 1 A04

# timer
timer B09 AF3
# pin B09: TIM11 CH1 (AF3)
timer A09 AF1
# pin A09: TIM1 CH2 (AF1)
timer A08 AF1
# pin A08: TIM1 CH1 (AF1)
timer C09 AF3
# pin C09: TIM8 CH4 (AF3)
timer C08 AF3
# pin C08: TIM8 CH3 (AF3)
timer B08 AF2
# pin B08: TIM4 CH3 (AF2)
timer B03 AF1
# pin B03: TIM2 CH2 (AF1)

# dma
dma ADC 3 1
# ADC 3: DMA2 Stream 1 Channel 2
dma pin A09 0
# pin A09: DMA2 Stream 6 Channel 0
dma pin A08 0
# pin A08: DMA2 Stream 6 Channel 0
dma pin C09 0
# pin C09: DMA2 Stream 7 Channel 7
dma pin C08 0
# pin C08: DMA2 Stream 2 Channel 0
dma pin B08 0
# pin B08: DMA1 Stream 7 Channel 2
dma pin B03 0
# pin B03: DMA1 Stream 6 Channel 3

# feature
feature RX_SERIAL
feature OSD

# serial
serial 0 64 115200 57600 0 115200

# master
set serialrx_provider = SBUS
set adc_device = 3
set blackbox_device = SPIFLASH
set current_meter = ADC
set battery_meter = ADC
set beeper_inversion = ON
set max7456_spi_bus = 2
set flash_spi_bus = 3
set gyro_1_bustype = SPI
set gyro_1_spibus = 1

*/
