/*==============================================================================
Generated on: 2025-04-16 00:58:15.294675
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: LUXMINIF7
Manufacturer ID: LMNR

//copy this line to madflight.ino to use this flight controller
#include <madflight_zzz_LMNR-LUXMINIF7.h>

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out, or use madflight_config to override.
==============================================================================*/

#define MF_BOARD_NAME "BETAFLIGHT-LMNR-LUXMINIF7"
#define MF_MCU_NAME "STM32F7X2"

const char* madflight_board = R""(
imu_bus_type SPI
imu_gizmo MPU6500 // #define USE_GYRO_SPI_MPU6500
imu_gizmo MPU6000 // #define USE_GYRO_SPI_MPU6000
// resource BEEPER 1 B00
pin_out0 PB6 // resource MOTOR 1 B06
pin_out1 PC8 // resource MOTOR 2 C08
pin_out2 PB7 // resource MOTOR 3 B07
pin_out3 PC9 // resource MOTOR 4 C09
// resource LED_STRIP 1 A15
pin_ser0_tx PA9 // resource SERIAL_TX 1 A09
pin_ser2_tx PC10 // resource SERIAL_TX 3 C10
pin_ser3_tx PA0 // resource SERIAL_TX 4 A00
pin_ser4_tx PC12 // resource SERIAL_TX 5 C12
pin_ser5_tx PC6 // resource SERIAL_TX 6 C06
pin_ser0_rx PA10 // resource SERIAL_RX 1 A10
pin_ser1_rx PA3 // resource SERIAL_RX 2 A03
pin_ser2_rx PC11 // resource SERIAL_RX 3 C11
pin_ser3_rx PA1 // resource SERIAL_RX 4 A01
pin_ser5_rx PC7 // resource SERIAL_RX 6 C07
pin_i2c0_scl PB8 // resource I2C_SCL 1 B08
pin_i2c0_sda PB9 // resource I2C_SDA 1 B09
pin_led PC15 // resource LED 1 C15
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi2_sclk PB3 // resource SPI_SCK 3 B03
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PB14 // resource SPI_MISO 2 B14
pin_spi2_miso PB4 // resource SPI_MISO 3 B04
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi1_mosi PB15 // resource SPI_MOSI 2 B15
pin_spi2_mosi PB5 // resource SPI_MOSI 3 B05
// resource CAMERA_CONTROL 1 A08
pin_bat_v PC0 // resource ADC_BATT 1 C00
bat_gizmo ADC
// resource ADC_RSSI 1 C02
pin_bat_i PC1 // resource ADC_CURR 1 C01
bat_gizmo ADC
// resource PINIO 1 B10
// resource FLASH_CS 1 B12
// resource OSD_CS 1 D02
pin_imu_cs PC4 // resource GYRO_CS 1 C04
// set serialrx_provider = SBUS
// set blackbox_device = SPIFLASH
// set dshot_burst = ON
// set current_meter = ADC
// set battery_meter = ADC
// set ibata_scale = 179
// set beeper_inversion = ON
// set beeper_od = OFF
// set max7456_spi_bus = 3
// set pinio_config = 129,1,1,1
// set pinio_box = 40,255,255,255
// set flash_spi_bus = 2
// set gyro_1_bustype = SPI
imu_spi_bus 0 // set gyro_1_spibus = 1
// set gyro_1_align_yaw = 1800
)""; //end of madflight_board


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F7X2 (S7X2) 4.1.0 Oct  6 2019 / 06:48:40 (c6452a55c) MSP API: 1.42
# manufacturer_id: LMNR   board_name: LUXMINIF7   custom defaults: NO

#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_GYRO_SPI_MPU6000
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

board_name LUXMINIF7
manufacturer_id LMNR

# resources
resource BEEPER 1 B00
resource MOTOR 1 B06
resource MOTOR 2 C08
resource MOTOR 3 B07
resource MOTOR 4 C09
resource LED_STRIP 1 A15
resource SERIAL_TX 1 A09
resource SERIAL_TX 3 C10
resource SERIAL_TX 4 A00
resource SERIAL_TX 5 C12
resource SERIAL_TX 6 C06
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 A03
resource SERIAL_RX 3 C11
resource SERIAL_RX 4 A01
resource SERIAL_RX 6 C07
resource I2C_SCL 1 B08
resource I2C_SDA 1 B09
resource LED 1 C15
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 B03
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 B04
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 B05
resource CAMERA_CONTROL 1 A08
resource ADC_BATT 1 C00
resource ADC_RSSI 1 C02
resource ADC_CURR 1 C01
resource PINIO 1 B10
resource FLASH_CS 1 B12
resource OSD_CS 1 D02
resource GYRO_CS 1 C04

# timer
timer A15 AF1
# pin A15: TIM2 CH1 (AF1)
timer A08 AF1
# pin A08: TIM1 CH1 (AF1)
timer B06 AF2
# pin B06: TIM4 CH1 (AF2)
timer C08 AF3
# pin C08: TIM8 CH3 (AF3)
timer B07 AF2
# pin B07: TIM4 CH2 (AF2)
timer C09 AF3
# pin C09: TIM8 CH4 (AF3)

# dma
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma ADC 3 1
# ADC 3: DMA2 Stream 1 Channel 2
dma pin A15 0
# pin A15: DMA1 Stream 5 Channel 3
dma pin A08 0
# pin A08: DMA2 Stream 6 Channel 0
dma pin B06 0
# pin B06: DMA1 Stream 0 Channel 2
dma pin C08 0
# pin C08: DMA2 Stream 2 Channel 0
dma pin B07 0
# pin B07: DMA1 Stream 3 Channel 2
dma pin C09 0
# pin C09: DMA2 Stream 7 Channel 7

# feature
feature RX_SERIAL
feature OSD

# serial
serial 0 64 115200 57600 0 115200

# master
set serialrx_provider = SBUS
set blackbox_device = SPIFLASH
set dshot_burst = ON
set current_meter = ADC
set battery_meter = ADC
set ibata_scale = 179
set beeper_inversion = ON
set beeper_od = OFF
set max7456_spi_bus = 3
set pinio_config = 129,1,1,1
set pinio_box = 40,255,255,255
set flash_spi_bus = 2
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_1_align_yaw = 1800

*/
