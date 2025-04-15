/*==============================================================================
Generated on: 2025-04-16 00:58:15.394953
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: TMOTORF7V2
Manufacturer ID: TMTR

//copy this line to madflight.ino to use this flight controller
#include <madflight_zzz_TMTR-TMOTORF7V2.h>

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out, or use madflight_config to override.
==============================================================================*/

#define MF_BOARD_NAME "BETAFLIGHT-TMTR-TMOTORF7V2"
#define MF_MCU_NAME "STM32F7X2"

const char* madflight_board = R""(
imu_bus_type SPI
imu_gizmo MPU6000 // #define USE_GYRO_SPI_MPU6000
imu_gizmo ICM42688P // #define USE_GYRO_SPI_ICM42688P
imu_gizmo MPU6500 // #define USE_GYRO_SPI_MPU6500
bar_gizmo BMP280 // #define USE_BARO_BMP280
// resource BEEPER 1 C13
pin_out0 PB0 // resource MOTOR 1 B00
pin_out1 PB1 // resource MOTOR 2 B01
pin_out2 PB4 // resource MOTOR 3 B04
pin_out3 PB5 // resource MOTOR 4 B05
pin_out4 PB6 // resource MOTOR 5 B06
pin_out5 PB7 // resource MOTOR 6 B07
pin_out6 PC8 // resource MOTOR 7 C08
pin_out7 PC9 // resource MOTOR 8 C09
pin_rcl_ppm PA3 // resource PPM 1 A03
// resource LED_STRIP 1 A08
pin_ser0_tx PA9 // resource SERIAL_TX 1 A09
pin_ser1_tx PA2 // resource SERIAL_TX 2 A02
pin_ser2_tx PB10 // resource SERIAL_TX 3 B10
pin_ser3_tx PA0 // resource SERIAL_TX 4 A00
pin_ser4_tx PC12 // resource SERIAL_TX 5 C12
pin_ser5_tx PC6 // resource SERIAL_TX 6 C06
pin_ser0_rx PA10 // resource SERIAL_RX 1 A10
pin_ser1_rx PA3 // resource SERIAL_RX 2 A03
pin_ser2_rx PB11 // resource SERIAL_RX 3 B11
pin_ser3_rx PA1 // resource SERIAL_RX 4 A01
pin_ser4_rx PD2 // resource SERIAL_RX 5 D02
pin_ser5_rx PC7 // resource SERIAL_RX 6 C07
pin_i2c0_scl PB8 // resource I2C_SCL 1 B08
pin_i2c0_sda PB9 // resource I2C_SDA 1 B09
pin_led PC14 // resource LED 1 C14
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi2_sclk PC10 // resource SPI_SCK 3 C10
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PB14 // resource SPI_MISO 2 B14
pin_spi2_miso PC11 // resource SPI_MISO 3 C11
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi1_mosi PB15 // resource SPI_MOSI 2 B15
pin_spi2_mosi PB2 // resource SPI_MOSI 3 B02
// resource CAMERA_CONTROL 1 B03
pin_bat_v PC2 // resource ADC_BATT 1 C02
bat_gizmo ADC
// resource ADC_RSSI 1 C00
pin_bat_i PC3 // resource ADC_CURR 1 C03
bat_gizmo ADC
// resource PINIO 1 C01
// resource PINIO 2 C15
// resource FLASH_CS 1 A15
// resource OSD_CS 1 B12
pin_imu_int PC4 // resource GYRO_EXTI 1 C04
pin_imu_cs PA4 // resource GYRO_CS 1 A04
// set mag_bustype = I2C
// set baro_bustype = I2C
bar_i2c_bus 0 // set baro_i2c_device = 1
// set serialrx_provider = SBUS
// set blackbox_device = SPIFLASH
// set current_meter = ADC
// set battery_meter = ADC
// set ibata_scale = 250
// set beeper_inversion = ON
// set beeper_od = OFF
// set max7456_spi_bus = 2
// set pinio_config = 129,1,1,1
// set pinio_box = 0,41,255,255
// set flash_spi_bus = 3
// set gyro_1_bustype = SPI
imu_spi_bus 0 // set gyro_1_spibus = 1
)""; //end of madflight_board


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F7X2 (S7X2) 4.2.8 Feb 15 2021 / 12:10:35 (101738d8e) MSP API: 1.43

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACCGYRO_BMI270
#define USE_GYRO_SPI_ICM42688P
#define USE_ACC_SPI_ICM42688P
#define USE_GYRO_SPI_MPU6500
#define USE_ACC_SPI_MPU6500
#define USE_BARO
#define USE_BARO_BMP280
#define USE_FLASH
#define USE_FLASH_M25P16
#define USE_MAX7456

board_name TMOTORF7V2
manufacturer_id TMTR

# resources
resource BEEPER 1 C13
resource MOTOR 1 B00
resource MOTOR 2 B01
resource MOTOR 3 B04
resource MOTOR 4 B05
resource MOTOR 5 B06
resource MOTOR 6 B07
resource MOTOR 7 C08
resource MOTOR 8 C09
resource PPM 1 A03
resource LED_STRIP 1 A08
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 A02
resource SERIAL_TX 3 B10
resource SERIAL_TX 4 A00
resource SERIAL_TX 5 C12
resource SERIAL_TX 6 C06
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 A03
resource SERIAL_RX 3 B11
resource SERIAL_RX 4 A01
resource SERIAL_RX 5 D02
resource SERIAL_RX 6 C07
resource I2C_SCL 1 B08
resource I2C_SDA 1 B09
resource LED 1 C14
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 C10
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 C11
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 B02
resource CAMERA_CONTROL 1 B03
resource ADC_BATT 1 C02
resource ADC_RSSI 1 C00
resource ADC_CURR 1 C03
resource PINIO 1 C01
resource PINIO 2 C15
resource FLASH_CS 1 A15
resource OSD_CS 1 B12
resource GYRO_EXTI 1 C04
resource GYRO_CS 1 A04

# timer
timer A03 AF2
# pin A03: TIM5 CH4 (AF2)
timer B00 AF2
# pin B00: TIM3 CH3 (AF2)
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer B04 AF2
# pin B04: TIM3 CH1 (AF2)
timer B05 AF2
# pin B05: TIM3 CH2 (AF2)
timer B06 AF2
# pin B06: TIM4 CH1 (AF2)
timer B07 AF2
# pin B07: TIM4 CH2 (AF2)
timer C08 AF3
# pin C08: TIM8 CH3 (AF3)
timer C09 AF3
# pin C09: TIM8 CH4 (AF3)
timer A08 AF1
# pin A08: TIM1 CH1 (AF1)
timer B03 AF1
# pin B03: TIM2 CH2 (AF1)

# dma
dma ADC 1 0
# ADC 1: DMA2 Stream 0 Channel 0
dma pin A03 0
# pin A03: DMA1 Stream 1 Channel 6
dma pin B00 0
# pin B00: DMA1 Stream 7 Channel 5
dma pin B01 0
# pin B01: DMA1 Stream 2 Channel 5
dma pin B04 0
# pin B04: DMA1 Stream 4 Channel 5
dma pin B05 0
# pin B05: DMA1 Stream 5 Channel 5
dma pin B06 0
# pin B06: DMA1 Stream 0 Channel 2
dma pin B07 0
# pin B07: DMA1 Stream 3 Channel 2
dma pin C08 0
# pin C08: DMA2 Stream 2 Channel 0
dma pin C09 0
# pin C09: DMA2 Stream 7 Channel 7
dma pin A08 0
# pin A08: DMA2 Stream 6 Channel 0
dma pin B03 0
# pin B03: DMA1 Stream 6 Channel 3

# feature
feature RX_SERIAL
feature OSD

# serial
serial 1 64 115200 57600 0 115200
serial 4 1 19200 57600 0 115200

# master
set mag_bustype = I2C
set baro_bustype = I2C
set baro_i2c_device = 1
set serialrx_provider = SBUS
set blackbox_device = SPIFLASH
set current_meter = ADC
set battery_meter = ADC
set ibata_scale = 250
set beeper_inversion = ON
set beeper_od = OFF
set max7456_spi_bus = 2
set pinio_config = 129,1,1,1
set pinio_box = 0,41,255,255
set flash_spi_bus = 3
set gyro_1_bustype = SPI
set gyro_1_spibus = 1

*/
