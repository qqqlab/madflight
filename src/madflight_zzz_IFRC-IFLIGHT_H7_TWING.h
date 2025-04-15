/*==============================================================================
Generated on: 2025-04-16 00:58:15.274825
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: IFLIGHT_H7_TWING
Manufacturer ID: IFRC

//copy this line to madflight.ino to use this flight controller
#include <madflight_zzz_IFRC-IFLIGHT_H7_TWING.h>

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out, or use madflight_config to override.
==============================================================================*/

#define MF_BOARD_NAME "BETAFLIGHT-IFRC-IFLIGHT_H7_TWING"
#define MF_MCU_NAME "STM32H743"

const char* madflight_board = R""(
imu_bus_type SPI
imu_gizmo ICM20689 // #define USE_GYRO_SPI_ICM20689
bar_gizmo BMP280 // #define USE_BARO_BMP280
// resource BEEPER 1 C13
pin_out0 PA0 // resource MOTOR 1 A00
pin_out1 PA1 // resource MOTOR 2 A01
pin_out2 PB0 // resource MOTOR 3 B00
pin_out3 PB1 // resource MOTOR 4 B01
pin_out4 PB6 // resource MOTOR 5 B06
pin_out5 PB7 // resource MOTOR 6 B07
pin_out6 PC8 // resource MOTOR 7 C08
pin_out7 PC9 // resource MOTOR 8 C09
pin_rcl_ppm PA3 // resource PPM 1 A03
// resource LED_STRIP 1 A08
pin_ser0_tx PA9 // resource SERIAL_TX 1 A09
pin_ser1_tx PA2 // resource SERIAL_TX 2 A02
pin_ser2_tx PD8 // resource SERIAL_TX 3 D08
pin_ser3_tx PD1 // resource SERIAL_TX 4 D01
pin_ser4_tx PC12 // resource SERIAL_TX 5 C12
pin_ser5_tx PC6 // resource SERIAL_TX 6 C06
pin_ser6_tx PB4 // resource SERIAL_TX 7 B04
pin_ser7_tx PE1 // resource SERIAL_TX 8 E01
pin_ser0_rx PA10 // resource SERIAL_RX 1 A10
pin_ser1_rx PA3 // resource SERIAL_RX 2 A03
pin_ser2_rx PD9 // resource SERIAL_RX 3 D09
pin_ser3_rx PD0 // resource SERIAL_RX 4 D00
pin_ser4_rx PD2 // resource SERIAL_RX 5 D02
pin_ser5_rx PC7 // resource SERIAL_RX 6 C07
pin_ser6_rx PB3 // resource SERIAL_RX 7 B03
pin_ser7_rx PE0 // resource SERIAL_RX 8 E00
pin_i2c0_scl PB8 // resource I2C_SCL 1 B08
pin_i2c0_sda PB9 // resource I2C_SDA 1 B09
pin_led PC2 // resource LED 1 C02
// resource LED 2 C03
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi3_sclk PE12 // resource SPI_SCK 4 E12
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PB14 // resource SPI_MISO 2 B14
pin_spi3_miso PE13 // resource SPI_MISO 4 E13
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi1_mosi PB15 // resource SPI_MOSI 2 B15
pin_spi3_mosi PE14 // resource SPI_MOSI 4 E14
// resource CAMERA_CONTROL 1 E05
pin_bat_v PC1 // resource ADC_BATT 1 C01
bat_gizmo ADC
// resource ADC_RSSI 1 C04
pin_bat_i PC0 // resource ADC_CURR 1 C00
bat_gizmo ADC
// resource OSD_CS 1 E11
pin_imu_int PC5 // resource GYRO_EXTI 1 C05
// resource GYRO_EXTI 2 B11
pin_imu_cs PA4 // resource GYRO_CS 1 A04
// resource GYRO_CS 2 B12
// set gyro_to_use = BOTH
// set mag_bustype = I2C
mag_i2c_bus 0 // set mag_i2c_device = 1
// set baro_bustype = I2C
bar_i2c_bus 0 // set baro_i2c_device = 1
// set blackbox_device = SPIFLASH
// set current_meter = ADC
// set battery_meter = ADC
// set beeper_inversion = ON
// set beeper_od = OFF
// set max7456_spi_bus = 4
// set dashboard_i2c_bus = 1
// set gyro_1_bustype = SPI
imu_spi_bus 0 // set gyro_1_spibus = 1
// set gyro_2_spibus = 2
// set gyro_2_sensor_align = CW90
// set gyro_2_align_yaw = 900
)""; //end of madflight_board


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32H743 (SH74) 4.3.0 Sep  8 2021 / 08:14:42 (481b0b563) MSP API: 1.44

#define USE_GYRO
#define USE_GYRO_SPI_ICM20689
#define USE_ACC
#define USE_ACC_SPI_ICM20689
#define USE_BARO
#define USE_BARO_BMP280
#define USE_FLASH
#define USE_FLASH_M25P16
#define USE_FLASH_W25N01G
#define USE_MAX7456

board_name IFLIGHT_H7_TWING
manufacturer_id IFRC

# resources
resource BEEPER 1 C13
resource MOTOR 1 A00
resource MOTOR 2 A01
resource MOTOR 3 B00
resource MOTOR 4 B01
resource MOTOR 5 B06
resource MOTOR 6 B07
resource MOTOR 7 C08
resource MOTOR 8 C09
resource PPM 1 A03
resource LED_STRIP 1 A08
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 A02
resource SERIAL_TX 3 D08
resource SERIAL_TX 4 D01
resource SERIAL_TX 5 C12
resource SERIAL_TX 6 C06
resource SERIAL_TX 7 B04
resource SERIAL_TX 8 E01
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 A03
resource SERIAL_RX 3 D09
resource SERIAL_RX 4 D00
resource SERIAL_RX 5 D02
resource SERIAL_RX 6 C07
resource SERIAL_RX 7 B03
resource SERIAL_RX 8 E00
resource I2C_SCL 1 B08
resource I2C_SDA 1 B09
resource LED 1 C02
resource LED 2 C03
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 4 E12
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 4 E13
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 4 E14
resource CAMERA_CONTROL 1 E05
resource ADC_BATT 1 C01
resource ADC_RSSI 1 C04
resource ADC_CURR 1 C00
resource OSD_CS 1 E11
resource GYRO_EXTI 1 C05
resource GYRO_EXTI 2 B11
resource GYRO_CS 1 A04
resource GYRO_CS 2 B12

# timer
timer A08 AF1
# pin A08: TIM1 CH1 (AF1)
timer A03 AF1
# pin A03: TIM2 CH4 (AF1)
timer E05 AF4
# pin E05: TIM15 CH1 (AF4)
timer A00 AF2
# pin A00: TIM5 CH1 (AF2)
timer A01 AF2
# pin A01: TIM5 CH2 (AF2)
timer B00 AF2
# pin B00: TIM3 CH3 (AF2)
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer B06 AF2
# pin B06: TIM4 CH1 (AF2)
timer B07 AF2
# pin B07: TIM4 CH2 (AF2)
timer C08 AF3
# pin C08: TIM8 CH3 (AF3)
timer C09 AF3
# pin C09: TIM8 CH4 (AF3)

# dma
dma ADC 1 8
# ADC 1: DMA2 Stream 0 Request 9
dma ADC 3 9
# ADC 3: DMA2 Stream 1 Request 115
dma TIMUP 1 0
# TIMUP 1: DMA1 Stream 0 Request 15
dma TIMUP 2 0
# TIMUP 2: DMA1 Stream 0 Request 22
dma TIMUP 3 0
# TIMUP 3: DMA1 Stream 0 Request 27
dma TIMUP 4 1
# TIMUP 4: DMA1 Stream 1 Request 32
dma TIMUP 5 0
# TIMUP 5: DMA1 Stream 0 Request 59
dma TIMUP 8 4
# TIMUP 8: DMA1 Stream 4 Request 51
dma pin A08 10
# pin A08: DMA2 Stream 2 Request 11
dma pin A03 0
# pin A03: DMA1 Stream 0 Request 21
dma pin E05 0
# pin E05: DMA1 Stream 0 Request 105
dma pin A00 0
# pin A00: DMA1 Stream 0 Request 55
dma pin A01 1
# pin A01: DMA1 Stream 1 Request 56
dma pin B00 2
# pin B00: DMA1 Stream 2 Request 25
dma pin B01 3
# pin B01: DMA1 Stream 3 Request 26
dma pin B06 4
# pin B06: DMA1 Stream 4 Request 29
dma pin B07 5
# pin B07: DMA1 Stream 5 Request 30
dma pin C08 6
# pin C08: DMA1 Stream 6 Request 49
dma pin C09 7
# pin C09: DMA1 Stream 7 Request 50

# feature
feature RX_SERIAL
feature TELEMETRY
feature RSSI_ADC
feature LED_STRIP
feature OSD

# master
set gyro_to_use = BOTH
set mag_bustype = I2C
set mag_i2c_device = 1
set baro_bustype = I2C
set baro_i2c_device = 1
set blackbox_device = SPIFLASH
set current_meter = ADC
set battery_meter = ADC
set beeper_inversion = ON
set beeper_od = OFF
set max7456_spi_bus = 4
set dashboard_i2c_bus = 1
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_2_spibus = 2
set gyro_2_sensor_align = CW90
set gyro_2_align_yaw = 900

*/
