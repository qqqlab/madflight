/*==============================================================================
Generated on: 2025-04-16 00:58:15.234822
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: KAKUTEH7V2
Manufacturer ID: HBRO

//copy this line to madflight.ino to use this flight controller
#include <madflight_zzz_HBRO-KAKUTEH7V2.h>

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out, or use madflight_config to override.
==============================================================================*/

#define MF_BOARD_NAME "BETAFLIGHT-HBRO-KAKUTEH7V2"
#define MF_MCU_NAME "STM32H743"

const char* madflight_board = R""(
imu_bus_type SPI
bar_gizmo BMP280 // #define USE_BARO_BMP280
// resource BEEPER 1 C13
pin_out0 PB0 // resource MOTOR 1 B00
pin_out1 PB1 // resource MOTOR 2 B01
pin_out2 PB3 // resource MOTOR 3 B03
pin_out3 PB10 // resource MOTOR 4 B10
pin_out4 PA0 // resource MOTOR 5 A00
pin_out5 PA2 // resource MOTOR 6 A02
pin_out6 PC8 // resource MOTOR 7 C08
pin_out7 PC9 // resource MOTOR 8 C09
// resource LED_STRIP 1 D12
pin_ser0_tx PA9 // resource SERIAL_TX 1 A09
pin_ser1_tx PD5 // resource SERIAL_TX 2 D05
pin_ser2_tx PD8 // resource SERIAL_TX 3 D08
pin_ser3_tx PD1 // resource SERIAL_TX 4 D01
pin_ser5_tx PC6 // resource SERIAL_TX 6 C06
pin_ser0_rx PA10 // resource SERIAL_RX 1 A10
pin_ser1_rx PD6 // resource SERIAL_RX 2 D06
pin_ser2_rx PD9 // resource SERIAL_RX 3 D09
pin_ser3_rx PD0 // resource SERIAL_RX 4 D00
pin_ser5_rx PC7 // resource SERIAL_RX 6 C07
pin_ser6_rx PE7 // resource SERIAL_RX 7 E07
pin_i2c0_scl PB6 // resource I2C_SCL 1 B06
pin_i2c0_sda PB7 // resource I2C_SDA 1 B07
pin_led PC2 // resource LED 1 C02
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi3_sclk PE2 // resource SPI_SCK 4 E02
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PB14 // resource SPI_MISO 2 B14
pin_spi3_miso PE5 // resource SPI_MISO 4 E05
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi1_mosi PB15 // resource SPI_MOSI 2 B15
pin_spi3_mosi PE6 // resource SPI_MOSI 4 E06
// resource CAMERA_CONTROL 1 E09
pin_bat_v PC0 // resource ADC_BATT 1 C00
bat_gizmo ADC
// resource ADC_RSSI 1 C05
pin_bat_i PC1 // resource ADC_CURR 1 C01
bat_gizmo ADC
// resource FLASH_CS 1 A04
// resource PINIO 1 E13
// resource PINIO 2 B11
// resource OSD_CS 1 B12
pin_imu_int PE1 // resource GYRO_EXTI 1 E01
pin_imu_cs PE4 // resource GYRO_CS 1 E04
// resource USB_DETECT 1 A08
// set mag_bustype = I2C
mag_i2c_bus 0 // set mag_i2c_device = 1
// set baro_bustype = I2C
bar_i2c_bus 0 // set baro_i2c_device = 1
// set serialrx_provider = SBUS
// set blackbox_device = SPIFLASH
// set current_meter = ADC
// set battery_meter = ADC
// set vbat_scale = 109
// set ibata_scale = 168
// set beeper_inversion = ON
// set beeper_od = OFF
// set flash_spi_bus = 1
// set max7456_spi_bus = 2
// set dashboard_i2c_bus = 1
// set pinio_config = 129,129,1,1
// set pinio_box = 0,40,255,255
// set gyro_1_bustype = SPI
imu_spi_bus 3 // set gyro_1_spibus = 4
)""; //end of madflight_board


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32H743 (SH74) 4.3.0 Apr 25 2022 / 01:08:20 (9d71184) MSP API: 1.44

#mcu STM32H743

#define USE_ACC
#define USE_GYRO
#define USE_ACCGYRO_BMI270
#define USE_BARO
#define USE_BARO_BMP280
#define USE_FLASH
#define USE_FLASH_W25N01G
#define USE_MAX7456

board_name KAKUTEH7V2
manufacturer_id  HBRO

# resources
resource BEEPER 1 C13
resource MOTOR 1 B00
resource MOTOR 2 B01
resource MOTOR 3 B03
resource MOTOR 4 B10
resource MOTOR 5 A00
resource MOTOR 6 A02
resource MOTOR 7 C08
resource MOTOR 8 C09
resource LED_STRIP 1 D12
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 D05
resource SERIAL_TX 3 D08
resource SERIAL_TX 4 D01
resource SERIAL_TX 6 C06
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 D06
resource SERIAL_RX 3 D09
resource SERIAL_RX 4 D00
resource SERIAL_RX 6 C07
resource SERIAL_RX 7 E07
resource I2C_SCL 1 B06
resource I2C_SDA 1 B07
resource LED 1 C02
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 4 E02
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 4 E05
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 4 E06
resource CAMERA_CONTROL 1 E09
resource ADC_BATT 1 C00
resource ADC_RSSI 1 C05
resource ADC_CURR 1 C01
resource FLASH_CS 1 A04
resource PINIO 1 E13
resource PINIO 2 B11
resource OSD_CS 1 B12
resource GYRO_EXTI 1 E01
resource GYRO_CS 1 E04
resource USB_DETECT 1 A08

# timer
timer B00 AF2
# pin B00: TIM3 CH3 (AF2)
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer B03 AF1
# pin B03: TIM2 CH2 (AF1)
timer B10 AF1
# pin B10: TIM2 CH3 (AF1)
timer A00 AF2
# pin A00: TIM5 CH1 (AF2)
timer A02 AF2
# pin A02: TIM5 CH3 (AF2)
timer C08 AF3
# pin C08: TIM8 CH3 (AF3)
timer C09 AF3
# pin C09: TIM8 CH4 (AF3)
timer D12 AF2
# pin D12: TIM4 CH1 (AF2)
timer E09 AF1
# pin E09: TIM1 CH1 (AF1)

# dma
dma SPI_TX 1 13
# SPI_TX 1: DMA2 Stream 5 Request 38
dma ADC 1 8
# ADC 1: DMA2 Stream 0 Request 9
dma ADC 3 9
# ADC 3: DMA2 Stream 1 Request 115
dma TIMUP 1 0
# TIMUP 1: DMA1 Stream 0 Request 15
dma TIMUP 2 0
# TIMUP 2: DMA1 Stream 0 Request 22
dma TIMUP 3 2
# TIMUP 3: DMA1 Stream 2 Request 27
dma TIMUP 4 0
# TIMUP 4: DMA1 Stream 0 Request 32
dma TIMUP 5 0
# TIMUP 5: DMA1 Stream 0 Request 59
dma TIMUP 8 1
# TIMUP 8: DMA1 Stream 1 Request 51
dma pin B00 0
# pin B00: DMA1 Stream 0 Request 25
dma pin B01 1
# pin B01: DMA1 Stream 1 Request 26
dma pin B03 2
# pin B03: DMA1 Stream 2 Request 19
dma pin B10 3
# pin B10: DMA1 Stream 3 Request 20
dma pin A00 4
# pin A00: DMA1 Stream 4 Request 55
dma pin A02 5
# pin A02: DMA1 Stream 5 Request 57
dma pin C08 6
# pin C08: DMA1 Stream 6 Request 49
dma pin C09 7
# pin C09: DMA1 Stream 7 Request 50
dma pin D12 14
# pin D12: DMA2 Stream 6 Request 29
dma pin E09 12
# pin E09: DMA2 Stream 4 Request 11

# feature
feature RX_SERIAL
feature TELEMETRY
feature OSD

# serial
serial 0 1 115200 57600 0 115200
serial 1 1 115200 57600 0 115200
serial 5 64 115200 57600 0 115200
serial 6 1024 115200 57600 0 115200

# master
set mag_bustype = I2C
set mag_i2c_device = 1
set baro_bustype = I2C
set baro_i2c_device = 1
set serialrx_provider = SBUS
set blackbox_device = SPIFLASH
set current_meter = ADC
set battery_meter = ADC
set vbat_scale = 109
set ibata_scale = 168
set beeper_inversion = ON
set beeper_od = OFF
set flash_spi_bus = 1
set max7456_spi_bus = 2
set dashboard_i2c_bus = 1
set pinio_config = 129,129,1,1
set pinio_box = 0,40,255,255
set gyro_1_bustype = SPI
set gyro_1_spibus = 4

*/
