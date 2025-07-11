/*==============================================================================
Generated on: 2025-06-11 20:35:54.185055
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: ZEEZF7V2
Manufacturer ID: ZEEZ

//copy this line to madflight.ino to use this flight controller
#define MF_BOARD "brd/betaflight/ZEEZ-ZEEZF7V2.h"

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out, or use madflight_config to override.
==============================================================================*/

#define MF_BOARD_NAME "BETAFLIGHT-ZEEZ-ZEEZF7V2"
#define MF_MCU_NAME "STM32F7X2"

const char madflight_board[] = R""(
imu_bus_type SPI
imu_gizmo MPU6000 // #define USE_GYRO_SPI_MPU6000
bar_gizmo DPS310 // #define USE_BARO_DPS310
bar_gizmo BMP280 // #define USE_BARO_BMP280
// resource BEEPER 1 B02
pin_out0 PB7 // resource MOTOR 1 B07
pin_out1 PB6 // resource MOTOR 2 B06
pin_out2 PB4 // resource MOTOR 3 B04
pin_out3 PB3 // resource MOTOR 4 B03
pin_out4 PC8 // resource MOTOR 5 C08
pin_out5 PC7 // resource MOTOR 6 C07
pin_out6 PC6 // resource MOTOR 7 C06
pin_out7 PB1 // resource MOTOR 8 B01
// resource LED_STRIP 1 B00
pin_ser0_tx PA9 // resource SERIAL_TX 1 A09
pin_ser1_tx PA2 // resource SERIAL_TX 2 A02
pin_ser2_tx PB10 // resource SERIAL_TX 3 B10
pin_ser3_tx PA0 // resource SERIAL_TX 4 A00
pin_ser4_tx PC12 // resource SERIAL_TX 5 C12
pin_ser0_rx PA10 // resource SERIAL_RX 1 A10
pin_ser1_rx PA3 // resource SERIAL_RX 2 A03
pin_ser2_rx PB11 // resource SERIAL_RX 3 B11
pin_ser3_rx PA1 // resource SERIAL_RX 4 A01
pin_ser4_rx PD2 // resource SERIAL_RX 5 D02
pin_i2c0_scl PB8 // resource I2C_SCL 1 B08
pin_i2c2_scl PA8 // resource I2C_SCL 3 A08
pin_i2c0_sda PB9 // resource I2C_SDA 1 B09
pin_i2c2_sda PC9 // resource I2C_SDA 3 C09
pin_led PC14 // resource LED 1 C14
// resource LED 2 C15
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi2_sclk PC10 // resource SPI_SCK 3 C10
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PC2 // resource SPI_MISO 2 C02
pin_spi2_miso PC11 // resource SPI_MISO 3 C11
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi1_mosi PC3 // resource SPI_MOSI 2 C03
pin_spi2_mosi PB5 // resource SPI_MOSI 3 B05
// resource CAMERA_CONTROL 1 B15
pin_bat_v PC0 // resource ADC_BATT 1 C00
bat_gizmo ADC
pin_bat_i PC1 // resource ADC_CURR 1 C01
bat_gizmo ADC
// resource FLASH_CS 1 B12
// resource OSD_CS 1 A15
pin_imu_int PC4 // resource GYRO_EXTI 1 C04
pin_imu_cs PA4 // resource GYRO_CS 1 A04
// set mag_bustype = I2C
mag_i2c_bus 2 // set mag_i2c_device = 3
// set baro_bustype = I2C
bar_i2c_bus 0 // set baro_i2c_device = 1
// set serialrx_provider = CRSF
// set blackbox_device = SPIFLASH
// set beeper_inversion = ON
// set beeper_od = OFF
// set current_meter = ADC
// set battery_meter = ADC
// set ibata_scale = 250
// set beeper_inversion = ON
// set beeper_od = OFF
// set osd_core_temp_alarm = 85
// set max7456_spi_bus = 3
// set flash_spi_bus = 2
// set gyro_1_bustype = SPI
imu_spi_bus 0 // set gyro_1_spibus = 1
)""; //end of madflight_board


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F7X2 (S7X2) 4.2.6 Jan  5 2021 / 19:08:42 (a4b6db1e7) MSP API: 1.43

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_BARO
#define USE_BARO_DPS310
#define USE_BARO_BMP280
#define USE_FLASH
#define USE_FLASH_M25P16
#define USE_MAX7456
#define USE_CAMERA_CONTROL

board_name ZEEZF7V2
manufacturer_id ZEEZ

# resources
resource BEEPER 1 B02
resource MOTOR 1 B07
resource MOTOR 2 B06
resource MOTOR 3 B04
resource MOTOR 4 B03
resource MOTOR 5 C08
resource MOTOR 6 C07
resource MOTOR 7 C06
resource MOTOR 8 B01
resource LED_STRIP 1 B00
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 A02
resource SERIAL_TX 3 B10
resource SERIAL_TX 4 A00
resource SERIAL_TX 5 C12
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 A03
resource SERIAL_RX 3 B11
resource SERIAL_RX 4 A01
resource SERIAL_RX 5 D02
resource I2C_SCL 1 B08
resource I2C_SCL 3 A08
resource I2C_SDA 1 B09
resource I2C_SDA 3 C09
resource LED 1 C14
resource LED 2 C15
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 C10
resource SPI_MISO 1 A06
resource SPI_MISO 2 C02
resource SPI_MISO 3 C11
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 C03
resource SPI_MOSI 3 B05
resource CAMERA_CONTROL 1 B15
resource ADC_BATT 1 C00
resource ADC_CURR 1 C01
resource FLASH_CS 1 B12
resource OSD_CS 1 A15
resource GYRO_EXTI 1 C04
resource GYRO_CS 1 A04

# timer
timer B07 AF2
# pin B07: TIM4 CH2 (AF2)
timer B06 AF2
# pin B06: TIM4 CH1 (AF2)
timer B04 AF2
# pin B04: TIM3 CH1 (AF2)
timer B03 AF1
# pin B03: TIM2 CH2 (AF1)
timer C08 AF2
# pin C08: TIM3 CH3 (AF2)
timer C07 AF2
# pin C07: TIM3 CH2 (AF2)
timer C06 AF3
# pin C06: TIM8 CH1 (AF3)
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer B00 AF1
# pin B00: TIM1 CH2N (AF1)
timer B15 AF9
# pin B15: TIM12 CH2 (AF9)

# dma
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin B07 0
# pin B07: DMA1 Stream 3 Channel 2
dma pin B06 0
# pin B06: DMA1 Stream 0 Channel 2
dma pin B04 0
# pin B04: DMA1 Stream 4 Channel 5
dma pin B03 0
# pin B03: DMA1 Stream 6 Channel 3
dma pin C08 0
# pin C08: DMA1 Stream 7 Channel 5
dma pin C07 0
# pin C07: DMA1 Stream 5 Channel 5
dma pin C06 1
# pin C06: DMA2 Stream 2 Channel 7
dma pin B01 0
# pin B01: DMA1 Stream 2 Channel 5
dma pin B00 0
# pin B00: DMA2 Stream 6 Channel 0

# feature
feature RX_SERIAL
feature LED_STRIP
feature OSD

# serial
serial 3 64 115200 57600 0 115200

# led
led 0 0,15::C:6
led 1 1,15::C:6
led 2 2,15::C:6
led 3 3,15::C:1
led 4 4,15::C:1
led 5 5,15::C:1
led 6 6,15::C:1
led 7 7,15::C:2
led 8 8,15::C:2
led 9 9,15::C:2
led 10 10,15::C:2

# master
set mag_bustype = I2C
set mag_i2c_device = 3
set baro_bustype = I2C
set baro_i2c_device = 1
set serialrx_provider = CRSF
set blackbox_device = SPIFLASH
set beeper_inversion = ON
set beeper_od = OFF
set current_meter = ADC
set battery_meter = ADC
set ibata_scale = 250
set beeper_inversion = ON
set beeper_od = OFF
set osd_core_temp_alarm = 85
set max7456_spi_bus = 3
set flash_spi_bus = 2
set gyro_1_bustype = SPI
set gyro_1_spibus = 1

*/
