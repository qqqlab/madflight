/*==============================================================================
Generated on: 2025-04-16 00:58:15.208351
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: GEMEF411
Manufacturer ID: GFPV

//copy this line to madflight.ino to use this flight controller
#include <madflight_zzz_GFPV-GEMEF411.h>

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out, or use madflight_config to override.
==============================================================================*/

#define MF_BOARD_NAME "BETAFLIGHT-GFPV-GEMEF411"
#define MF_MCU_NAME "STM32F411"

const char* madflight_board = R""(
imu_bus_type SPI
imu_gizmo ICM42688P // #define USE_GYRO_SPI_ICM42688P
// resource BEEPER 1 C14
pin_out0 PA8 // resource MOTOR 1 A08
pin_out1 PB4 // resource MOTOR 2 B04
pin_out2 PB0 // resource MOTOR 3 B00
pin_out3 PB5 // resource MOTOR 4 B05
pin_out4 PA9 // resource MOTOR 5 A09
pin_out5 PA10 // resource MOTOR 6 A10
pin_rcl_ppm PA2 // resource PPM 1 A02
// resource LED_STRIP 1 A00
pin_ser0_tx PA15 // resource SERIAL_TX 1 A15
pin_ser1_tx PA2 // resource SERIAL_TX 2 A02
pin_ser10_tx PB10 // resource SERIAL_TX 11 B10
pin_ser11_tx PB6 // resource SERIAL_TX 12 B06
pin_ser0_rx PB3 // resource SERIAL_RX 1 B03
pin_ser1_rx PA3 // resource SERIAL_RX 2 A03
pin_ser10_rx PB10 // resource SERIAL_RX 11 B10
pin_ser11_rx PB6 // resource SERIAL_RX 12 B06
pin_i2c0_scl PB8 // resource I2C_SCL 1 B08
pin_i2c0_sda PB9 // resource I2C_SDA 1 B09
pin_led PC13 // resource LED 1 C13
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PB14 // resource SPI_MISO 2 B14
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi1_mosi PB15 // resource SPI_MOSI 2 B15
pin_bat_v PA1 // resource ADC_BATT 1 A01
bat_gizmo ADC
pin_bat_i PB1 // resource ADC_CURR 1 B01
bat_gizmo ADC
// resource FLASH_CS 1 B02
// resource OSD_CS 1 B12
pin_imu_int PC15 // resource GYRO_EXTI 1 C15
pin_imu_cs PA4 // resource GYRO_CS 1 A04
// set dshot_bitbang = ON
// set mag_bustype = I2C
mag_i2c_bus 0 // set mag_i2c_device = 1
// set baro_bustype = I2C
bar_i2c_bus 0 // set baro_i2c_device = 1
// set blackbox_device = SPIFLASH
// set serialrx_provider = SBUS
// set dshot_burst = AUTO
// set current_meter = ADC
// set battery_meter = ADC
// set ibata_scale = 800
// set beeper_inversion = ON
// set beeper_od = OFF
// set system_hse_mhz = 8
// set max7456_spi_bus = 2
// set flash_spi_bus = 2
// set gyro_1_bustype = SPI
imu_spi_bus 0 // set gyro_1_spibus = 1
imu_align CW180 // set gyro_1_sensor_align = CW180
// set gyro_1_align_yaw = 1800
)""; //end of madflight_board


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F411 (S411) 4.3.0 Jun 14 2022 / 00:48:04 (229ac66) MSP API: 1.44

#define USE_GYRO
#define USE_GYRO_SPI_ICM42688P
#define USE_ACC
#define USE_ACC_SPI_ICM42688P
#define USE_ACCGYRO_BMI270
#define USE_FLASH
#define USE_FLASH_M25P16
#define USE_MAX7456

board_name GEMEF411
manufacturer_id GFPV

# resources
resource BEEPER 1 C14
resource MOTOR 1 A08
resource MOTOR 2 B04
resource MOTOR 3 B00
resource MOTOR 4 B05
resource MOTOR 5 A09
resource MOTOR 6 A10
resource PPM 1 A02
resource LED_STRIP 1 A00
resource SERIAL_TX 1 A15
resource SERIAL_TX 2 A02
resource SERIAL_TX 11 B10
resource SERIAL_TX 12 B06
resource SERIAL_RX 1 B03
resource SERIAL_RX 2 A03
resource SERIAL_RX 11 B10
resource SERIAL_RX 12 B06
resource I2C_SCL 1 B08
resource I2C_SDA 1 B09
resource LED 1 C13
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource ADC_BATT 1 A01
resource ADC_CURR 1 B01
resource FLASH_CS 1 B02
resource OSD_CS 1 B12
resource GYRO_EXTI 1 C15
resource GYRO_CS 1 A04

# timer
timer A02 AF3
# pin A02: TIM9 CH1 (AF3)
timer A08 AF1
# pin A08: TIM1 CH1 (AF1)
timer A09 AF1
# pin A09: TIM1 CH2 (AF1)
timer A10 AF1
# pin A10: TIM1 CH3 (AF1)
timer B00 AF2
# pin B00: TIM3 CH3 (AF2)
timer B04 AF2
# pin B04: TIM3 CH1 (AF2)
timer B05 AF2
# pin B05: TIM3 CH2 (AF2)
timer A00 AF2
# pin A00: TIM5 CH1 (AF2)
timer B06 AF2
# pin B06: TIM4 CH1 (AF2)
timer B10 AF1
# pin B10: TIM2 CH3 (AF1)

# dma
dma ADC 1 0
# ADC 1: DMA2 Stream 0 Channel 0
dma pin A08 0
# pin A08: DMA2 Stream 6 Channel 0
dma pin A09 0
# pin A09: DMA2 Stream 6 Channel 0
dma pin A10 0
# pin A10: DMA2 Stream 6 Channel 0
dma pin B00 0
# pin B00: DMA1 Stream 7 Channel 5
dma pin B04 0
# pin B04: DMA1 Stream 4 Channel 5
dma pin B05 0
# pin B05: DMA1 Stream 5 Channel 5
dma pin A00 0
# pin A00: DMA1 Stream 2 Channel 6
dma pin B06 0
# pin B06: DMA1 Stream 0 Channel 2
dma pin B10 0
# pin B10: DMA1 Stream 1 Channel 3

# feature
feature SOFTSERIAL
feature LED_STRIP
feature OSD
feature RX_SERIAL

# serial
serial 1 64 115200 57600 0 115200
serial 30 2048 115200 57600 0 115200

# master
set dshot_bitbang = ON
set mag_bustype = I2C
set mag_i2c_device = 1
set baro_bustype = I2C
set baro_i2c_device = 1
set blackbox_device = SPIFLASH
set serialrx_provider = SBUS
set dshot_burst = AUTO
set current_meter = ADC
set battery_meter = ADC
set ibata_scale = 800
set beeper_inversion = ON
set beeper_od = OFF
set system_hse_mhz = 8
set max7456_spi_bus = 2
set flash_spi_bus = 2
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW180
set gyro_1_align_yaw = 1800

*/
