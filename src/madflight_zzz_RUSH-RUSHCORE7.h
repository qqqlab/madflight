/*==============================================================================
Generated on: 2025-04-16 00:58:15.344964
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: RUSHCORE7
Manufacturer ID: RUSH

//copy this line to madflight.ino to use this flight controller
#include <madflight_zzz_RUSH-RUSHCORE7.h>

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out, or use madflight_config to override.
==============================================================================*/

#define MF_BOARD_NAME "BETAFLIGHT-RUSH-RUSHCORE7"
#define MF_MCU_NAME "STM32F7X2"

const char* madflight_board = R""(
imu_bus_type SPI
imu_gizmo MPU6500 // #define USE_GYRO_SPI_MPU6500
imu_gizmo MPU6000 // #define USE_GYRO_SPI_MPU6000
// resource BEEPER 1 B01
pin_out0 PC8 // resource MOTOR 1 C08
pin_out1 PC6 // resource MOTOR 2 C06
pin_out2 PC9 // resource MOTOR 3 C09
pin_out3 PC7 // resource MOTOR 4 C07
pin_out4 PA8 // resource MOTOR 5 A08
pin_out5 PA9 // resource MOTOR 6 A09
pin_rcl_ppm PA3 // resource PPM 1 A03
// resource LED_STRIP 1 B11
pin_ser0_tx PB6 // resource SERIAL_TX 1 B06
pin_ser1_tx PA2 // resource SERIAL_TX 2 A02
pin_ser2_tx PC10 // resource SERIAL_TX 3 C10
pin_ser3_tx PA0 // resource SERIAL_TX 4 A00
pin_ser4_tx PC12 // resource SERIAL_TX 5 C12
pin_ser0_rx PB7 // resource SERIAL_RX 1 B07
pin_ser1_rx PA3 // resource SERIAL_RX 2 A03
pin_ser2_rx PC11 // resource SERIAL_RX 3 C11
pin_ser3_rx PA1 // resource SERIAL_RX 4 A01
pin_ser4_rx PD2 // resource SERIAL_RX 5 D02
pin_i2c0_scl PB8 // resource I2C_SCL 1 B08
pin_i2c0_sda PB9 // resource I2C_SDA 1 B09
pin_led PC13 // resource LED 1 C13
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi2_sclk PB3 // resource SPI_SCK 3 B03
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PB14 // resource SPI_MISO 2 B14
pin_spi2_miso PB4 // resource SPI_MISO 3 B04
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi1_mosi PB15 // resource SPI_MOSI 2 B15
pin_spi2_mosi PB5 // resource SPI_MOSI 3 B05
// resource CAMERA_CONTROL 1 B00
pin_bat_v PC1 // resource ADC_BATT 1 C01
bat_gizmo ADC
// resource ADC_RSSI 1 A00
pin_bat_i PC3 // resource ADC_CURR 1 C03
bat_gizmo ADC
// resource FLASH_CS 1 C15
// resource OSD_CS 1 B12
pin_imu_int PC4 // resource GYRO_EXTI 1 C04
pin_imu_cs PA4 // resource GYRO_CS 1 A04
// set mag_bustype = I2C
mag_i2c_bus 0 // set mag_i2c_device = 1
// set baro_bustype = I2C
bar_i2c_bus 0 // set baro_i2c_device = 1
// set adc_device = 3
// set blackbox_device = SPIFLASH
// set dshot_burst = ON
// set current_meter = ADC
// set battery_meter = ADC
// set beeper_inversion = ON
// set beeper_od = OFF
// set max7456_spi_bus = 2
// set flash_spi_bus = 3
imu_spi_bus 0 // set gyro_1_spibus = 1
imu_align CW270FLIP // set gyro_1_sensor_align = CW270FLIP
// set gyro_1_align_pitch = 1800
// set gyro_1_align_yaw = 2700
)""; //end of madflight_board


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F7X2 (S7X2) 4.1.0 Oct 10 2019 / 02:46:26 (e78f9761b) MSP API: 1.42

#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_ACC_SPI_MPU6000
#define USE_ACC_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_MPU6000
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

board_name RUSHCORE7
manufacturer_id RUSH

# resources
resource BEEPER 1 B01
resource MOTOR 1 C08
resource MOTOR 2 C06
resource MOTOR 3 C09
resource MOTOR 4 C07
resource MOTOR 5 A08
resource MOTOR 6 A09
resource PPM 1 A03
resource LED_STRIP 1 B11
resource SERIAL_TX 1 B06
resource SERIAL_TX 2 A02
resource SERIAL_TX 3 C10
resource SERIAL_TX 4 A00
resource SERIAL_TX 5 C12
resource SERIAL_RX 1 B07
resource SERIAL_RX 2 A03
resource SERIAL_RX 3 C11
resource SERIAL_RX 4 A01
resource SERIAL_RX 5 D02
resource I2C_SCL 1 B08
resource I2C_SDA 1 B09
resource LED 1 C13
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 B03
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 B04
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 B05
resource CAMERA_CONTROL 1 B00
resource ADC_BATT 1 C01
resource ADC_RSSI 1 A00
resource ADC_CURR 1 C03
resource FLASH_CS 1 C15
resource OSD_CS 1 B12
resource GYRO_EXTI 1 C04
resource GYRO_CS 1 A04

# timer
timer A03 AF3
# pin A03: TIM9 CH2 (AF3)
timer C08 AF3
# pin C08: TIM8 CH3 (AF3)
timer C06 AF3
# pin C06: TIM8 CH1 (AF3)
timer C09 AF3
# pin C09: TIM8 CH4 (AF3)
timer C07 AF3
# pin C07: TIM8 CH2 (AF3)
timer A08 AF1
# pin A08: TIM1 CH1 (AF1)
timer A09 AF1
# pin A09: TIM1 CH2 (AF1)
timer B11 AF1
# pin B11: TIM2 CH4 (AF1)
timer B00 AF2
# pin B00: TIM3 CH3 (AF2)

# dma
dma ADC 3 0
# ADC 3: DMA2 Stream 0 Channel 2
dma pin C08 0
# pin C08: DMA2 Stream 2 Channel 0
dma pin C06 0
# pin C06: DMA2 Stream 2 Channel 0
dma pin C09 0
# pin C09: DMA2 Stream 7 Channel 7
dma pin C07 0
# pin C07: DMA2 Stream 2 Channel 0
dma pin A08 0
# pin A08: DMA2 Stream 6 Channel 0
dma pin A09 0
# pin A09: DMA2 Stream 6 Channel 0
dma pin B11 0
# pin B11: DMA1 Stream 7 Channel 3
dma pin B00 0
# pin B00: DMA1 Stream 7 Channel 5

# feature
feature OSD

# master
set mag_bustype = I2C
set mag_i2c_device = 1
set baro_bustype = I2C
set baro_i2c_device = 1
set adc_device = 3
set blackbox_device = SPIFLASH
set dshot_burst = ON
set current_meter = ADC
set battery_meter = ADC
set beeper_inversion = ON
set beeper_od = OFF
set max7456_spi_bus = 2
set flash_spi_bus = 3
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW270FLIP
set gyro_1_align_pitch = 1800
set gyro_1_align_yaw = 2700

*/
