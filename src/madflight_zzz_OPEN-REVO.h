/*==============================================================================
Generated on: 2025-04-16 00:58:15.334893
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: REVO
Manufacturer ID: OPEN

//copy this line to madflight.ino to use this flight controller
#include <madflight_zzz_OPEN-REVO.h>

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out, or use madflight_config to override.
==============================================================================*/

#define MF_BOARD_NAME "BETAFLIGHT-OPEN-REVO"
#define MF_MCU_NAME "STM32F405"

const char* madflight_board = R""(
imu_bus_type SPI
imu_gizmo MPU6500 // #define USE_GYRO_SPI_MPU6500
imu_gizmo MPU6000 // #define USE_GYRO_SPI_MPU6000
pin_out0 PB0 // resource MOTOR 1 B00
pin_out1 PB1 // resource MOTOR 2 B01
pin_out2 PA3 // resource MOTOR 3 A03
pin_out3 PA2 // resource MOTOR 4 A02
pin_out4 PA1 // resource MOTOR 5 A01
pin_out5 PA0 // resource MOTOR 6 A00
pin_rcl_ppm PB14 // resource PPM 1 B14
// resource PWM 1 B14
// resource PWM 2 B15
// resource PWM 3 C06
// resource PWM 4 C07
// resource PWM 5 C08
// resource PWM 6 C09
// resource LED_STRIP 1 A01
pin_ser0_tx PA9 // resource SERIAL_TX 1 A09
pin_ser2_tx PB10 // resource SERIAL_TX 3 B10
pin_ser3_tx PA0 // resource SERIAL_TX 4 A00
pin_ser5_tx PC6 // resource SERIAL_TX 6 C06
pin_ser0_rx PA10 // resource SERIAL_RX 1 A10
pin_ser2_rx PB11 // resource SERIAL_RX 3 B11
pin_ser3_rx PA1 // resource SERIAL_RX 4 A01
pin_ser5_rx PC7 // resource SERIAL_RX 6 C07
pin_ser0_inv PC0 // resource INVERTER 1 C00
pin_i2c0_scl PB8 // resource I2C_SCL 1 B08
pin_i2c0_sda PB9 // resource I2C_SDA 1 B09
pin_led PB5 // resource LED 1 B05
// resource LED 2 B04
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi2_sclk PC10 // resource SPI_SCK 3 C10
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi2_miso PC11 // resource SPI_MISO 3 C11
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi2_mosi PC12 // resource SPI_MOSI 3 C12
// resource ESCSERIAL 1 B14
pin_bat_v PC2 // resource ADC_BATT 1 C02
bat_gizmo ADC
pin_bat_i PC1 // resource ADC_CURR 1 C01
bat_gizmo ADC
// resource PINIO 1 C08
// resource FLASH_CS 1 B03
pin_imu_int PC4 // resource GYRO_EXTI 1 C04
pin_imu_cs PA4 // resource GYRO_CS 1 A04
// resource USB_DETECT 1 C05
// set mag_bustype = I2C
mag_i2c_bus 0 // set mag_i2c_device = 1
// set baro_bustype = I2C
bar_i2c_bus 0 // set baro_i2c_device = 1
// set system_hse_mhz = 8
// set dashboard_i2c_bus = 1
// set flash_spi_bus = 3
// set gyro_1_bustype = SPI
imu_spi_bus 0 // set gyro_1_spibus = 1
imu_align CW270 // set gyro_1_sensor_align = CW270
// set gyro_1_align_yaw = 2700
)""; //end of madflight_board


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F405 (S405) 4.1.0 Oct 16 2019 / 11:57:16 (c37a7c91a) MSP API: 1.42

#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_ACC_SPI_MPU6000
#define USE_GYRO_SPI_MPU6000
#define USE_FLASH
#define USE_FLASH_M25P16

board_name REVO
manufacturer_id OPEN

# resources
resource MOTOR 1 B00
resource MOTOR 2 B01
resource MOTOR 3 A03
resource MOTOR 4 A02
resource MOTOR 5 A01
resource MOTOR 6 A00
resource PPM 1 B14
resource PWM 1 B14
resource PWM 2 B15
resource PWM 3 C06
resource PWM 4 C07
resource PWM 5 C08
resource PWM 6 C09
resource LED_STRIP 1 A01
resource SERIAL_TX 1 A09
resource SERIAL_TX 3 B10
resource SERIAL_TX 4 A00
resource SERIAL_TX 6 C06
resource SERIAL_RX 1 A10
resource SERIAL_RX 3 B11
resource SERIAL_RX 4 A01
resource SERIAL_RX 6 C07
resource INVERTER 1 C00
resource I2C_SCL 1 B08
resource I2C_SDA 1 B09
resource LED 1 B05
resource LED 2 B04
resource SPI_SCK 1 A05
resource SPI_SCK 3 C10
resource SPI_MISO 1 A06
resource SPI_MISO 3 C11
resource SPI_MOSI 1 A07
resource SPI_MOSI 3 C12
resource ESCSERIAL 1 B14
resource ADC_BATT 1 C02
resource ADC_CURR 1 C01
resource PINIO 1 C08
resource FLASH_CS 1 B03
resource GYRO_EXTI 1 C04
resource GYRO_CS 1 A04
resource USB_DETECT 1 C05

# timer
timer B14 AF9
# pin B14: TIM12 CH1 (AF9)
timer B15 AF9
# pin B15: TIM12 CH2 (AF9)
timer C06 AF3
# pin C06: TIM8 CH1 (AF3)
timer C07 AF3
# pin C07: TIM8 CH2 (AF3)
timer C08 AF3
# pin C08: TIM8 CH3 (AF3)
timer C09 AF3
# pin C09: TIM8 CH4 (AF3)
timer B00 AF2
# pin B00: TIM3 CH3 (AF2)
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer A03 AF1
# pin A03: TIM2 CH4 (AF1)
timer A02 AF1
# pin A02: TIM2 CH3 (AF1)
timer A01 AF2
# pin A01: TIM5 CH2 (AF2)
timer A00 AF2
# pin A00: TIM5 CH1 (AF2)

# dma
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin C06 0
# pin C06: DMA2 Stream 2 Channel 0
dma pin C07 0
# pin C07: DMA2 Stream 2 Channel 0
dma pin C08 0
# pin C08: DMA2 Stream 2 Channel 0
dma pin C09 0
# pin C09: DMA2 Stream 7 Channel 7
dma pin B00 0
# pin B00: DMA1 Stream 7 Channel 5
dma pin B01 0
# pin B01: DMA1 Stream 2 Channel 5
dma pin A03 1
# pin A03: DMA1 Stream 6 Channel 3
dma pin A02 0
# pin A02: DMA1 Stream 1 Channel 3
dma pin A01 0
# pin A01: DMA1 Stream 4 Channel 6
dma pin A00 0
# pin A00: DMA1 Stream 2 Channel 6

# feature
feature RX_SERIAL

# master
set mag_bustype = I2C
set mag_i2c_device = 1
set baro_bustype = I2C
set baro_i2c_device = 1
set system_hse_mhz = 8
set dashboard_i2c_bus = 1
set flash_spi_bus = 3
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW270
set gyro_1_align_yaw = 2700

*/
