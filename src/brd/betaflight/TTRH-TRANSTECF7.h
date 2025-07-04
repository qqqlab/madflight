/*==============================================================================
Generated on: 2025-06-11 20:35:54.173132
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: TRANSTECF7
Manufacturer ID: TTRH

//copy this line to madflight.ino to use this flight controller
#define MF_BOARD "brd/betaflight/TTRH-TRANSTECF7.h"

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out, or use madflight_config to override.
==============================================================================*/

#define MF_BOARD_NAME "BETAFLIGHT-TTRH-TRANSTECF7"
#define MF_MCU_NAME "STM32F7X2"

const char madflight_board[] = R""(
imu_bus_type SPI
imu_gizmo MPU6000 // #define USE_GYRO_SPI_MPU6000
pin_out0 PB0 // resource MOTOR 1 B00
pin_out1 PB1 // resource MOTOR 2 B01
pin_out2 PC6 // resource MOTOR 3 C06
pin_out3 PC7 // resource MOTOR 4 C07
pin_out4 PB3 // resource MOTOR 5 B03
pin_out5 PB4 // resource MOTOR 6 B04
// resource CAMERA_CONTROL 1 B08
// resource LED_STRIP 1 A15
pin_ser0_tx PA9 // resource SERIAL_TX 1 A09
pin_ser1_tx PA2 // resource SERIAL_TX 2 A02
pin_ser2_tx PC10 // resource SERIAL_TX 3 C10
pin_ser3_tx PA0 // resource SERIAL_TX 4 A00
pin_ser4_tx PC12 // resource SERIAL_TX 5 C12
pin_ser0_rx PA10 // resource SERIAL_RX 1 A10
pin_ser1_rx PA3 // resource SERIAL_RX 2 A03
pin_ser2_rx PC11 // resource SERIAL_RX 3 C11
pin_ser3_rx PA1 // resource SERIAL_RX 4 A01
pin_ser4_rx PD2 // resource SERIAL_RX 5 D02
pin_i2c0_scl PB6 // resource I2C_SCL 1 B06
pin_i2c0_sda PB7 // resource I2C_SDA 1 B07
pin_led PA14 // resource LED 1 A14
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PB14 // resource SPI_MISO 2 B14
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi1_mosi PB15 // resource SPI_MOSI 2 B15
pin_bat_v PC0 // resource ADC_BATT 1 C00
bat_gizmo ADC
// resource ADC_RSSI 1 B05
pin_bat_i PC1 // resource ADC_CURR 1 C01
bat_gizmo ADC
// resource PINIO 1 B12
// resource OSD_CS 1 B10
pin_imu_int PC3 // resource GYRO_EXTI 1 C03
pin_imu_cs PC2 // resource GYRO_CS 1 C02
// resource USB_DETECT 1 A04
// set mag_hardware = NONE
// set baro_hardware = NONE
// set serialrx_provider = SBUS
// set blackbox_device = NONE
// set dshot_burst = ON
// set current_meter = ADC
// set battery_meter = ADC
// set max7456_spi_bus = 2
imu_spi_bus 0 // set gyro_1_spibus = 1
imu_align CW180FLIP // set gyro_1_sensor_align = CW180FLIP
// set gyro_2_spibus = 1
// set pinio_box = 40,255,255,255
)""; //end of madflight_board


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F7X2 (S7X2) 4.0.2 May  5 2019 / 12:20:37 (56bdc8d26) MSP API: 1.41

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACC_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_MPU6000
#define USE_MAX7456

board_name TRANSTECF7
manufacturer_id TTRH

# name

# resources
resource MOTOR 1 B00
resource MOTOR 2 B01
resource MOTOR 3 C06
resource MOTOR 4 C07
resource MOTOR 5 B03
resource MOTOR 6 B04
resource CAMERA_CONTROL 1 B08
resource LED_STRIP 1 A15
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 A02
resource SERIAL_TX 3 C10
resource SERIAL_TX 4 A00
resource SERIAL_TX 5 C12
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 A03
resource SERIAL_RX 3 C11
resource SERIAL_RX 4 A01
resource SERIAL_RX 5 D02
resource I2C_SCL 1 B06
resource I2C_SDA 1 B07
resource LED 1 A14
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource ADC_BATT 1 C00
resource ADC_RSSI 1 B05
resource ADC_CURR 1 C01
resource PINIO 1 B12
resource OSD_CS 1 B10
resource GYRO_EXTI 1 C03
resource GYRO_CS 1 C02
resource USB_DETECT 1 A04

# timer
timer B00 AF2
# pin B00: TIM3 CH3 (AF2)
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer C06 AF3
# pin C06: TIM8 CH1 (AF3)
timer C07 AF3
# pin C07: TIM8 CH2 (AF3)
timer B03 AF1
# pin B03: TIM2 CH2 (AF1)
timer B04 AF2
# pin B04: TIM3 CH1 (AF2)
timer B08 AF2
# pin B08: TIM4 CH3 (AF2)
timer A15 AF1
# pin A15: TIM2 CH1 (AF1)

# dma
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin B00 0
# pin B00: DMA1 Stream 7 Channel 5
dma pin B01 0
# pin B01: DMA1 Stream 2 Channel 5
dma pin C06 0
# pin C06: DMA2 Stream 2 Channel 0
dma pin C07 0
# pin C07: DMA2 Stream 2 Channel 0
dma pin B03 0
# pin B03: DMA1 Stream 6 Channel 3
dma pin B04 0
# pin B04: DMA1 Stream 4 Channel 5
dma pin B08 0
# pin B08: DMA1 Stream 7 Channel 2
dma pin A15 0
# pin A15: DMA1 Stream 5 Channel 3

# feature
feature RX_SERIAL
feature OSD

# serial
serial 0 64 115200 57600 0 115200
serial 4 2048 115200 57600 0 115200

# display_name
display_name = TransTEC

# master
set mag_hardware = NONE
set baro_hardware = NONE
set serialrx_provider = SBUS
set blackbox_device = NONE
set dshot_burst = ON
set current_meter = ADC
set battery_meter = ADC
set max7456_spi_bus = 2
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW180FLIP
set gyro_2_spibus = 1
set pinio_box = 40,255,255,255

*/
