/*==============================================================================
Generated on: 2025-04-16 00:58:15.194685
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: FRSKYF4
Manufacturer ID: FRSK

//copy this line to madflight.ino to use this flight controller
#include <madflight_zzz_FRSK-FRSKYF4.h>

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out, or use madflight_config to override.
==============================================================================*/

#define MF_BOARD_NAME "BETAFLIGHT-FRSK-FRSKYF4"
#define MF_MCU_NAME "STM32F405"

const char* madflight_board = R""(
imu_bus_type SPI
imu_gizmo MPU6000 // #define USE_GYRO_SPI_MPU6000
bar_gizmo SPI_BMP280 // #define USE_BARO_SPI_BMP280
// resource BEEPER 1 B04
pin_out0 PB0 // resource MOTOR 1 B00
pin_out1 PB1 // resource MOTOR 2 B01
pin_out2 PA3 // resource MOTOR 3 A03
pin_out3 PA2 // resource MOTOR 4 A02
pin_out4 PA1 // resource MOTOR 5 A01
pin_out5 PA8 // resource MOTOR 6 A08
pin_rcl_ppm PB8 // resource PPM 1 B08
// resource PWM 1 B08
// resource PWM 2 B09
// resource PWM 3 C06
// resource PWM 4 C07
// resource PWM 5 C08
// resource PWM 6 C09
// resource LED_STRIP 1 B06
pin_ser0_tx PA9 // resource SERIAL_TX 1 A09
pin_ser2_tx PB10 // resource SERIAL_TX 3 B10
pin_ser5_tx PC6 // resource SERIAL_TX 6 C06
pin_ser0_rx PA10 // resource SERIAL_RX 1 A10
pin_ser2_rx PB11 // resource SERIAL_RX 3 B11
pin_ser5_rx PC7 // resource SERIAL_RX 6 C07
pin_ser5_inv PC8 // resource INVERTER 6 C08
pin_led PB5 // resource LED 1 B05
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi2_sclk PC10 // resource SPI_SCK 3 C10
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PB14 // resource SPI_MISO 2 B14
pin_spi2_miso PC11 // resource SPI_MISO 3 C11
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi1_mosi PB15 // resource SPI_MOSI 2 B15
pin_spi2_mosi PC12 // resource SPI_MOSI 3 C12
// resource ESCSERIAL 1 B08
pin_bat_v PC2 // resource ADC_BATT 1 C02
bat_gizmo ADC
pin_bat_i PC1 // resource ADC_CURR 1 C01
bat_gizmo ADC
// resource BARO_CS 1 B03
pin_bbx_cs PB12 // resource SDCARD_CS 1 B12
bbx_gizmo SDSPI
// resource SDCARD_DETECT 1 B07
// resource OSD_CS 1 A15
pin_imu_int PC4 // resource GYRO_EXTI 1 C04
pin_imu_cs PA4 // resource GYRO_CS 1 A04
// resource USB_DETECT 1 C05
// set baro_spi_device = 3
// set serialrx_provider = SBUS
// set blackbox_device = SDCARD
// set beeper_inversion = ON
// set beeper_od = OFF
// set sdcard_detect_inverted = ON
// set sdcard_mode = SPI
bbx_spi_bus 1 // set sdcard_spi_bus = 2
// set system_hse_mhz = 8
// set max7456_spi_bus = 3
// set gyro_1_bustype = SPI
imu_spi_bus 0 // set gyro_1_spibus = 1
imu_align CW270 // set gyro_1_sensor_align = CW270
// set gyro_1_align_yaw = 2700
)""; //end of madflight_board


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F405 (S405) 4.2.0 Dec  6 2019 / 00:42:14 (b18478f43e) MSP API: 1.43

#mcu STM32F405

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_BARO
#define USE_BARO_SPI_BMP280
#define USE_MAX7456
#define USE_SDCARD

board_name FRSKYF4
manufacturer_id FRSK

# resources
resource BEEPER 1 B04
resource MOTOR 1 B00
resource MOTOR 2 B01
resource MOTOR 3 A03
resource MOTOR 4 A02
resource MOTOR 5 A01
resource MOTOR 6 A08
resource PPM 1 B08
resource PWM 1 B08
resource PWM 2 B09
resource PWM 3 C06
resource PWM 4 C07
resource PWM 5 C08
resource PWM 6 C09
resource LED_STRIP 1 B06
resource SERIAL_TX 1 A09
resource SERIAL_TX 3 B10
resource SERIAL_TX 6 C06
resource SERIAL_RX 1 A10
resource SERIAL_RX 3 B11
resource SERIAL_RX 6 C07
resource INVERTER 6 C08
resource LED 1 B05
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 C10
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 C11
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 C12
resource ESCSERIAL 1 B08
resource ADC_BATT 1 C02
resource ADC_CURR 1 C01
resource BARO_CS 1 B03
resource SDCARD_CS 1 B12
resource SDCARD_DETECT 1 B07
resource OSD_CS 1 A15
resource GYRO_EXTI 1 C04
resource GYRO_CS 1 A04
resource USB_DETECT 1 C05

# timer
timer B08 AF2
# pin B08: TIM4 CH3 (AF2)
timer B09 AF2
# pin B09: TIM4 CH4 (AF2)
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
timer A08 AF1
# pin A08: TIM1 CH1 (AF1)
timer B06 AF2
# pin B06: TIM4 CH1 (AF2)

# dma
dma SPI_TX 2 0
# SPI_TX 2: DMA1 Stream 4 Channel 0
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin B08 0
# pin B08: DMA1 Stream 7 Channel 2
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
dma pin A08 0
# pin A08: DMA2 Stream 6 Channel 0
dma pin B06 0
# pin B06: DMA1 Stream 0 Channel 2

# feature
feature RX_SERIAL
feature TELEMETRY

# serial
serial 0 64 115200 57600 0 115200
serial 5 32 115200 57600 0 115200

# master
set baro_spi_device = 3
set serialrx_provider = SBUS
set blackbox_device = SDCARD
set beeper_inversion = ON
set beeper_od = OFF
set sdcard_detect_inverted = ON
set sdcard_mode = SPI
set sdcard_spi_bus = 2
set system_hse_mhz = 8
set max7456_spi_bus = 3
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW270
set gyro_1_align_yaw = 2700

*/
