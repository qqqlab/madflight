/*==============================================================================
Generated on: 2025-04-16 00:58:15.287762
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: VRRACE
Manufacturer ID: LEGA

//copy this line to madflight.ino to use this flight controller
#include <madflight_zzz_LEGA-VRRACE.h>

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out, or use madflight_config to override.
==============================================================================*/

#define MF_BOARD_NAME "BETAFLIGHT-LEGA-VRRACE"
#define MF_MCU_NAME "STM32F405"

const char* madflight_board = R""(
imu_bus_type SPI
imu_gizmo MPU6500 // #define USE_GYRO_SPI_MPU6500
// resource BEEPER 1 A00
pin_out0 PA1 // resource MOTOR 1 A01
pin_out1 PA2 // resource MOTOR 2 A02
pin_out2 PA3 // resource MOTOR 3 A03
pin_out3 PB5 // resource MOTOR 4 B05
pin_out4 PB0 // resource MOTOR 5 B00
pin_out5 PB1 // resource MOTOR 6 B01
pin_rcl_ppm PE9 // resource PPM 1 E09
// resource PWM 1 E09
// resource PWM 2 E11
// resource PWM 3 E13
// resource PWM 4 E14
// resource PWM 5 E06
// resource PWM 6 E07
pin_ser0_tx PB6 // resource SERIAL_TX 1 B06
pin_ser1_tx PD5 // resource SERIAL_TX 2 D05
pin_ser2_tx PD8 // resource SERIAL_TX 3 D08
pin_ser5_tx PC6 // resource SERIAL_TX 6 C06
pin_ser10_tx PE11 // resource SERIAL_TX 11 E11
pin_ser0_rx PB7 // resource SERIAL_RX 1 B07
pin_ser1_rx PD6 // resource SERIAL_RX 2 D06
pin_ser2_rx PD9 // resource SERIAL_RX 3 D09
pin_ser5_rx PC7 // resource SERIAL_RX 6 C07
pin_ser10_rx PE13 // resource SERIAL_RX 11 E13
pin_ser5_inv PD7 // resource INVERTER 6 D07
pin_led PD14 // resource LED 1 D14
// resource LED 2 D15
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PB14 // resource SPI_MISO 2 B14
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi1_mosi PB15 // resource SPI_MOSI 2 B15
// resource ESCSERIAL 1 E09
pin_bat_v PC0 // resource ADC_BATT 1 C00
bat_gizmo ADC
// resource ADC_RSSI 1 B01
pin_bat_i PA5 // resource ADC_CURR 1 A05
bat_gizmo ADC
pin_imu_int PD10 // resource GYRO_EXTI 1 D10
pin_imu_cs PE10 // resource GYRO_CS 1 E10
// set battery_meter = ADC
// set beeper_inversion = ON
// set beeper_od = OFF
// set system_hse_mhz = 8
// set gyro_1_bustype = SPI
imu_spi_bus 1 // set gyro_1_spibus = 2
imu_align CW270 // set gyro_1_sensor_align = CW270
// set gyro_1_align_yaw = 2700
)""; //end of madflight_board


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F405 (S405) 4.2.0 Mar  5 2020 / 22:29:55 (c29b125a59) MSP API: 1.43

#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_MAX7456

board_name VRRACE
manufacturer_id LEGA

# resources
resource BEEPER 1 A00
resource MOTOR 1 A01
resource MOTOR 2 A02
resource MOTOR 3 A03
resource MOTOR 4 B05
resource MOTOR 5 B00
resource MOTOR 6 B01
resource PPM 1 E09
resource PWM 1 E09
resource PWM 2 E11
resource PWM 3 E13
resource PWM 4 E14
resource PWM 5 E06
resource PWM 6 E07
resource SERIAL_TX 1 B06
resource SERIAL_TX 2 D05
resource SERIAL_TX 3 D08
resource SERIAL_TX 6 C06
resource SERIAL_TX 11 E11
resource SERIAL_RX 1 B07
resource SERIAL_RX 2 D06
resource SERIAL_RX 3 D09
resource SERIAL_RX 6 C07
resource SERIAL_RX 11 E13
resource INVERTER 6 D07
resource LED 1 D14
resource LED 2 D15
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource ESCSERIAL 1 E09
resource ADC_BATT 1 C00
resource ADC_RSSI 1 B01
resource ADC_CURR 1 A05
resource GYRO_EXTI 1 D10
resource GYRO_CS 1 E10

# timer
timer E09 AF1
# pin E09: TIM1 CH1 (AF1)
timer E11 AF1
# pin E11: TIM1 CH2 (AF1)
timer E13 AF1
# pin E13: TIM1 CH3 (AF1)
timer E14 AF1
# pin E14: TIM1 CH4 (AF1)
timer A01 AF1
# pin A01: TIM2 CH2 (AF1)
timer A02 AF1
# pin A02: TIM2 CH3 (AF1)
timer A03 AF1
# pin A03: TIM2 CH4 (AF1)
timer B05 AF2
# pin B05: TIM3 CH2 (AF2)
timer B00 AF2
# pin B00: TIM3 CH3 (AF2)
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)

# dma
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin E09 0
# pin E09: DMA2 Stream 6 Channel 0
dma pin E11 0
# pin E11: DMA2 Stream 6 Channel 0
dma pin E13 0
# pin E13: DMA2 Stream 6 Channel 0
dma pin E14 0
# pin E14: DMA2 Stream 4 Channel 6
dma pin A01 0
# pin A01: DMA1 Stream 6 Channel 3
dma pin A02 0
# pin A02: DMA1 Stream 1 Channel 3
dma pin A03 0
# pin A03: DMA1 Stream 7 Channel 3
dma pin B05 0
# pin B05: DMA1 Stream 5 Channel 5
dma pin B00 0
# pin B00: DMA1 Stream 7 Channel 5
dma pin B01 0
# pin B01: DMA1 Stream 2 Channel 5

# master
set battery_meter = ADC
set beeper_inversion = ON
set beeper_od = OFF
set system_hse_mhz = 8
set gyro_1_bustype = SPI
set gyro_1_spibus = 2
set gyro_1_sensor_align = CW270
set gyro_1_align_yaw = 2700

*/
