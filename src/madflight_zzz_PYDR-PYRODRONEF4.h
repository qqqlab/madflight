/*==============================================================================
Generated on: 2025-04-16 00:58:15.334893
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: PYRODRONEF4
Manufacturer ID: PYDR

//copy this line to madflight.ino to use this flight controller
#include <madflight_zzz_PYDR-PYRODRONEF4.h>

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out, or use madflight_config to override.
==============================================================================*/

#define MF_BOARD_NAME "BETAFLIGHT-PYDR-PYRODRONEF4"
#define MF_MCU_NAME "STM32F405"

const char* madflight_board = R""(
imu_bus_type SPI
imu_gizmo MPU6000 // #define USE_GYRO_SPI_MPU6000
// resource BEEPER 1 B05
pin_out0 PB1 // resource MOTOR 1 B01
pin_out1 PB0 // resource MOTOR 2 B00
pin_out2 PC9 // resource MOTOR 3 C09
pin_out3 PA8 // resource MOTOR 4 A08
// resource LED_STRIP 1 B08
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
pin_ser0_inv PC3 // resource INVERTER 1 C03
pin_led PB4 // resource LED 1 B04
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PB14 // resource SPI_MISO 2 B14
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi1_mosi PB15 // resource SPI_MOSI 2 B15
// resource ESCSERIAL 1 B09
// resource CAMERA_CONTROL 1 B09
pin_bat_v PC2 // resource ADC_BATT 1 C02
bat_gizmo ADC
pin_bat_i PC1 // resource ADC_CURR 1 C01
bat_gizmo ADC
// resource PINIO 1 A15
// resource OSD_CS 1 B12
pin_imu_int PC4 // resource GYRO_EXTI 1 C04
pin_imu_cs PA4 // resource GYRO_CS 1 A04
// set mag_hardware = NONE
// set baro_hardware = NONE
// set motor_pwm_protocol = DSHOT600
// set current_meter = ADC
// set battery_meter = ADC
// set vbat_scale = 160
// set ibata_scale = 150
// set beeper_inversion = ON
// set beeper_od = OFF
// set pid_process_denom = 1
// set system_hse_mhz = 8
// set max7456_spi_bus = 2
// set pinio_box = 40,255,255,255
// set gyro_1_bustype = SPI
imu_spi_bus 0 // set gyro_1_spibus = 1
)""; //end of madflight_board


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F405 (S405) 4.1.1 Nov 15 2019 / 12:54:53 (1e5e3d369) MSP API: 1.42

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_MAX7456

board_name PYRODRONEF4
manufacturer_id PYDR

# resources
resource BEEPER 1 B05
resource MOTOR 1 B01
resource MOTOR 2 B00
resource MOTOR 3 C09
resource MOTOR 4 A08
resource LED_STRIP 1 B08
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
resource INVERTER 1 C03
resource LED 1 B04
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource ESCSERIAL 1 B09
resource CAMERA_CONTROL 1 B09
resource ADC_BATT 1 C02
resource ADC_CURR 1 C01
resource PINIO 1 A15
resource OSD_CS 1 B12
resource GYRO_EXTI 1 C04
resource GYRO_CS 1 A04

# timer
timer B09 AF3           # pin B09: TIM11 CH1 (AF3)
timer B01 AF2           # pin B01: TIM3 CH4 (AF2)
timer B00 AF3           # pin B00: TIM8 CH2N (AF3)
timer C09 AF3           # pin C09: TIM8 CH4 (AF3)
timer A08 AF1           # pin A08: TIM1 CH1 (AF1)
timer B08 AF2           # pin B08: TIM4 CH3 (AF2)

# dma
dma ADC 1 1             # ADC 1: DMA2 Stream 4 Channel 0
dma pin B01 0           # pin B01: DMA1 Stream 2 Channel 5
dma pin B00 0           # pin B00: DMA2 Stream 2 Channel 0
dma pin C09 0           # pin C09: DMA2 Stream 7 Channel 7
dma pin A08 1           # pin A08: DMA2 Stream 1 Channel 6
dma pin B08 0           # pin B08: DMA1 Stream 7 Channel 2

# feature
feature OSD

# aux
aux 0 40 255 900 2100 0 0

# master
set mag_hardware = NONE
set baro_hardware = NONE
set motor_pwm_protocol = DSHOT600
set current_meter = ADC
set battery_meter = ADC
set vbat_scale = 160
set ibata_scale = 150
set beeper_inversion = ON
set beeper_od = OFF
set pid_process_denom = 1
set system_hse_mhz = 8
set max7456_spi_bus = 2
set pinio_box = 40,255,255,255
set gyro_1_bustype = SPI
set gyro_1_spibus = 1

*/
