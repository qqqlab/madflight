/*==============================================================================
Generated on: 2025-03-18 18:40:43.534543
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: NEUTRONRCF411SX1280
Manufacturer ID: NERC

//copy this line to madflight.ino to use this flight controller
#include <madflight_board_NERC-NEUTRONRCF411SX1280.h>

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out.
==============================================================================*/

#define HAL_BOARD_NAME "BETAFLIGHT-NERC-NEUTRONRCF411SX1280"
#define HAL_MCU "STM32F411SX1280"

#define MADFLIGHT_BOARD R""(
imu_gizmo ICM42688P // #define USE_GYRO_SPI_ICM42688P
imu_gizmo ICM42605 // #define USE_GYRO_SPI_ICM42605
imu_gizmo ICM20689 // #define USE_GYRO_SPI_ICM20689
imu_gizmo MPU6000 // #define USE_GYRO_SPI_MPU6000
// resource BEEPER 1 A14
pin_out0 PB8 // resource MOTOR 1 B08
pin_out1 PA0 // resource MOTOR 2 A00
pin_out2 PB10 // resource MOTOR 3 B10
pin_out3 PB7 // resource MOTOR 4 B07
// resource LED_STRIP 1 B01
pin_ser0_tx PA9 // resource SERIAL_TX 1 A09
pin_ser1_tx PA2 // resource SERIAL_TX 2 A02
pin_ser0_rx PA10 // resource SERIAL_RX 1 A10
pin_ser1_rx PA3 // resource SERIAL_RX 2 A03
pin_led PC14 // resource LED 1 C14
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi2_sclk PB3 // resource SPI_SCK 3 B03
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PB14 // resource SPI_MISO 2 B14
pin_spi2_miso PB4 // resource SPI_MISO 3 B04
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi1_mosi PB15 // resource SPI_MOSI 2 B15
pin_spi2_mosi PB5 // resource SPI_MOSI 3 B05
pin_bat_v PA1 // resource ADC_BATT 1 A01
bat_gizmo ADC
pin_bat_i PB0 // resource ADC_CURR 1 B00
bat_gizmo ADC
// resource FLASH_CS 1 A08
// resource OSD_CS 1 B12
// resource RX_SPI_CS 1 A15
// resource RX_SPI_EXTI 1 C13
// resource RX_SPI_BIND 1 B02
// resource RX_SPI_LED 1 C15
// resource RX_SPI_EXPRESSLRS_RESET 1 B09
// resource RX_SPI_EXPRESSLRS_BUSY 1 A13
pin_imu_int PB6 // resource GYRO_EXTI 1 B06
pin_imu_cs PA4 // resource GYRO_CS 1 A04
// set rx_spi_bus = 3
// set rx_spi_led_inversion = ON
// set blackbox_device = SPIFLASH
// set dshot_burst = OFF
// set dshot_bitbang = OFF
// set motor_pwm_protocol = DSHOT600
// set current_meter = ADC
// set battery_meter = ADC
// set ibata_scale = 165
// set beeper_inversion = ON
// set beeper_od = OFF
// set max7456_spi_bus = 2
// set led_inversion = 1
// set flash_spi_bus = 2
// set gyro_1_bustype = SPI
imu_spi_bus 0 // set gyro_1_spibus = 1
imu_align CW0 // set gyro_1_sensor_align = CW0
// set gyro_1_align_yaw = -45
)"" //end of MADFLIGHT_BOARD


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F411SX1280 (S4SX) 4.3.0 Jun 12 2022 / 14:56:52 (6d17f3f) MSP API: 1.44
#mcu STM32F411

#define USE_ACC
#define USE_ACC_SPI_ICM42688P
#define USE_ACC_SPI_ICM42605
#define USE_GYRO
#define USE_GYRO_SPI_ICM42688P
#define USE_GYRO_SPI_ICM42605
#define USE_GYRO_SPI_ICM20689
#define USE_ACC_SPI_ICM20689
#define USE_ACC_SPI_MPU6000
#define USE_GYRO_SPI_MPU6000
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_MAX7456
#define USE_RX_SPI
#define USE_RX_EXPRESSLRS
#define USE_RX_EXPRESSLRS_TELEMETRY
#define USE_RX_SX1280
#define RX_CHANNELS_AETR
#define RX_SPI_DEFAULT_PROTOCOL         RX_SPI_EXPRESSLRS
#define RX_EXPRESSLRS_TIMER_INSTANCE    TIM5
#define RX_EXPRESSLRS_SPI_RESET_PIN     PB9
#define RX_EXPRESSLRS_SPI_BUSY_PIN      PA13
#define RX_SPI_CS                       PA15
#define RX_SPI_EXTI                     PC13
#define RX_SPI_BIND                     PB2
#define RX_SPI_LED                      PC15

board_name NEUTRONRCF411SX1280
manufacturer_id NERC

# resource
resource BEEPER 1 A14
resource MOTOR 1 B08
resource MOTOR 2 A00
resource MOTOR 3 B10
resource MOTOR 4 B07
resource LED_STRIP 1 B01
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 A02
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 A03
resource LED 1 C14
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 B03
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 B04
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 B05
resource ADC_BATT 1 A01
resource ADC_CURR 1 B00
resource FLASH_CS 1 A08
resource OSD_CS 1 B12
resource RX_SPI_CS 1 A15
resource RX_SPI_EXTI 1 C13
resource RX_SPI_BIND 1 B02
resource RX_SPI_LED 1 C15
resource RX_SPI_EXPRESSLRS_RESET 1 B09
resource RX_SPI_EXPRESSLRS_BUSY 1 A13
resource GYRO_EXTI 1 B06
resource GYRO_CS 1 A04

# timer
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer B08 AF2
# pin B08: TIM4 CH3 (AF2)
timer A00 AF1
# pin A00: TIM2 CH1 (AF1)
timer B10 AF1
# pin B10: TIM2 CH3 (AF1)
timer B07 AF2
# pin B07: TIM4 CH2 (AF2)

# dma
dma ADC 1 0
# ADC 1: DMA2 Stream 0 Channel 0
dma pin B01 0
# pin B01: DMA1 Stream 2 Channel 5
dma pin B08 0
# pin B08: DMA1 Stream 7 Channel 2
dma pin A00 0
# pin A00: DMA1 Stream 5 Channel 3
dma pin B10 0
# pin B10: DMA1 Stream 1 Channel 3
dma pin B07 0
# pin B07: DMA1 Stream 3 Channel 2


# master
set rx_spi_bus = 3
set rx_spi_led_inversion = ON
set blackbox_device = SPIFLASH
set dshot_burst = OFF
set dshot_bitbang = OFF
set motor_pwm_protocol = DSHOT600
set current_meter = ADC
set battery_meter = ADC
set ibata_scale = 165
set beeper_inversion = ON
set beeper_od = OFF
set max7456_spi_bus = 2
set led_inversion = 1
set flash_spi_bus = 2
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW0
set gyro_1_align_yaw = -45

*/
