/*==============================================================================
Generated on: 2025-04-16 00:58:15.184694
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: FOXEERF722V2
Manufacturer ID: FOXE

//copy this line to madflight.ino to use this flight controller
#include <madflight_zzz_FOXE-FOXEERF722V2.h>

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out, or use madflight_config to override.
==============================================================================*/

#define MF_BOARD_NAME "BETAFLIGHT-FOXE-FOXEERF722V2"
#define MF_MCU_NAME "STM32F7X2"

const char* madflight_board = R""(
imu_bus_type SPI
imu_gizmo MPU6000 // #define USE_GYRO_SPI_MPU6000
// resource BEEPER 1 A04
pin_out0 PA9 // resource MOTOR 1 A09
pin_out1 PA8 // resource MOTOR 2 A08
pin_out2 PC9 // resource MOTOR 3 C09
pin_out3 PC8 // resource MOTOR 4 C08
pin_out4 PC6 // resource MOTOR 5 C06
pin_out5 PC7 // resource MOTOR 6 C07
pin_rcl_ppm PB7 // resource PPM 1 B07
// resource LED_STRIP 1 A15
pin_ser0_tx PB6 // resource SERIAL_TX 1 B06
pin_ser1_tx PA2 // resource SERIAL_TX 2 A02
pin_ser2_tx PB10 // resource SERIAL_TX 3 B10
pin_ser3_tx PA0 // resource SERIAL_TX 4 A00
pin_ser4_tx PC12 // resource SERIAL_TX 5 C12
pin_ser0_rx PB7 // resource SERIAL_RX 1 B07
pin_ser1_rx PA3 // resource SERIAL_RX 2 A03
pin_ser2_rx PB11 // resource SERIAL_RX 3 B11
pin_ser3_rx PA1 // resource SERIAL_RX 4 A01
pin_ser4_rx PD2 // resource SERIAL_RX 5 D02
pin_i2c0_scl PB8 // resource I2C_SCL 1 B08
pin_i2c0_sda PB9 // resource I2C_SDA 1 B09
pin_led PC15 // resource LED 1 C15
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi2_sclk PC10 // resource SPI_SCK 3 C10
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PB14 // resource SPI_MISO 2 B14
pin_spi2_miso PC11 // resource SPI_MISO 3 C11
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi1_mosi PB15 // resource SPI_MOSI 2 B15
pin_spi2_mosi PB5 // resource SPI_MOSI 3 B05
// resource CAMERA_CONTROL 1 B03
pin_bat_v PC0 // resource ADC_BATT 1 C00
bat_gizmo ADC
// resource ADC_RSSI 1 A00
pin_bat_i PC2 // resource ADC_CURR 1 C02
bat_gizmo ADC
// resource FLASH_CS 1 B12
// resource OSD_CS 1 C03
pin_imu_int PC4 // resource GYRO_EXTI 1 C04
pin_imu_cs PB2 // resource GYRO_CS 1 B02
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
// set max7456_spi_bus = 3
// set flash_spi_bus = 2
// set gyro_1_bustype = SPI
imu_spi_bus 0 // set gyro_1_spibus = 1
imu_align CW270 // set gyro_1_sensor_align = CW270
)""; //end of madflight_board


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F7X2 (S7X2) 4.0.0 Apr  4 2019 / 00:54:12 (e44f75b4a) MSP API: 1.41

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_MAX7456
#define USE_CAMERA_CONTROL

board_name FOXEERF722V2
manufacturer_id FOXE

# resources
resource BEEPER 1 A04
resource MOTOR 1 A09
resource MOTOR 2 A08
resource MOTOR 3 C09
resource MOTOR 4 C08
resource MOTOR 5 C06
resource MOTOR 6 C07
resource PPM 1 B07
resource LED_STRIP 1 A15
resource SERIAL_TX 1 B06
resource SERIAL_TX 2 A02
resource SERIAL_TX 3 B10
resource SERIAL_TX 4 A00
resource SERIAL_TX 5 C12
resource SERIAL_RX 1 B07
resource SERIAL_RX 2 A03
resource SERIAL_RX 3 B11
resource SERIAL_RX 4 A01
resource SERIAL_RX 5 D02
resource I2C_SCL 1 B08
resource I2C_SDA 1 B09
resource LED 1 C15
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 C10
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 C11
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 B05
resource CAMERA_CONTROL 1 B03
resource ADC_BATT 1 C00
resource ADC_RSSI 1 A00
resource ADC_CURR 1 C02
resource FLASH_CS 1 B12
resource OSD_CS 1 C03
resource GYRO_EXTI 1 C04

resource GYRO_CS 1 B02


# timer
timer B07 AF2
# pin B07: TIM4 CH2 (AF2)
timer A09 AF1
# pin A09: TIM1 CH2 (AF1)
timer A08 AF1
# pin A08: TIM1 CH1 (AF1)
timer C09 AF3
# pin C09: TIM8 CH4 (AF3)
timer C08 AF3
# pin C08: TIM8 CH3 (AF3)
timer C06 AF3
# pin C06: TIM8 CH1 (AF3)
timer C07 AF3
# pin C07: TIM8 CH2 (AF3)
timer A15 AF1
# pin A15: TIM2 CH1 (AF1)
timer B03 AF1
# pin B03: TIM2 CH2 (AF1)

# dma
dma ADC 3 0
# ADC 3: DMA2 Stream 0 Channel 2
dma pin B07 0
# pin B07: DMA1 Stream 3 Channel 2
dma pin A09 0
# pin A09: DMA2 Stream 6 Channel 0
dma pin A08 0
# pin A08: DMA2 Stream 6 Channel 0
dma pin C09 0
# pin C09: DMA2 Stream 7 Channel 7
dma pin C08 0
# pin C08: DMA2 Stream 2 Channel 0
dma pin C06 0
# pin C06: DMA2 Stream 2 Channel 0
dma pin C07 0
# pin C07: DMA2 Stream 2 Channel 0
dma pin A15 0
# pin A15: DMA1 Stream 5 Channel 3
dma pin B03 0
# pin B03: DMA1 Stream 6 Channel 3

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
set max7456_spi_bus = 3
set flash_spi_bus = 2
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW270

*/
