/*==============================================================================
Generated on: 2025-04-16 00:58:15.239922
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: HIFIONRCF7
Manufacturer ID: HFOR

//copy this line to madflight.ino to use this flight controller
#include <madflight_zzz_HFOR-HIFIONRCF7.h>

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out, or use madflight_config to override.
==============================================================================*/

#define MF_BOARD_NAME "BETAFLIGHT-HFOR-HIFIONRCF7"
#define MF_MCU_NAME "STM32F7X2"

const char* madflight_board = R""(
imu_bus_type SPI
imu_gizmo MPU6000 // #define USE_GYRO_SPI_MPU6000
bar_gizmo BMP280 // #define USE_BARO_BMP280
// resource BEEPER 1 C15
pin_out0 PB0 // resource MOTOR 1 B00
pin_out1 PB1 // resource MOTOR 2 B01
pin_out2 PB4 // resource MOTOR 3 B04
pin_out3 PB3 // resource MOTOR 4 B03
pin_out4 PC9 // resource MOTOR 5 C09
pin_out5 PC8 // resource MOTOR 6 C08
pin_rcl_ppm PA3 // resource PPM 1 A03
// resource LED_STRIP 1 A08
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
pin_i2c0_scl PB6 // resource I2C_SCL 1 B06
pin_i2c0_sda PB7 // resource I2C_SDA 1 B07
pin_led PA15 // resource LED 1 A15
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi2_sclk PC10 // resource SPI_SCK 3 C10
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PB14 // resource SPI_MISO 2 B14
pin_spi2_miso PC11 // resource SPI_MISO 3 C11
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi1_mosi PB15 // resource SPI_MOSI 2 B15
pin_spi2_mosi PB5 // resource SPI_MOSI 3 B05
// resource CAMERA_CONTROL 1 B08
pin_bat_v PC2 // resource ADC_BATT 1 C02
bat_gizmo ADC
// resource ADC_RSSI 1 C00
pin_bat_i PC1 // resource ADC_CURR 1 C01
bat_gizmo ADC
// resource PINIO 1 C14
// resource PINIO 2 B09
// resource FLASH_CS 1 C13
// resource OSD_CS 1 B12
pin_imu_int PC4 // resource GYRO_EXTI 1 C04
// resource GYRO_EXTI 2 C03
pin_imu_cs PB2 // resource GYRO_CS 1 B02
// resource GYRO_CS 2 A04
// set baro_bustype = I2C
bar_i2c_bus 0 // set baro_i2c_device = 1
// set adc_device = 3
// set blackbox_device = SPIFLASH
// set dshot_burst = ON
// set current_meter = ADC
// set battery_meter = ADC
// set ibata_scale = 450
// set beeper_inversion = ON
// set beeper_od = OFF
// set max7456_spi_bus = 2
// set pinio_box = 40,41,255,255
// set flash_spi_bus = 3
// set gyro_1_bustype = SPI
imu_spi_bus 0 // set gyro_1_spibus = 1
imu_align CW90 // set gyro_1_sensor_align = CW90
// set gyro_2_spibus = 1
// set gyro_2_sensor_align = CW90
)""; //end of madflight_board


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F7X2 (S7X2) 4.0.0 Apr  1 2019 / 16:45:55 (f0ac671fa) MSP API: 1.41

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_BARO
#define USE_BARO_BMP280
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

board_name HIFIONRCF7
manufacturer_id HFOR

# resources
resource BEEPER 1 C15
resource MOTOR 1 B00
resource MOTOR 2 B01
resource MOTOR 3 B04
resource MOTOR 4 B03
resource MOTOR 5 C09
resource MOTOR 6 C08
resource PPM 1 A03
resource LED_STRIP 1 A08
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
resource I2C_SCL 1 B06
resource I2C_SDA 1 B07
resource LED 1 A15
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 C10
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 C11
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 B05
resource CAMERA_CONTROL 1 B08
resource ADC_BATT 1 C02
resource ADC_RSSI 1 C00
resource ADC_CURR 1 C01
resource PINIO 1 C14
resource PINIO 2 B09
resource FLASH_CS 1 C13
resource OSD_CS 1 B12
resource GYRO_EXTI 1 C04
resource GYRO_EXTI 2 C03
resource GYRO_CS 1 B02
resource GYRO_CS 2 A04

# timer
timer A03 AF3
# pin A03: TIM9 CH2 (AF3)
timer B00 AF2
# pin B00: TIM3 CH3 (AF2)
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer B04 AF2
# pin B04: TIM3 CH1 (AF2)
timer B03 AF1
# pin B03: TIM2 CH2 (AF1)
timer C09 AF3
# pin C09: TIM8 CH4 (AF3)
timer C08 AF3
# pin C08: TIM8 CH3 (AF3)
timer A08 AF1
# pin A08: TIM1 CH1 (AF1)
timer B08 AF2
# pin B08: TIM4 CH3 (AF2)

# dma
dma ADC 3 1
# ADC 3: DMA2 Stream 1 Channel 2
dma pin B00 0
# pin B00: DMA1 Stream 7 Channel 5
dma pin B01 0
# pin B01: DMA1 Stream 2 Channel 5
dma pin B04 0
# pin B04: DMA1 Stream 4 Channel 5
dma pin B03 0
# pin B03: DMA1 Stream 6 Channel 3
dma pin C09 0
# pin C09: DMA2 Stream 7 Channel 7
dma pin C08 0
# pin C08: DMA2 Stream 2 Channel 0
dma pin A08 0
# pin A08: DMA2 Stream 6 Channel 0
dma pin B08 0
# pin B08: DMA1 Stream 7 Channel 2

# feature
feature OSD

# master
set baro_bustype = I2C
set baro_i2c_device = 1
set adc_device = 3
set blackbox_device = SPIFLASH
set dshot_burst = ON
set current_meter = ADC
set battery_meter = ADC
set ibata_scale = 450
set beeper_inversion = ON
set beeper_od = OFF
set max7456_spi_bus = 2
set pinio_box = 40,41,255,255
set flash_spi_bus = 3
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW90
set gyro_2_spibus = 1
set gyro_2_sensor_align = CW90

*/
