/*==============================================================================
Generated on: 2025-04-16 00:58:15.094908
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: APEXF7
Manufacturer ID: APEX

//copy this line to madflight.ino to use this flight controller
#include <madflight_zzz_APEX-APEXF7.h>

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out, or use madflight_config to override.
==============================================================================*/

#define MF_BOARD_NAME "BETAFLIGHT-APEX-APEXF7"
#define MF_MCU_NAME "STM32F7X2"

const char* madflight_board = R""(
imu_bus_type SPI
imu_gizmo MPU6000 // #define USE_GYRO_SPI_MPU6000
bar_gizmo BMP280 // #define USE_BARO_BMP280
// resource BEEPER 1 B00
pin_out0 PC8 // resource MOTOR 1 C08
pin_out1 PB6 // resource MOTOR 2 B06
pin_out2 PC9 // resource MOTOR 3 C09
pin_out3 PB7 // resource MOTOR 4 B07
// resource LED_STRIP 1 A15
pin_ser0_tx PA9 // resource SERIAL_TX 1 A09
pin_ser1_tx PA2 // resource SERIAL_TX 2 A02
pin_ser2_tx PC10 // resource SERIAL_TX 3 C10
pin_ser3_tx PA0 // resource SERIAL_TX 4 A00
pin_ser4_tx PC12 // resource SERIAL_TX 5 C12
pin_ser5_tx PC6 // resource SERIAL_TX 6 C06
pin_ser0_rx PA10 // resource SERIAL_RX 1 A10
pin_ser2_rx PC11 // resource SERIAL_RX 3 C11
pin_ser3_rx PA1 // resource SERIAL_RX 4 A01
pin_ser5_rx PC7 // resource SERIAL_RX 6 C07
pin_i2c0_scl PB8 // resource I2C_SCL 1 B08
pin_i2c0_sda PB9 // resource I2C_SDA 1 B09
pin_led PB12 // resource LED 1 B12
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi2_sclk PB3 // resource SPI_SCK 3 B03
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PB14 // resource SPI_MISO 2 B14
pin_spi2_miso PB4 // resource SPI_MISO 3 B04
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi1_mosi PB15 // resource SPI_MOSI 2 B15
pin_spi2_mosi PB5 // resource SPI_MOSI 3 B05
// resource CAMERA_CONTROL 1 A08
pin_bat_v PC0 // resource ADC_BATT 1 C00
bat_gizmo ADC
// resource ADC_RSSI 1 C02
pin_bat_i PC1 // resource ADC_CURR 1 C01
bat_gizmo ADC
// resource PINIO 1 C14
// resource PINIO 2 B11
// resource FLASH_CS 1 A03
// resource OSD_CS 1 C15
pin_imu_int PA4 // resource GYRO_EXTI 1 A04
// resource GYRO_EXTI 2 C03
pin_imu_cs PD2 // resource GYRO_CS 1 D02
// resource GYRO_CS 2 B10
// set baro_bustype = I2C
bar_i2c_bus 0 // set baro_i2c_device = 1
// set blackbox_device = SPIFLASH
// set dshot_burst = ON
// set current_meter = ADC
// set battery_meter = ADC
// set beeper_inversion = ON
// set beeper_od = OFF
// set max7456_spi_bus = 2
// set pinio_config = 129,129,1,1
// set pinio_box = 0,40,255,255
// set flash_spi_bus = 1
// set gyro_1_bustype = SPI
imu_spi_bus 2 // set gyro_1_spibus = 3
// set gyro_2_spibus = 3
// set gyro_2_sensor_align = CW270
// set gyro_2_align_yaw = 2700
)""; //end of madflight_board


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F7X2 (S7X2) 4.1.6 Apr 25 2020 / 05:12:26 (283bda8bf) MSP API: 1.42

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_BARO
#define USE_BARO_BMP280
#define USE_MAX7456

board_name APEXF7
manufacturer_id APEX

# resources
resource BEEPER 1 B00
resource MOTOR 1 C08
resource MOTOR 2 B06
resource MOTOR 3 C09
resource MOTOR 4 B07
resource LED_STRIP 1 A15
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 A02
resource SERIAL_TX 3 C10
resource SERIAL_TX 4 A00
resource SERIAL_TX 5 C12
resource SERIAL_TX 6 C06
resource SERIAL_RX 1 A10
resource SERIAL_RX 3 C11
resource SERIAL_RX 4 A01
resource SERIAL_RX 6 C07
resource I2C_SCL 1 B08
resource I2C_SDA 1 B09
resource LED 1 B12
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 B03
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 B04
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 B05
resource CAMERA_CONTROL 1 A08
resource ADC_BATT 1 C00
resource ADC_RSSI 1 C02
resource ADC_CURR 1 C01
resource PINIO 1 C14
resource PINIO 2 B11
resource FLASH_CS 1 A03
resource OSD_CS 1 C15
resource GYRO_EXTI 1 A04
resource GYRO_EXTI 2 C03
resource GYRO_CS 1 D02
resource GYRO_CS 2 B10

# timer
timer A15 AF1
# pin A15: TIM2 CH1 (AF1)
timer A08 AF1
# pin A08: TIM1 CH1 (AF1)
timer C08 AF3
# pin C08: TIM8 CH3 (AF3)
timer B06 AF2
# pin B06: TIM4 CH1 (AF2)
timer C09 AF3
# pin C09: TIM8 CH4 (AF3)
timer B07 AF2
# pin B07: TIM4 CH2 (AF2)

# dma
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin A15 0
# pin A15: DMA1 Stream 5 Channel 3
dma pin A08 0
# pin A08: DMA2 Stream 6 Channel 0
dma pin C08 0
# pin C08: DMA2 Stream 2 Channel 0
dma pin B06 0
# pin B06: DMA1 Stream 0 Channel 2
dma pin C09 0
# pin C09: DMA2 Stream 7 Channel 7
dma pin B07 0
# pin B07: DMA1 Stream 3 Channel 2

# feature
feature OSD
feature RX_SERIAL

# serial
serial 1 1024 115200 57600 0 115200
serial 3 1 19200 57600 0 115200

# master
set baro_bustype = I2C
set baro_i2c_device = 1
set blackbox_device = SPIFLASH
set dshot_burst = ON
set current_meter = ADC
set battery_meter = ADC
set beeper_inversion = ON
set beeper_od = OFF
set max7456_spi_bus = 2
set pinio_config = 129,129,1,1
set pinio_box = 0,40,255,255
set flash_spi_bus = 1
set gyro_1_bustype = SPI
set gyro_1_spibus = 3
set gyro_2_spibus = 3
set gyro_2_sensor_align = CW270
set gyro_2_align_yaw = 2700

*/
