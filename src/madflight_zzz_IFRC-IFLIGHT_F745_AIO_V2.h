/*==============================================================================
Generated on: 2025-04-16 00:58:15.271972
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: IFLIGHT_F745_AIO_V2
Manufacturer ID: IFRC

//copy this line to madflight.ino to use this flight controller
#include <madflight_zzz_IFRC-IFLIGHT_F745_AIO_V2.h>

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out, or use madflight_config to override.
==============================================================================*/

#define MF_BOARD_NAME "BETAFLIGHT-IFRC-IFLIGHT_F745_AIO_V2"
#define MF_MCU_NAME "STM32F745"

const char* madflight_board = R""(
imu_bus_type SPI
imu_gizmo MPU6000 // #define USE_GYRO_SPI_MPU6000
bar_gizmo DPS310 // #define USE_BARO_DPS310
// resource BEEPER 1 D02
pin_out0 PB0 // resource MOTOR 1 B00
pin_out1 PB1 // resource MOTOR 2 B01
pin_out2 PB4 // resource MOTOR 3 B04
pin_out3 PB5 // resource MOTOR 4 B05
pin_out4 PD12 // resource MOTOR 5 D12
pin_out5 PD13 // resource MOTOR 6 D13
pin_out6 PC8 // resource MOTOR 7 C08
pin_out7 PC9 // resource MOTOR 8 C09
// resource LED_STRIP 1 A08
pin_ser0_tx PA9 // resource SERIAL_TX 1 A09
pin_ser1_tx PA2 // resource SERIAL_TX 2 A02
pin_ser2_tx PB10 // resource SERIAL_TX 3 B10
pin_ser3_tx PA0 // resource SERIAL_TX 4 A00
pin_ser5_tx PC6 // resource SERIAL_TX 6 C06
pin_ser6_tx PE8 // resource SERIAL_TX 7 E08
pin_ser7_tx PE1 // resource SERIAL_TX 8 E01
pin_ser0_rx PA10 // resource SERIAL_RX 1 A10
pin_ser1_rx PA3 // resource SERIAL_RX 2 A03
pin_ser2_rx PB11 // resource SERIAL_RX 3 B11
pin_ser3_rx PA1 // resource SERIAL_RX 4 A01
pin_ser5_rx PC7 // resource SERIAL_RX 6 C07
pin_ser6_rx PE7 // resource SERIAL_RX 7 E07
pin_ser7_rx PE0 // resource SERIAL_RX 8 E00
pin_i2c0_scl PB8 // resource I2C_SCL 1 B08
pin_i2c1_scl PB10 // resource I2C_SCL 2 B10
pin_i2c0_sda PB9 // resource I2C_SDA 1 B09
pin_i2c1_sda PB11 // resource I2C_SDA 2 B11
pin_led PC13 // resource LED 1 C13
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi2_sclk PC10 // resource SPI_SCK 3 C10
pin_spi3_sclk PE2 // resource SPI_SCK 4 E02
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PB14 // resource SPI_MISO 2 B14
pin_spi2_miso PC11 // resource SPI_MISO 3 C11
pin_spi3_miso PE5 // resource SPI_MISO 4 E05
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi1_mosi PB15 // resource SPI_MOSI 2 B15
pin_spi2_mosi PC12 // resource SPI_MOSI 3 C12
pin_spi3_mosi PE6 // resource SPI_MOSI 4 E06
// resource CAMERA_CONTROL 1 B03
pin_bat_v PC3 // resource ADC_BATT 1 C03
bat_gizmo ADC
// resource ADC_RSSI 1 C05
pin_bat_i PC2 // resource ADC_CURR 1 C02
bat_gizmo ADC
// resource ADC_EXT 1 C01
// resource FLASH_CS 1 A15
// resource OSD_CS 1 E04
pin_imu_int PD0 // resource GYRO_EXTI 1 D00
// resource GYRO_EXTI 2 D08
pin_imu_cs PA4 // resource GYRO_CS 1 A04
// resource GYRO_CS 2 B12
// set align_mag = CW180
// set mag_align_yaw = 1800
// set mag_bustype = I2C
mag_i2c_bus 0 // set mag_i2c_device = 1
// set baro_bustype = I2C
bar_i2c_bus 0 // set baro_i2c_device = 1
// set serialrx_provider = SBUS
// set blackbox_device = SPIFLASH
// set dshot_bidir = ON
// set motor_pwm_protocol = DSHOT600
// set current_meter = ADC
// set battery_meter = ADC
// set ibata_scale = 100
// set beeper_inversion = ON
// set beeper_od = OFF
// set max7456_spi_bus = 4
// set flash_spi_bus = 3
// set gyro_1_bustype = SPI
imu_spi_bus 0 // set gyro_1_spibus = 1
// set gyro_2_spibus = 2
// set gyro_2_sensor_align = CW180
// set gyro_2_align_yaw = 1800
)""; //end of madflight_board


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F745 (S745) 4.1.0 Sep 25 2019 / 01:23:16 (064ca9b75) MSP API: 1.42

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACCGYRO_BMI270
#define USE_BARO
#define USE_BARO_DPS310
#define USE_FLASH
#define USE_FLASH_M25P16
#define USE_MAX7456

board_name IFLIGHT_F745_AIO_V2
manufacturer_id IFRC

# resources
resource BEEPER 1 D02
resource MOTOR 1 B00
resource MOTOR 2 B01
resource MOTOR 3 B04
resource MOTOR 4 B05
resource MOTOR 5 D12
resource MOTOR 6 D13
resource MOTOR 7 C08
resource MOTOR 8 C09
resource LED_STRIP 1 A08
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 A02
resource SERIAL_TX 3 B10
resource SERIAL_TX 4 A00
resource SERIAL_TX 6 C06
resource SERIAL_TX 7 E08
resource SERIAL_TX 8 E01
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 A03
resource SERIAL_RX 3 B11
resource SERIAL_RX 4 A01
resource SERIAL_RX 6 C07
resource SERIAL_RX 7 E07
resource SERIAL_RX 8 E00
resource I2C_SCL 1 B08
resource I2C_SCL 2 B10
resource I2C_SDA 1 B09
resource I2C_SDA 2 B11
resource LED 1 C13
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 C10
resource SPI_SCK 4 E02
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 C11
resource SPI_MISO 4 E05
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 C12
resource SPI_MOSI 4 E06
resource CAMERA_CONTROL 1 B03
resource ADC_BATT 1 C03
resource ADC_RSSI 1 C05
resource ADC_CURR 1 C02
resource ADC_EXT 1 C01
resource FLASH_CS 1 A15
resource OSD_CS 1 E04
resource GYRO_EXTI 1 D00
resource GYRO_EXTI 2 D08
resource GYRO_CS 1 A04
resource GYRO_CS 2 B12

# timer
timer A08 AF1
# pin A08: TIM1 CH1 (AF1)
timer B03 AF1
# pin B03: TIM2 CH2 (AF1)
timer B00 AF2
# pin B00: TIM3 CH3 (AF2)
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer B04 AF2
# pin B04: TIM3 CH1 (AF2)
timer B05 AF2
# pin B05: TIM3 CH2 (AF2)
timer D12 AF2
# pin D12: TIM4 CH1 (AF2)
timer D13 AF2
# pin D13: TIM4 CH2 (AF2)
timer C08 AF3
# pin C08: TIM8 CH3 (AF3)
timer C09 AF3
# pin C09: TIM8 CH4 (AF3)

# dma
dma ADC 1 1
# ADC 1: DMA2 Stream 0 Channel 0
dma pin A08 0
# pin A08: DMA2 Stream 6 Channel 0
dma pin B03 0
# pin B03: DMA1 Stream 6 Channel 3
dma pin B00 0
# pin B00: DMA1 Stream 7 Channel 5
dma pin B01 0
# pin B01: DMA1 Stream 2 Channel 5
dma pin B04 0
# pin B04: DMA1 Stream 4 Channel 5
dma pin B05 0
# pin B05: DMA1 Stream 5 Channel 5
dma pin D12 0
# pin D12: DMA1 Stream 0 Channel 2
dma pin D13 0
# pin D13: DMA1 Stream 3 Channel 2
dma pin C08 1
# pin C08: DMA2 Stream 4 Channel 7
dma pin C09 0
# pin C09: DMA2 Stream 7 Channel 7

# feature
feature -AIRMODE
feature RX_SERIAL
feature TELEMETRY
feature LED_STRIP
feature OSD

# master
set align_mag = CW180
set mag_align_yaw = 1800
set mag_bustype = I2C
set mag_i2c_device = 1
set baro_bustype = I2C
set baro_i2c_device = 1
set serialrx_provider = SBUS
set blackbox_device = SPIFLASH
set dshot_bidir = ON
set motor_pwm_protocol = DSHOT600
set current_meter = ADC
set battery_meter = ADC
set ibata_scale = 100
set beeper_inversion = ON
set beeper_od = OFF
set max7456_spi_bus = 4
set flash_spi_bus = 3
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_2_spibus = 2
set gyro_2_sensor_align = CW180
set gyro_2_align_yaw = 1800

*/
