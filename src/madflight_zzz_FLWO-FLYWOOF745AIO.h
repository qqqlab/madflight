/*==============================================================================
Generated on: 2025-04-16 00:58:15.176865
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: FLYWOOF745AIO
Manufacturer ID: FLWO

//copy this line to madflight.ino to use this flight controller
#include <madflight_zzz_FLWO-FLYWOOF745AIO.h>

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out, or use madflight_config to override.
==============================================================================*/

#define MF_BOARD_NAME "BETAFLIGHT-FLWO-FLYWOOF745AIO"
#define MF_MCU_NAME "STM32F745"

const char* madflight_board = R""(
imu_bus_type SPI
imu_gizmo MPU6000 // #define USE_GYRO_SPI_MPU6000
bar_gizmo BMP280 // #define USE_BARO_BMP280
bar_gizmo DPS310 // #define USE_BARO_DPS310
// resource BEEPER 1 D15
pin_out0 PB0 // resource MOTOR 1 B00
pin_out1 PB1 // resource MOTOR 2 B01
pin_out2 PE9 // resource MOTOR 3 E09
pin_out3 PE11 // resource MOTOR 4 E11
pin_out4 PC9 // resource MOTOR 5 C09
pin_out5 PA3 // resource MOTOR 6 A03
pin_out6 PB4 // resource MOTOR 7 B04
pin_out7 PB5 // resource MOTOR 8 B05
pin_rcl_ppm PE13 // resource PPM 1 E13
// resource LED_STRIP 1 D12
pin_ser0_tx PA9 // resource SERIAL_TX 1 A09
pin_ser1_tx PD5 // resource SERIAL_TX 2 D05
pin_ser2_tx PB10 // resource SERIAL_TX 3 B10
pin_ser3_tx PA0 // resource SERIAL_TX 4 A00
pin_ser4_tx PC12 // resource SERIAL_TX 5 C12
pin_ser5_tx PC6 // resource SERIAL_TX 6 C06
pin_ser6_tx PE8 // resource SERIAL_TX 7 E08
pin_ser0_rx PA10 // resource SERIAL_RX 1 A10
pin_ser1_rx PD6 // resource SERIAL_RX 2 D06
pin_ser2_rx PB11 // resource SERIAL_RX 3 B11
pin_ser3_rx PA1 // resource SERIAL_RX 4 A01
pin_ser4_rx PD2 // resource SERIAL_RX 5 D02
pin_ser5_rx PC7 // resource SERIAL_RX 6 C07
pin_ser6_rx PE7 // resource SERIAL_RX 7 E07
pin_i2c0_scl PB6 // resource I2C_SCL 1 B06
pin_i2c0_sda PB7 // resource I2C_SDA 1 B07
pin_led PA2 // resource LED 1 A02
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi3_sclk PE2 // resource SPI_SCK 4 E02
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PB14 // resource SPI_MISO 2 B14
pin_spi3_miso PE5 // resource SPI_MISO 4 E05
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi1_mosi PB15 // resource SPI_MOSI 2 B15
pin_spi3_mosi PE6 // resource SPI_MOSI 4 E06
pin_bat_v PC3 // resource ADC_BATT 1 C03
bat_gizmo ADC
// resource ADC_RSSI 1 C05
pin_bat_i PC2 // resource ADC_CURR 1 C02
bat_gizmo ADC
// resource PINIO 1 C00
// resource FLASH_CS 1 A04
// resource OSD_CS 1 B12
pin_imu_int PE1 // resource GYRO_EXTI 1 E01
pin_imu_cs PE4 // resource GYRO_CS 1 E04
// resource USB_DETECT 1 A08
// set mag_bustype = I2C
mag_i2c_bus 0 // set mag_i2c_device = 1
// set baro_bustype = I2C
bar_i2c_bus 0 // set baro_i2c_device = 1
// set blackbox_device = SPIFLASH
// set dshot_bitbang = OFF
// set current_meter = ADC
// set ibata_scale = 175
// set battery_meter = ADC
// set beeper_inversion = ON
// set beeper_od = OFF
// set max7456_spi_bus = 2
// set dashboard_i2c_bus = 1
// set flash_spi_bus = 1
// set gyro_1_bustype = SPI
imu_spi_bus 3 // set gyro_1_spibus = 4
imu_align CW270 // set gyro_1_sensor_align = CW270
// set gyro_2_spibus = 4
)""; //end of madflight_board


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F745 (S745) 4.2.0 Jun 14 2020 / 03:05:04 (8f2d21460) MSP API: 1.43

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_DPS310
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

board_name FLYWOOF745AIO
manufacturer_id FLWO

# resources
resource BEEPER 1 D15
resource MOTOR 1 B00
resource MOTOR 2 B01
resource MOTOR 3 E09
resource MOTOR 4 E11
resource MOTOR 5 C09
resource MOTOR 6 A03
resource MOTOR 7 B04
resource MOTOR 8 B05
resource PPM 1 E13
resource LED_STRIP 1 D12
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 D05
resource SERIAL_TX 3 B10
resource SERIAL_TX 4 A00
resource SERIAL_TX 5 C12
resource SERIAL_TX 6 C06
resource SERIAL_TX 7 E08
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 D06
resource SERIAL_RX 3 B11
resource SERIAL_RX 4 A01
resource SERIAL_RX 5 D02
resource SERIAL_RX 6 C07
resource SERIAL_RX 7 E07
resource I2C_SCL 1 B06
resource I2C_SDA 1 B07
resource LED 1 A02
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 4 E02
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 4 E05
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 4 E06
resource ADC_BATT 1 C03
resource ADC_RSSI 1 C05
resource ADC_CURR 1 C02
resource PINIO 1 C00
resource FLASH_CS 1 A04
resource OSD_CS 1 B12
resource GYRO_EXTI 1 E01
resource GYRO_CS 1 E04
resource USB_DETECT 1 A08

# timer
timer E13 AF1
# pin E13: TIM1 CH3 (AF1)
timer B00 AF2
# pin B00: TIM3 CH3 (AF2)
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer E09 AF1
# pin E09: TIM1 CH1 (AF1)
timer E11 AF1
# pin E11: TIM1 CH2 (AF1)
timer C09 AF3
# pin C09: TIM8 CH4 (AF3)
timer A03 AF2
# pin A03: TIM5 CH4 (AF2)
timer B03 AF1
# pin B03: TIM2 CH2 (AF1)
timer B04 AF2
# pin B04: TIM3 CH1 (AF2)
timer B05 AF2
# pin B05: TIM3 CH2 (AF2)
timer D12 AF2
# pin D12: TIM4 CH1 (AF2)

# dma
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin E13 1
# pin E13: DMA2 Stream 6 Channel 6
dma pin B00 0
# pin B00: DMA1 Stream 7 Channel 5
dma pin B01 0
# pin B01: DMA1 Stream 2 Channel 5
dma pin E09 2
# pin E09: DMA2 Stream 3 Channel 6
dma pin E11 1
# pin E11: DMA2 Stream 2 Channel 6
dma pin C09 0
# pin C09: DMA2 Stream 7 Channel 7
dma pin A03 0
# pin A03: DMA1 Stream 1 Channel 6
dma pin B03 0
# pin B03: DMA1 Stream 6 Channel 3
dma pin B04 0
# pin B04: DMA1 Stream 4 Channel 5
dma pin B05 0
# pin B05: DMA1 Stream 5 Channel 5
dma pin D12 0
# pin D12: DMA1 Stream 0 Channel 2

# feature
feature RX_SERIAL
feature LED_STRIP
feature OSD

# master
set mag_bustype = I2C
set mag_i2c_device = 1
set baro_bustype = I2C
set baro_i2c_device = 1
set blackbox_device = SPIFLASH
set dshot_bitbang = OFF
set current_meter = ADC
set ibata_scale = 175
set battery_meter = ADC
set beeper_inversion = ON
set beeper_od = OFF
set max7456_spi_bus = 2
set dashboard_i2c_bus = 1
set flash_spi_bus = 1
set gyro_1_bustype = SPI
set gyro_1_spibus = 4
set gyro_1_sensor_align = CW270
set gyro_2_spibus = 4

*/
