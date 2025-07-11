/*==============================================================================
Generated on: 2025-06-11 20:35:54.079121
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: NEUTRONRCH7BT
Manufacturer ID: NERC

//copy this line to madflight.ino to use this flight controller
#define MF_BOARD "brd/betaflight/NERC-NEUTRONRCH7BT.h"

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out, or use madflight_config to override.
==============================================================================*/

#define MF_BOARD_NAME "BETAFLIGHT-NERC-NEUTRONRCH7BT"
#define MF_MCU_NAME "STM32H743"

const char madflight_board[] = R""(
imu_bus_type SPI
bar_gizmo DPS310 // #define USE_BARO_DPS310
// resource BEEPER 1 A15
pin_out0 PB0 // resource MOTOR 1 B00
pin_out1 PB1 // resource MOTOR 2 B01
pin_out2 PA0 // resource MOTOR 3 A00
pin_out3 PA1 // resource MOTOR 4 A01
pin_out4 PA2 // resource MOTOR 5 A02
pin_out5 PA3 // resource MOTOR 6 A03
pin_out6 PD12 // resource MOTOR 7 D12
pin_out7 PD13 // resource MOTOR 8 D13
// resource LED_STRIP 1 A08
pin_ser0_tx PA9 // resource SERIAL_TX 1 A09
pin_ser1_tx PD5 // resource SERIAL_TX 2 D05
pin_ser2_tx PD8 // resource SERIAL_TX 3 D08
pin_ser3_tx PB9 // resource SERIAL_TX 4 B09
pin_ser5_tx PC6 // resource SERIAL_TX 6 C06
pin_ser6_tx PE8 // resource SERIAL_TX 7 E08
pin_ser7_tx PE1 // resource SERIAL_TX 8 E01
pin_ser0_rx PA10 // resource SERIAL_RX 1 A10
pin_ser1_rx PD6 // resource SERIAL_RX 2 D06
pin_ser2_rx PD9 // resource SERIAL_RX 3 D09
pin_ser3_rx PB8 // resource SERIAL_RX 4 B08
pin_ser5_rx PC7 // resource SERIAL_RX 6 C07
pin_ser6_rx PE7 // resource SERIAL_RX 7 E07
pin_ser7_rx PE0 // resource SERIAL_RX 8 E00
pin_i2c0_scl PB6 // resource I2C_SCL 1 B06
pin_i2c1_scl PB10 // resource I2C_SCL 2 B10
pin_i2c0_sda PB7 // resource I2C_SDA 1 B07
pin_i2c1_sda PB11 // resource I2C_SDA 2 B11
pin_led PE3 // resource LED 1 E03
// resource LED 2 E04
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi2_sclk PB3 // resource SPI_SCK 3 B03
pin_spi3_sclk PE12 // resource SPI_SCK 4 E12
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PB14 // resource SPI_MISO 2 B14
pin_spi2_miso PB4 // resource SPI_MISO 3 B04
pin_spi3_miso PE13 // resource SPI_MISO 4 E13
pin_spi0_mosi PD7 // resource SPI_MOSI 1 D07
pin_spi1_mosi PB15 // resource SPI_MOSI 2 B15
pin_spi2_mosi PB5 // resource SPI_MOSI 3 B05
pin_spi3_mosi PE14 // resource SPI_MOSI 4 E14
pin_bat_v PC0 // resource ADC_BATT 1 C00
bat_gizmo ADC
// resource ADC_RSSI 1 C05
pin_bat_i PC1 // resource ADC_CURR 1 C01
bat_gizmo ADC
// resource ADC_EXT 1 C04
pin_mmc_clk PC12 // resource SDIO_CK 1 C12
pin_mmc_cmd PD2 // resource SDIO_CMD 1 D02
pin_mmc_dat PC8 // resource SDIO_D0 1 C08
bbx_gizmo SDMMC
// resource SDIO_D1 1 C09
// resource SDIO_D2 1 C10
// resource SDIO_D3 1 C11
// resource PINIO 1 C03
// resource PINIO 2 D10
// resource PINIO 3 D11
// resource OSD_CS 1 B12
pin_imu_int PB2 // resource GYRO_EXTI 1 B02
// resource GYRO_EXTI 2 E15
pin_imu_cs PC15 // resource GYRO_CS 1 C15
// resource GYRO_CS 2 E11
// resource USB_DETECT 1 E02
// set gyro_to_use = BOTH
// set mag_bustype = I2C
mag_i2c_bus 0 // set mag_i2c_device = 1
// set baro_bustype = I2C
bar_i2c_bus 1 // set baro_i2c_device = 2
// set blackbox_device = SDCARD
// set current_meter = ADC
// set battery_meter = ADC
// set beeper_inversion = ON
// set beeper_od = OFF
// set sdio_use_4bit_width = ON
// set sdio_device = 1
// set max7456_spi_bus = 2
// set pinio_config = 129,1,1,1
// set pinio_box = 0,40,41,42
// set gyro_1_bustype = SPI
imu_spi_bus 0 // set gyro_1_spibus = 1
imu_align CW270 // set gyro_1_sensor_align = CW270
// set gyro_1_align_yaw = 2700
// set gyro_2_bustype = SPI
// set gyro_2_spibus = 4
)""; //end of madflight_board


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32H743 (SH74) 4.3.0 Aug 10 2021 / 12:14:35 (8ca4fdc58) MSP API: 1.44

#define USE_ACC
#define USE_GYRO
#define USE_ACCGYRO_BMI270
#define USE_BARO
#define USE_BARO_DPS310
#define USE_MAX7456
#define USE_SDCARD

board_name NEUTRONRCH7BT
manufacturer_id NERC

# resources
resource BEEPER 1 A15
resource MOTOR 1 B00
resource MOTOR 2 B01
resource MOTOR 3 A00
resource MOTOR 4 A01
resource MOTOR 5 A02
resource MOTOR 6 A03
resource MOTOR 7 D12
resource MOTOR 8 D13
resource LED_STRIP 1 A08
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 D05
resource SERIAL_TX 3 D08
resource SERIAL_TX 4 B09
resource SERIAL_TX 6 C06
resource SERIAL_TX 7 E08
resource SERIAL_TX 8 E01
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 D06
resource SERIAL_RX 3 D09
resource SERIAL_RX 4 B08
resource SERIAL_RX 6 C07
resource SERIAL_RX 7 E07
resource SERIAL_RX 8 E00
resource I2C_SCL 1 B06
resource I2C_SCL 2 B10
resource I2C_SDA 1 B07
resource I2C_SDA 2 B11
resource LED 1 E03
resource LED 2 E04
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 B03
resource SPI_SCK 4 E12
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 B04
resource SPI_MISO 4 E13
resource SPI_MOSI 1 D07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 B05
resource SPI_MOSI 4 E14
resource ADC_BATT 1 C00
resource ADC_RSSI 1 C05
resource ADC_CURR 1 C01
resource ADC_EXT 1 C04
resource SDIO_CK 1 C12
resource SDIO_CMD 1 D02
resource SDIO_D0 1 C08
resource SDIO_D1 1 C09
resource SDIO_D2 1 C10
resource SDIO_D3 1 C11
resource PINIO 1 C03
resource PINIO 2 D10
resource PINIO 3 D11
resource OSD_CS 1 B12
resource GYRO_EXTI 1 B02
resource GYRO_EXTI 2 E15
resource GYRO_CS 1 C15
resource GYRO_CS 2 E11
resource USB_DETECT 1 E02

# timer
timer B00 AF2
# pin B00: TIM3 CH3 (AF2)
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer A00 AF2
# pin A00: TIM5 CH1 (AF2)
timer A01 AF2
# pin A01: TIM5 CH2 (AF2)
timer A02 AF2
# pin A02: TIM5 CH3 (AF2)
timer A03 AF2
# pin A03: TIM5 CH4 (AF2)
timer D12 AF2
# pin D12: TIM4 CH1 (AF2)
timer D13 AF2
# pin D13: TIM4 CH2 (AF2)
timer D14 AF2
# pin D14: TIM4 CH3 (AF2)
timer D15 AF2
# pin D15: TIM4 CH4 (AF2)
timer E05 AF4
# pin E05: TIM15 CH1 (AF4)
timer E06 AF4
# pin E06: TIM15 CH2 (AF4)
timer A08 AF1
# pin A08: TIM1 CH1 (AF1)
timer A15 AF1
# pin A15: TIM2 CH1 (AF1)
timer C07 AF3
# pin C07: TIM8 CH2 (AF3)
timer C06 AF3
# pin C06: TIM8 CH1 (AF3)
timer B08 AF1
# pin B08: TIM16 CH1 (AF1)
timer B09 AF1
# pin B09: TIM17 CH1 (AF1)

# dma
dma ADC 1 8
# ADC 1: DMA2 Stream 0 Request 9
dma ADC 3 9
# ADC 3: DMA2 Stream 1 Request 115
dma TIMUP 1 0
# TIMUP 1: DMA1 Stream 0 Request 15
dma TIMUP 2 0
# TIMUP 2: DMA1 Stream 0 Request 22
dma TIMUP 3 2
# TIMUP 3: DMA1 Stream 2 Request 27
dma TIMUP 4 1
# TIMUP 4: DMA1 Stream 1 Request 32
dma TIMUP 5 0
# TIMUP 5: DMA1 Stream 0 Request 59
dma TIMUP 8 0
# TIMUP 8: DMA1 Stream 0 Request 51
dma pin B00 0
# pin B00: DMA1 Stream 0 Request 25
dma pin B01 1
# pin B01: DMA1 Stream 1 Request 26
dma pin A00 2
# pin A00: DMA1 Stream 2 Request 55
dma pin A01 3
# pin A01: DMA1 Stream 3 Request 56
dma pin A02 4
# pin A02: DMA1 Stream 4 Request 57
dma pin A03 5
# pin A03: DMA1 Stream 5 Request 58
dma pin D12 6
# pin D12: DMA1 Stream 6 Request 29
dma pin D13 7
# pin D13: DMA1 Stream 7 Request 30
dma pin D14 12
# pin D14: DMA2 Stream 4 Request 31
dma pin E05 0
# pin E05: DMA1 Stream 0 Request 105
dma pin A08 14
# pin A08: DMA2 Stream 6 Request 11
dma pin A15 0
# pin A15: DMA1 Stream 0 Request 18
dma pin C07 0
# pin C07: DMA1 Stream 0 Request 48
dma pin C06 0
# pin C06: DMA1 Stream 0 Request 47
dma pin B08 0
# pin B08: DMA1 Stream 0 Request 109
dma pin B09 0
# pin B09: DMA1 Stream 0 Request 111

# feature
feature RX_SERIAL
feature TELEMETRY
feature OSD
feature ESC_SENSOR

# serial
serial 1 2 115200 57600 0 115200
serial 2 1 19200 57600 0 115200
serial 3 8192 115200 57600 0 115200
serial 5 64 115200 57600 0 115200
serial 7 1024 115200 57600 0 115200

# master
set gyro_to_use = BOTH
set mag_bustype = I2C
set mag_i2c_device = 1
set baro_bustype = I2C
set baro_i2c_device = 2
set blackbox_device = SDCARD
set current_meter = ADC
set battery_meter = ADC
set beeper_inversion = ON
set beeper_od = OFF
set sdio_use_4bit_width = ON
set sdio_device = 1
set max7456_spi_bus = 2
set pinio_config = 129,1,1,1
set pinio_box = 0,40,41,42
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW270
set gyro_1_align_yaw = 2700
set gyro_2_bustype = SPI
set gyro_2_spibus = 4

*/
