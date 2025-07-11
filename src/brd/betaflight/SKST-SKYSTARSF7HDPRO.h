/*==============================================================================
Generated on: 2025-06-11 20:35:54.110791
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: SKYSTARSF7HDPRO
Manufacturer ID: SKST

//copy this line to madflight.ino to use this flight controller
#define MF_BOARD "brd/betaflight/SKST-SKYSTARSF7HDPRO.h"

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out, or use madflight_config to override.
==============================================================================*/

#define MF_BOARD_NAME "BETAFLIGHT-SKST-SKYSTARSF7HDPRO"
#define MF_MCU_NAME "STM32F7X2"

const char madflight_board[] = R""(
imu_bus_type SPI
imu_gizmo MPU6000 // #define USE_GYRO_SPI_MPU6000
bar_gizmo SPI_BMP280 // #define USE_BARO_SPI_BMP280
// resource BEEPER 1 B02
pin_out0 PC8 // resource MOTOR 1 C08
pin_out1 PC9 // resource MOTOR 2 C09
pin_out2 PB6 // resource MOTOR 3 B06
pin_out3 PB7 // resource MOTOR 4 B07
pin_rcl_ppm PB4 // resource PPM 1 B04
// resource LED_STRIP 1 B03
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
pin_i2c0_scl PB8 // resource I2C_SCL 1 B08
pin_i2c0_sda PB9 // resource I2C_SDA 1 B09
pin_led PC15 // resource LED 1 C15
// resource LED 2 C14
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi2_sclk PC10 // resource SPI_SCK 3 C10
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PB14 // resource SPI_MISO 2 B14
pin_spi2_miso PC11 // resource SPI_MISO 3 C11
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi1_mosi PB15 // resource SPI_MOSI 2 B15
pin_spi2_mosi PB5 // resource SPI_MOSI 3 B05
// resource CAMERA_CONTROL 1 A08
pin_bat_v PC1 // resource ADC_BATT 1 C01
bat_gizmo ADC
// resource ADC_RSSI 1 C02
pin_bat_i PC3 // resource ADC_CURR 1 C03
bat_gizmo ADC
// resource BARO_CS 1 B01
// resource PINIO 1 A14
// resource FLASH_CS 1 A15
// resource OSD_CS 1 B12
pin_imu_int PC4 // resource GYRO_EXTI 1 C04
// resource GYRO_EXTI 2 C00
pin_imu_cs PA4 // resource GYRO_CS 1 A04
// resource GYRO_CS 2 C13
// set mag_bustype = I2C
mag_i2c_bus 0 // set mag_i2c_device = 1
// set baro_spi_device = 2
// set serialrx_provider = SBUS
// set adc_device = 2
// set blackbox_device = SPIFLASH
// set current_meter = ADC
// set battery_meter = ADC
// set ibata_scale = 290
// set beeper_inversion = ON
// set beeper_od = OFF
// set pid_process_denom = 1
// set osd_rssi_pos = 2445
// set osd_tim_2_pos = 2133
// set osd_crosshairs_pos = 2285
// set osd_current_pos = 2156
// set osd_warnings_pos = 14761
// set osd_avg_cell_voltage_pos = 2114
// set max7456_spi_bus = 2
// set pinio_config = 129,1,1,1
// set pinio_box = 40,255,255,255
// set flash_spi_bus = 3
// set gyro_1_bustype = SPI
imu_spi_bus 0 // set gyro_1_spibus = 1
imu_align CW90FLIP // set gyro_1_sensor_align = CW90FLIP
// set gyro_1_align_pitch = 1800
// set gyro_1_align_yaw = 900
)""; //end of madflight_board


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F7X2 (S7X2) 4.1.0 Oct 16 2019 / 11:58:45 (c37a7c91a) MSP API: 1.42

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACCGYRO_BMI270
#define USE_BARO
#define USE_BARO_SPI_BMP280
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

board_name SKYSTARSF7HDPRO
manufacturer_id SKST

# resources
resource BEEPER 1 B02
resource MOTOR 1 C08
resource MOTOR 2 C09
resource MOTOR 3 B06
resource MOTOR 4 B07
resource PPM 1 B04
resource LED_STRIP 1 B03
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
resource I2C_SCL 1 B08
resource I2C_SDA 1 B09
resource LED 1 C15
resource LED 2 C14
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 C10
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 C11
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 B05
resource CAMERA_CONTROL 1 A08
resource ADC_BATT 1 C01
resource ADC_RSSI 1 C02
resource ADC_CURR 1 C03
resource BARO_CS 1 B01
resource PINIO 1 A14
resource FLASH_CS 1 A15
resource OSD_CS 1 B12
resource GYRO_EXTI 1 C04
resource GYRO_EXTI 2 C00
resource GYRO_CS 1 A04
resource GYRO_CS 2 C13

# timer
timer B04 AF2
# pin B04: TIM3 CH1 (AF2)
timer C08 AF3
# pin C08: TIM8 CH3 (AF3)
timer C09 AF3
# pin C09: TIM8 CH4 (AF3)
timer A08 AF1
# pin A08: TIM1 CH1 (AF1)
timer B06 AF2
# pin B06: TIM4 CH1 (AF2)
timer B07 AF2
# pin B07: TIM4 CH2 (AF2)
timer B03 AF1
# pin B03: TIM2 CH2 (AF1)

# dma
dma SPI_TX 2 0
# SPI_TX 2: DMA1 Stream 4 Channel 0
dma SPI_TX 3 0
# SPI_TX 3: DMA1 Stream 5 Channel 0
dma ADC 2 1
# ADC 2: DMA2 Stream 3 Channel 1
dma pin C08 0
# pin C08: DMA2 Stream 2 Channel 0
dma pin C09 0
# pin C09: DMA2 Stream 7 Channel 7
dma pin B06 0
# pin B06: DMA1 Stream 0 Channel 2
dma pin B07 0
# pin B07: DMA1 Stream 3 Channel 2
dma pin B03 0
# pin B03: DMA1 Stream 6 Channel 3

# feature
feature RX_SERIAL
feature TELEMETRY
feature LED_STRIP
feature OSD

# serial
serial 0 64 115200 57600 0 115200
serial 3 1 115200 57600 0 115200
serial 5 2048 115200 57600 0 115200

# aux
aux 0 0 0 1700 2100 0 0
aux 1 13 1 1700 2100 0 0
aux 2 28 1 1300 1700 0 0
aux 3 40 2 1700 2100 0 0

# master
set mag_bustype = I2C
set mag_i2c_device = 1
set baro_spi_device = 2
set serialrx_provider = SBUS
set adc_device = 2
set blackbox_device = SPIFLASH
set current_meter = ADC
set battery_meter = ADC
set ibata_scale = 290
set beeper_inversion = ON
set beeper_od = OFF
set pid_process_denom = 1
set osd_rssi_pos = 2445
set osd_tim_2_pos = 2133
set osd_crosshairs_pos = 2285
set osd_current_pos = 2156
set osd_warnings_pos = 14761
set osd_avg_cell_voltage_pos = 2114
set max7456_spi_bus = 2
set pinio_config = 129,1,1,1
set pinio_box = 40,255,255,255
set flash_spi_bus = 3
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW90FLIP
set gyro_1_align_pitch = 1800
set gyro_1_align_yaw = 900

*/
