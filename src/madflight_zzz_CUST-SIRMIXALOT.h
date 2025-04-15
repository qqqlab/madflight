/*==============================================================================
Generated on: 2025-04-16 00:58:15.112601
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: SIRMIXALOT
Manufacturer ID: CUST

//copy this line to madflight.ino to use this flight controller
#include <madflight_zzz_CUST-SIRMIXALOT.h>

Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out, or use madflight_config to override.
==============================================================================*/

#define MF_BOARD_NAME "BETAFLIGHT-CUST-SIRMIXALOT"
#define MF_MCU_NAME "STM32F7X2"

const char* madflight_board = R""(
imu_bus_type SPI
// resource BEEPER 1 C11
pin_out0 PB6 // resource MOTOR 1 B06
pin_out1 PB7 // resource MOTOR 2 B07
pin_out2 PB8 // resource MOTOR 3 B08
pin_out3 PA15 // resource MOTOR 4 A15
// resource LED_STRIP 1 B01
pin_ser0_tx PA9 // resource SERIAL_TX 1 A09
pin_ser1_tx PA2 // resource SERIAL_TX 2 A02
pin_ser2_tx PB10 // resource SERIAL_TX 3 B10
pin_ser4_tx PC12 // resource SERIAL_TX 5 C12
pin_ser5_tx PC6 // resource SERIAL_TX 6 C06
pin_ser10_tx PA0 // resource SERIAL_TX 11 A00
pin_ser0_rx PA10 // resource SERIAL_RX 1 A10
pin_ser1_rx PA3 // resource SERIAL_RX 2 A03
pin_ser3_rx PA1 // resource SERIAL_RX 4 A01
pin_ser4_rx PD2 // resource SERIAL_RX 5 D02
pin_ser5_rx PC7 // resource SERIAL_RX 6 C07
pin_ser10_rx PB11 // resource SERIAL_RX 11 B11
pin_i2c0_scl PA8 // resource I2C_SCL 1 A08
pin_i2c0_sda PC9 // resource I2C_SDA 1 C09
pin_led PB12 // resource LED 1 B12
// resource LED 2 C08
pin_spi0_sclk PA5 // resource SPI_SCK 1 A05
pin_spi1_sclk PB13 // resource SPI_SCK 2 B13
pin_spi2_sclk PB3 // resource SPI_SCK 3 B03
pin_spi0_miso PA6 // resource SPI_MISO 1 A06
pin_spi1_miso PB14 // resource SPI_MISO 2 B14
pin_spi2_miso PB4 // resource SPI_MISO 3 B04
pin_spi0_mosi PA7 // resource SPI_MOSI 1 A07
pin_spi1_mosi PB15 // resource SPI_MOSI 2 B15
pin_spi2_mosi PB5 // resource SPI_MOSI 3 B05
// resource CAMERA_CONTROL 1 C10
pin_bat_v PC2 // resource ADC_BATT 1 C02
bat_gizmo ADC
// resource ADC_RSSI 1 C03
pin_bat_i PC1 // resource ADC_CURR 1 C01
bat_gizmo ADC
// resource BARO_CS 1 B09
// resource FLASH_CS 1 C00
// resource OSD_CS 1 C14
pin_imu_int PC4 // resource GYRO_EXTI 1 C04
pin_imu_cs PA4 // resource GYRO_CS 1 A04
// resource USB_DETECT 1 B02
// set mag_hardware = AUTO
// set baro_spi_device = 3
// set serialrx_provider = FPORT
// set serialrx_inverted = ON
// set serialrx_halfduplex = ON
// set blackbox_device = SPIFLASH
// set dshot_burst = ON
// set dshot_bidir = OFF
// set motor_pwm_protocol = DSHOT600
// set current_meter = ADC
// set battery_meter = ADC
// set vbat_scale = 80
// set ibata_scale = 210
// set beeper_inversion = ON
// set beeper_od = OFF
// set pid_process_denom = 1
// set ledstrip_visual_beeper = ON
// set osd_vbat_pos = 2403
// set osd_rssi_pos = 2436
// set osd_rssi_dbm_pos = 66
// set osd_tim_1_pos = 2071
// set osd_tim_2_pos = 2103
// set osd_flymode_pos = 2424
// set osd_vtx_channel_pos = 480
// set osd_craft_name_pos = 2058
// set osd_gps_speed_pos = 2359
// set osd_gps_sats_pos = 2392
// set osd_home_dir_pos = 2095
// set osd_altitude_pos = 2135
// set osd_warnings_pos = 14698
// set osd_disarmed_pos = 2250
// set vtx_band = 1
// set vtx_channel = 3
// set vtx_power = 1
// set vtx_freq = 5825
// set max7456_spi_bus = 3
// set led_inversion = 3
// set flash_spi_bus = 2
// set gyro_1_bustype = SPI
imu_spi_bus 0 // set gyro_1_spibus = 1
imu_align CW180 // set gyro_1_sensor_align = CW180
// set gyro_1_align_yaw = 1800
)""; //end of madflight_board


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F7X2 (S7X2) 4.1.0 Sep 25 2019 / 14:09:57 (08e8afa09) MSP API: 1.42
# manufacturer_id: CUST   board_name: SIRMIXALOT   custom defaults: NO

board_name SIRMIXALOT
manufacturer_id CUST

# resources
resource BEEPER 1 C11
resource MOTOR 1 B06
resource MOTOR 2 B07
resource MOTOR 3 B08
resource MOTOR 4 A15
resource LED_STRIP 1 B01
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 A02
resource SERIAL_TX 3 B10
resource SERIAL_TX 5 C12
resource SERIAL_TX 6 C06
resource SERIAL_TX 11 A00
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 A03
resource SERIAL_RX 4 A01
resource SERIAL_RX 5 D02
resource SERIAL_RX 6 C07
resource SERIAL_RX 11 B11
resource I2C_SCL 1 A08
resource I2C_SDA 1 C09
resource LED 1 B12
resource LED 2 C08
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 B03
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 B04
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 B05
resource CAMERA_CONTROL 1 C10
resource ADC_BATT 1 C02
resource ADC_RSSI 1 C03
resource ADC_CURR 1 C01
resource BARO_CS 1 B09
resource FLASH_CS 1 C00
resource OSD_CS 1 C14
resource GYRO_EXTI 1 C04
resource GYRO_CS 1 A04
resource USB_DETECT 1 B02

# timer
timer B06 AF2
# pin B06: TIM4 CH1 (AF2)
timer B07 AF2
# pin B07: TIM4 CH2 (AF2)
timer B08 AF2
# pin B08: TIM4 CH3 (AF2)
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer A15 AF1
# pin A15: TIM2 CH1 (AF1)

# dma
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin B06 0
# pin B06: DMA1 Stream 0 Channel 2
dma pin B07 0
# pin B07: DMA1 Stream 3 Channel 2
dma pin B08 0
# pin B08: DMA1 Stream 7 Channel 2
dma pin B01 0
# pin B01: DMA1 Stream 2 Channel 5
dma pin A15 0
# pin A15: DMA1 Stream 5 Channel 3

# feature
feature RX_SERIAL
feature SOFTSERIAL
feature TELEMETRY
feature LED_STRIP
feature OSD

# beeper
beeper -ON_USB

# serial
serial 2 2048 115200 57600 0 115200
serial 3 1024 115200 57600 0 115200
serial 4 64 115200 57600 0 115200
serial 30 16384 115200 57600 0 115200

# led
led 0 7,7::A:2

# color
color 10 240,128,255

# mode_color
mode_color 6 1 2

# aux
aux 0 0 0 1700 2100 0 0
aux 1 27 8 1300 2100 0 0
aux 2 13 4 1300 2100 0 0
aux 3 15 5 1300 2100 0 0
aux 4 26 0 900 2100 0 0
aux 5 33 2 1300 1700 0 0
aux 6 34 3 1300 1700 0 0
aux 7 36 6 1700 2100 0 0

# master
set mag_hardware = AUTO
set baro_spi_device = 3
set serialrx_provider = FPORT
set serialrx_inverted = ON
set serialrx_halfduplex = ON
set blackbox_device = SPIFLASH
set dshot_burst = ON
set dshot_bidir = OFF
set motor_pwm_protocol = DSHOT600
set current_meter = ADC
set battery_meter = ADC
set vbat_scale = 80
set ibata_scale = 210
set beeper_inversion = ON
set beeper_od = OFF
set pid_process_denom = 1
set ledstrip_visual_beeper = ON
set osd_vbat_pos = 2403
set osd_rssi_pos = 2436
set osd_rssi_dbm_pos = 66
set osd_tim_1_pos = 2071
set osd_tim_2_pos = 2103
set osd_flymode_pos = 2424
set osd_vtx_channel_pos = 480
set osd_craft_name_pos = 2058
set osd_gps_speed_pos = 2359
set osd_gps_sats_pos = 2392
set osd_home_dir_pos = 2095
set osd_altitude_pos = 2135
set osd_warnings_pos = 14698
set osd_disarmed_pos = 2250
set vtx_band = 1
set vtx_channel = 3
set vtx_power = 1
set vtx_freq = 5825
set max7456_spi_bus = 3
set led_inversion = 3
set flash_spi_bus = 2
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW180
set gyro_1_align_yaw = 1800

*/
