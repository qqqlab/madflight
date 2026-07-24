/* Configuration for: madflight SM1 Rev-A Sensor Module + ESP32-S3 SuperMini Board

Specifications:

MCU: ESP32-S3
IMU: LSM6DSV
BAR: HP203A
MAG: QMC6309
LED: WS2812B

*/

#pragma once

#define MF_BOARD_NAME "madflight SM1A + ESP32-S3 SuperMini"
#define MF_MCU_NAME "ESP32-S3"

const char madflight_board[] = R""(

// PINOUT

// Serial Pins
pin_ser0_rx    44
pin_ser0_tx    43
pin_ser0_inv   -1
pin_ser1_rx    -1
pin_ser1_tx    -1
pin_ser1_inv   -1
pin_ser2_rx    -1
pin_ser2_tx    -1
pin_ser2_inv   -1
pin_ser3_rx    -1
pin_ser3_tx    -1
pin_ser3_inv   -1
pin_ser4_rx    -1
pin_ser4_tx    -1
pin_ser4_inv   -1
pin_ser5_rx    -1
pin_ser5_tx    -1
pin_ser5_inv   -1
pin_ser6_rx    -1
pin_ser6_tx    -1
pin_ser6_inv   -1
pin_ser7_rx    -1
pin_ser7_tx    -1
pin_ser7_inv   -1

// SPI Pins
pin_spi0_miso   3
pin_spi0_mosi   6
pin_spi0_sclk   5
pin_spi1_miso  -1
pin_spi1_mosi  -1
pin_spi1_sclk  -1
pin_spi2_miso  -1
pin_spi2_mosi  -1
pin_spi2_sclk  -1
pin_spi3_miso  -1
pin_spi3_mosi  -1
pin_spi3_sclk  -1

// I2C Pins
pin_i2c0_sda    1
pin_i2c0_scl    2
pin_i2c1_sda   -1
pin_i2c1_scl   -1
pin_i2c2_sda   -1
pin_i2c2_scl   -1
pin_i2c3_sda   -1
pin_i2c3_scl   -1

// OUT Pins
pin_out0       13
pin_out1       12
pin_out2       11
pin_out3       10
pin_out4        9
pin_out5        8
pin_out6       -1
pin_out7       -1
pin_out8       -1
pin_out9       -1
pin_out10      -1
pin_out11      -1
pin_out12      -1
pin_out13      -1
pin_out14      -1
pin_out15      -1

// Other Pins
pin_bat_i      -1
pin_bat_v      -1
pin_bbx_cs     -1

//--- LED --- 
led_gizmo     RGB // options: NONE, HIGH_IS_ON, LOW_IS_ON, RGB
pin_led        48

//--- RCL --- Radio Receiver Link
rcl_gizmo CRSF
rcl_ser_bus     0

//--- IMU --- Inertial Measurement Unit  (use spi -OR- i2c bus)
imu_gizmo      AUTO
imu_bus_type   SPI     // options: SPI, I2C (not all combinations of gizmo and bus_type are supported)
imu_align      CW0     //board edge with pins is forward direction - options: CW0, CW90, CW180, CW270, CW0FLIP, CW90FLIP, CW180FLIP, CW270FLIP
imu_spi_bus     0 //spi
pin_imu_cs      4 //spi
pin_imu_int     7 //spi and i2c
imu_i2c_bus    -1 //i2c
imu_i2c_adr     0 //i2c: enter decimal i2c address, not hex (use 0 for default i2c address)

// IMPORTANT: the IMU sensor should be the ONLY sensor on the selected bus


//--- BAR --- Barometer
bar_gizmo      HP203B
bar_i2c_adr    118  //always 118 (0x76) for HP203B
bar_i2c_bus    0

//--- MAG --- Magnetometer
mag_gizmo      QMC6309
mag_align      CW180   //board edge with pins is forward direction - options: CW0, CW90, CW180, CW270, CW0FLIP, CW90FLIP, CW180FLIP, CW270FLIP
mag_i2c_adr    124   //always 124 (0x7C) for QMC6309
mag_i2c_bus    0

)""; //end of madflight_board
