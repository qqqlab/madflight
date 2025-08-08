//This pin layout is optimized for Espressif ESP32-S3-DevKitC-1 (44 pin) board, use "ESP32-S3 Dev Module" as board in Arduino IDE

//see https://madflight.com for details

//ESP32-S3 - Most pins can be assigned freely

//Arduino "Serial" is uart0 (pin 1 tx, pin 3 rx) connected to serial->USB converter and is used for programming/CLI
//you could use Serial0/uart0 as third serial port, when using USB (not serial->USB) for programming/CLI

#pragma once

#define MF_BOARD_NAME "ESP32-S3 DevKitC-1 (default board)"
#define MF_MCU_NAME "ESP32-S3"

const char madflight_board[] = R""(

// PINOUT

// Serial Pins
pin_ser0_rx    18 // Serial1/uart1
pin_ser0_tx    17 // Serial1/uart1
pin_ser0_inv   -1
pin_ser1_rx     3 // Serial2/uart2
pin_ser1_tx    46 // Serial2/uart2
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
pin_spi0_miso  12
pin_spi0_mosi  11
pin_spi0_sclk  13
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
pin_i2c0_sda    8
pin_i2c0_scl    9
pin_i2c1_sda   21
pin_i2c1_scl   47
pin_i2c2_sda   -1
pin_i2c2_scl   -1
pin_i2c3_sda   -1
pin_i2c3_scl   -1

// OUT Pins
pin_out0        4
pin_out1        5
pin_out2        6
pin_out3        7
pin_out4       15
pin_out5       16
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
pin_imu_cs     10
pin_imu_int    14
pin_led         2
led_gizmo     HIGH_IS_ON // options: NONE, HIGH_IS_ON, LOW_IS_ON, RGB
pin_mmc_dat    40
pin_mmc_clk    39
pin_mmc_cmd    38
pin_rcl_ppm    -1

// BUSSES

// Serial Busses
rcl_ser_bus     0
gps_ser_bus     1
rdr_ser_bus    -1

// SPI Busses
imu_spi_bus    -1  //setup in .ino
bbx_spi_bus    -1

// I2C Busses
imu_i2c_bus    -1 //setup in .ino
bar_i2c_bus     0
mag_i2c_bus     0
bat_i2c_bus     0

)""; //end of madflight_board
