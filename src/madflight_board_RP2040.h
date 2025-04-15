

#ifdef PICO_RP2350
  #define MF_BOARD_NAME "Raspberry Pi Pico2 (default board)"
  #define MF_MCU_NAME "RP2350"
#else
  #define MF_BOARD_NAME "Raspberry Pi Pico (default board)"
  #define MF_MCU_NAME "RP2040"
#endif

const char* madflight_board = R""(

// PINOUT

// Serial Pins
pin_ser0_rx     1 // uart0: 1(default), 5, 13, 17
pin_ser0_tx     0 // uart0: 0(default), 4, 12, 16
pin_ser0_inv   -1
pin_ser1_rx     9 // uart1: 5, 9(default)
pin_ser1_tx     8 // uart1: 4, 8(default)
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
pin_spi0_miso  16 // spi0: 0, 4, 16(default)
pin_spi0_mosi  19 // spi0: 3, 7, 19(default)
pin_spi0_sclk  18 // spi0: 2, 6, 18(default)
pin_spi1_miso  12 // spi1: 8, 12(default)
pin_spi1_mosi  15 // spi1: 11, 15(default)
pin_spi1_sclk  14 // spi1: 10, 14(default)
pin_spi2_miso  -1
pin_spi2_mosi  -1
pin_spi2_sclk  -1
pin_spi3_miso  -1
pin_spi3_mosi  -1
pin_spi3_sclk  -1

// I2C Pins
pin_i2c0_sda   20 // i2c0: 0, 4(default), 8, 12, 16, 20
pin_i2c0_scl   21 // i2c0: 1, 5(default), 9, 13, 17, 21
pin_i2c1_sda   10 // i2c1: 2, 6, 10, 14, 18, 26(default)
pin_i2c1_scl   11 // i2c1: 3, 7, 11, 15, 19, 27(default)
pin_i2c2_sda   -1
pin_i2c2_scl   -1
pin_i2c3_sda   -1
pin_i2c3_scl   -1

// OUT Pins
pin_out0        2
pin_out1        3
pin_out2        4
pin_out3        5
pin_out4        6
pin_out5        7
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
pin_bat_i      27
pin_bat_v      28
pin_bbx_cs     13 // spi1: 9, 13(default)
pin_imu_cs     17 // spi0: 1, 5, 17(default)
pin_imu_int    22
pin_led        25
led_on      HIGH_IS_ON
pin_mmc_dat    -1
pin_mmc_clk    -1
pin_mmc_cmd    -1
pin_rcl_ppm    -1

// BUSSES

// Serial Busses
rcl_ser_bus     0
gps_ser_bus     1
rdr_ser_bus     2

// SPI Busses
imu_spi_bus    -1 //setup in .ino
bbx_spi_bus     1

// I2C Busses
imu_i2c_bus    -1 //setup in .ino
bar_i2c_bus     0
mag_i2c_bus     0
bat_i2c_bus     0

)""; //end of madflight_board
