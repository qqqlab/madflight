//This pin layout for the Black Pill board is based on MATEK F411SE (MTKS-MATEKF411SE betaflight target)
//see https://madflight.com for details

//Arduino F411 defines: -DSTM32F4xx -DARDUINO=10607 -DARDUINO_GENERIC_F411CEUX -DARDUINO_ARCH_STM32 -DBOARD_NAME="GENERIC_F411CEUX" -DVARIANT_H="variant_generic.h" -DSTM32F411xE -DUSBCON -DUSBD_VID=0 -DUSBD_PID=0 -DHAL_PCD_MODULE_ENABLED -DUSBD_USE_CDC -DHAL_UART_MODULE_ENABLED

//Arduino F411: Serial and Serial1 both map TX1/RX1 on pin A9/A10.
//Arduino F411: Serial debug on USB Serial port (USB is on on PA11,PA12, shared with USART6)

/* BUSSES
3x USART
USART1_TX:PA9,PA15,PB6
USART1_RX:PA10,PB3,PB7
USART2_TX:PA2
USART2_RX:PA3
USART6_TX:PA11,PC6
USART6_RX:PA12,PC7

3x I2C
I2C1_SCL:PB6,PB8
I2C2_SDA:PB7,PB9
I2C2_SCL:PB10
I2C2_SDA:[PB11],PB3,PB9
I2C3_SCL:PA8,PB8
I2C3_SDA:PC9,PB4

5x SPI
SPI1_SCK:PA5,PB3
SPI1_MISO:PA6,PB4
SPI1_MOSI:PA7,PB5
SPI2_SCK:PB10,PB13,PC7,PD3
SPI2_MISO:PB14,PC2
SPI2_MOSI:PB15,PC3
SPI3_SCK:PB3,PC10
SPI3_MISO:PB4,PC11
SPI3_MOSI:PB5,PC12
SPI4_SCK:PE2,PE12,PB13
SPI4_MISO:PE5,PE13,PA10
SPI4_MOSI:PA1,PD6,PE6,PE14
SPI5_SCK:PB0,PE2,PE12
SPI5_MISO:PA12,PE5,PE13
SPI5_MOSI:PA10,PE6,PE14

*/

#pragma once

#define MF_BOARD_NAME "STM32F411CE Black Pill (default board)" 
#define MF_MCU_NAME "STM32F411CEUx" //STM32F411CEUx - not all pin combinations are allowed, see datasheet

const char* madflight_board = R""(

// PINOUT

// Serial Pins
pin_ser0_rx    PB3  //USART1
pin_ser0_tx    PA15 //USART1
pin_ser0_inv   -1
pin_ser1_rx    PA3  //USART2
pin_ser1_tx    PA2  //USART2
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
pin_spi0_miso  PA6  //SPI1
pin_spi0_mosi  PA7  //SPI1
pin_spi0_sclk  PA5  //SPI1
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
pin_i2c0_sda   PB7 //I2C1
pin_i2c0_scl   PB6 //I2C1
pin_i2c1_sda   PB9 //I2C2
pin_i2c1_scl   PB10 //I2C2
pin_i2c2_sda   -1
pin_i2c2_scl   -1
pin_i2c3_sda   -1
pin_i2c3_scl   -1

// OUT Pins
pin_out0       PB13
pin_out1       PB14
pin_out2       PA15
pin_out3       PA8
pin_out4       PA9
pin_out5       PA10
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
pin_bat_i      PB1
pin_bat_v      PB0
pin_bbx_cs     -1
pin_imu_cs     PA4
pin_imu_int    PB2
pin_led        PC13
led_on      LOW_IS_ON
pin_mmc_dat    -1
pin_mmc_clk    -1
pin_mmc_cmd    -1
pin_rcl_ppm    -1

// BUSSES

// Serial Busses
rcl_ser_bus    0
gps_ser_bus    1
rdr_ser_bus    -1

// SPI Busses
imu_spi_bus    -1 //setup in .ino
bbx_spi_bus    -1

// I2C Busses
imu_i2c_bus    -1 //setup in .ino
bar_i2c_bus    0
mag_i2c_bus    0
bat_i2c_bus    0

)""; //end of madflight_board
