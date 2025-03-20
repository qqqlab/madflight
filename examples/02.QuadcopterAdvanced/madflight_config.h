//========================================================================================================================//
//                                               PINS CONFIG                                                              //
//========================================================================================================================//
//
// You have 3 options to setup the pins (gpio numbers) and busses for the flight controller:
//
//   1) Default - #include <madflight_board.h> and see https://madflight.com for default pinout diagrams for the supported
//      processor families. Default pinouts are defined in the board header files library/src/madflight_board_default_XXX.h
// 
//   2) Header - #include the BetaFlight flight controller you want to use. See library/madflight/src for all available 
//      boards. For example: #include <madflight_board_betaflight_MTKS-MATEKH743.h>
// 
//   3) Custom - Do not include a board file here, and set your own board definition in the CUSTOM PINS section below.
//
//========================================================================================================================//

#include <madflight_board.h>


//========================================================================================================================//
//                                               HARDWARE CONFIG                                                          //
//========================================================================================================================//
//
// Hardware configuration is a simple key-value list. Anything after '#' or '/' is ignored as comment
//
//========================================================================================================================//

#define MADFLIGHT_CONFIG R""(

// IMU - Inertial Measurement Unit (acc/gyro)

// Uncomment ONE bus: SPI or I2C
imu_spi_bus    0        // connect IMU to SPI bus 0
//imu_i2c_bus    1        // connect IMU to I2C bus 1

//NOTE: the IMU sensor should be the ONLY sensor on the selected bus

imu_gizmo      MPU6500  // options: NONE, BMI270, MPU6000, MPU6050, MPU6500, MPU9150, MPU9250 
imu_align      CW90     // options: CW0, CW90, CW180, CW270, CW0FLIP, CW90FLIP, CW180FLIP, CW270FLIP
imu_i2c_adr    0        // use 0 for default i2c address

// RCL - Remote Controller Link
rcl_gizmo      SBUS     // options: NONE, MAVLINK, CRSF, SBUS, DSM, PPM, PWM
rcl_num_ch     8        // number of channels
rcl_deadband   0        // center stick deadband

// BAR - Barometer
bar_gizmo      BMP280   // options: NONE, BMP390, BMP388, BMP280, MS5611
bar_i2c_adr    0

// MAG - Magnetometer
mag_gizmo      QMC5883  // options: NONE, QMC5883, MS5611
mag_i2c_adr    0

// BAT - Battery Monitor
bat_gizmo      INA226   // options: NONE, ADC, INA226, INA228
bat_i2c_adr    0

// GPS
gps_gizmo      UBLOX    // options: NONE, UBLOX
gps_baud       0        // use 0 for auto baud

// BBX - Black Box Data Logger
bbx_gizmo      NONE     // options: NONE, SDSPI, SDMMC

)"" // End of MADFLIGHT_CONFIG


//========================================================================================================================//
//                                               CUSTOM PINS CONFIG                                                       //
//========================================================================================================================//

/*  <-- Add a / here to setup your custom pins below - Check datasheet that what you're doing is actually possible!!!

#define MADFLIGHT_BOARD R""(

// PINOUT

// Serial Pins
pin_ser0_rx    -1
pin_ser0_tx    -1
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
pin_spi0_miso  -1
pin_spi0_mosi  -1
pin_spi0_sclk  -1
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
pin_i2c0_sda   -1
pin_i2c0_scl   -1
pin_i2c1_sda   -1
pin_i2c1_scl   -1
pin_i2c2_sda   -1
pin_i2c2_scl   -1
pin_i2c3_sda   -1
pin_i2c3_scl   -1

// OUT Pins
pin_out0       -1
pin_out1       -1
pin_out2       -1
pin_out3       -1
pin_out4       -1
pin_out5       -1
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
pin_imu_cs     -1
pin_imu_int    -1
pin_led        -1
led_on       LOW_IS_ON
pin_mmc_dat    -1
pin_mmc_clk    -1
pin_mmc_cmd    -1
pin_rcl_ppm    -1

// BUSSES

// Serial Busses
rcl_ser_bus    -1
gps_ser_bus    -1
rdr_ser_bus    -1

// SPI Busses
imu_spi_bus    -1
bbx_spi_bus    -1

// I2C Busses
bar_i2c_bus    -1
mag_i2c_bus    -1
bat_i2c_bus    -1
imu_i2c_bus    -1

)"" // end of MADFLIGHT_BOARD */

//========================================================================================================================//
//                                               COMPILER OPTIONS                                                         //
//========================================================================================================================//

//-- AHRS sensor fusion 
#define AHR_USE AHR_USE_MAHONY // Select one: AHRS_USE_MAHONY, AHRS_USE_MAHONY_BF, AHRS_USE_MADGWICK, AHRS_USE_VQF
