/*========================================================================================================================
                                              MADFLIGHT CONFIG                                                          
==========================================================================================================================

MADFLIGHT_CONFIG is a multi-line raw string in with a key-value list. Anything after '#' or '/' is ignored as comment.

You have 4 options to setup the pins (gpio numbers) and serial/spi/i2c busses for the flight controller:

  1) Default - #include <madflight_board.h> and see https://madflight.com for default pinout diagrams for the supported
     processor families. Default pinouts are defined in the board header files library/madflight/src/madflight_board_XXX.h
     Leave lines with "pin_..." and "..._bus" commented in MADFLIGHT_CONFIG. 
 
  2) BetaFlight - #include the BetaFlight flight controller you want to use. See library/madflight/src for all available 
     boards. For example: #include <madflight_zzz_MTKS-MATEKH743.h>

  3) Override - Keep #include <madflight_board.h>, and uncomment lines in MADFLIGHT_CONFIG to override the board settings.
 
  4) Custom - Remove #include <madflight_board.h>, and set the full configuration in MADFLIGHT_CONFIG below.

Pins and busses use zero-based numbering, i.e. imu_spi_bus 0 is the first spi bus. Use -1 to disable a pin or bus.

You can also modify the configuration from the CLI, for example "set imu_gizmo MPU6500" or "set imu_spi_bus 1", then
use "cwrite" to save the config to eeprom and reboot to use the new config.

If things do not work as expected, have a good look at the startup messages!

========================================================================================================================*/

#include <madflight_board.h>

#define MADFLIGHT_CONFIG R""(

//--- IMU --- Inertial Measurement Unit  (use spi -OR- i2c bus)
imu_gizmo      MPU6500  // options: NONE, BMI270, MPU6000, MPU6050, MPU6500, MPU9150, MPU9250 
imu_bus_type   SPI      // options: SPI, I2C (not all combinations of gizmo and bus_type are supported)
imu_align      CW90     // options: CW0, CW90, CW180, CW270, CW0FLIP, CW90FLIP, CW180FLIP, CW270FLIP
//imu_spi_bus    -1 //spi
//pin_imu_cs     -1 //spi
//pin_imu_int    -1 //spi
//imu_i2c_bus    -1 //i2c
//imu_i2c_adr     0 //i2c: enter decimal i2c address, not hex (use 0 for default i2c address)

// IMPORTANT: the IMU sensor should be the ONLY sensor on the selected bus


//--- RCL --- Remote Controller Link  (use serial bus -OR- ppm pin)
rcl_gizmo      CRSF  // options: NONE, MAVLINK, CRSF, SBUS, DSM, PPM
rcl_num_ch     8     // number of channels
rcl_deadband   0     // center stick deadband
//rcl_ser_bus   -1
//pin_rcl_ppm   -1

//--- BAR --- Barometer
bar_gizmo      NONE  // options: NONE, BMP390, BMP388, BMP280, MS5611, HP203B
//bar_i2c_adr    0
//bar_i2c_bus   -1

//--- MAG --- Magnetometer
mag_gizmo      NONE  // options: NONE, QMC5883, QMC6309, RM3100
//mag_i2c_adr    0
//mag_i2c_bus   -1

//--- BAT --- Battery Monitor  (use i2c bus -OR- adc pins)
bat_gizmo      NONE  // options: NONE, ADC, INA226, INA228
//bat_i2c_adr    0
//bat_i2c_bus   -1
//pin_bat_i     -1
//pin_bat_v     -1

//--- GPS ---
gps_gizmo      NONE  // options: NONE, UBLOX
//gps_baud       0   // use 0 for auto baud
//gps_ser_bus   -1

//--- BBX --- Black Box Data Logger  (use spi -OR- mmc)
bbx_gizmo      NONE  // options: NONE, SDSPI, SDMMC
//pin_bbx_cs    -1  // spi
//bbx_spi_bus   -1  // spi
//pin_mmc_dat   -1  // mmc
//pin_mmc_clk   -1  // mmc
//pin_mmc_cmd   -1  // mmc

//--- RDR --- Radar (use serial bus -OR- trig+echo pins)
rdr_gizmo      NONE  // options: NONE, LD2411S, LD2413, USD1, SR04
//rdr_baud       0
//rdr_ser_bus   -1
//pin_rdr_trig  -1
//pin_rdr_echo  -1

//--- LED ---
//led_on         LOW_IS_OFF // options: LOW_IS_OFF, LOW_IS_ON
//pin_led       -1

//--- AHR --- AHRS (keep MAHONY, unless you want to experiment)
ahr_gizmo      MAHONY  // options: MAHONY, MAHONY_BF, MADGWICK, VQF

//--- Serial bus 0 ---
//pin_ser0_rx   -1
//pin_ser0_tx   -1

//--- Serial bus 1 ---
//pin_ser1_rx   -1
//pin_ser1_tx   -1 

//--- SPI bus 0 ---
//pin_spi0_miso -1
//pin_spi0_mosi -1
//pin_spi0_sclk -1

//--- SPI bus 1 ---
//pin_spi1_miso -1
//pin_spi1_mosi -1
//pin_spi1_sclk -1

//--- I2C Bus 0 ---
//pin_i2c0_sda  -1
//pin_i2c0_scl  -1

//--- I2C Bus 1 ---
//pin_i2c1_sda  -1
//pin_i2c1_scl  -1

//--- OUT Pins ---
//pin_out0      -1
//pin_out1      -1
//pin_out2      -1
//pin_out3      -1
//pin_out4      -1
//pin_out5      -1
//pin_out6      -1
//pin_out7      -1
//pin_out8      -1
//pin_out9      -1
//pin_out10     -1
//pin_out11     -1
//pin_out12     -1
//pin_out13     -1
//pin_out14     -1
//pin_out15     -1

)"" // End of MADFLIGHT_CONFIG


//========================================================================================================================//
//                                               COMPILER OPTIONS                                                         //
//========================================================================================================================//

// Reset config eeprom to defaults (uncomment this, upload, execute, then comment out, and upload again)
//#define MF_CONFIG_CLEAR

// Uncomment to print additional debug information and reduce startup delay
//#define MF_DEBUG
