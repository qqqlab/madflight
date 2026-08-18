/*========================================================================================================================
                                              MADFLIGHT CONFIG
==========================================================================================================================

The configuration is loaded from two strings: first from madflight_board in from the board header in /brd/, and then from 
madflight_config in this file. If the same setting occurs more than once, the last one is applied.

The strings are multi-line raw strings in with a key-value list. Anything after '#' or '/' is ignored as comment.

You have 3 options to setup the flight controller:

  1) Default - Keep #define MF_BOARD "brd/default.h", this defines the default pinout as shown on https://madflight.com 
     for the supported processor families. Now edit madflight_config below, and uncomment and configure imu_gizmo, 
     imu_bus_type, rcl_gizmo, and other lines as needed. (Do not change the pin_xxx_yyy and xxx_yyy_bus settings.)
 
  2) BetaFlight - Change "brd/default.h" to the BetaFlight flight controller you want to use, for example: 
     #define MF_BOARD "brd/betaflight/MTKS-MATEKH743.h". See library/madflight/src for all available boards. Edit 
     madflight_config to fine-tune the configuration.
 
  3) Bare Metal - Comment out "#define MF_BOARD", and set the full configuration in madflight_config below.

Pins and spi/i2c/serial busses use zero-based numbering, i.e. "gps_ser_bus 0" connects the GPS to the first serial bus
with pins pin_ser0_tx and pin_ser0_rx. Pins use GPIO numbers, not physical pin numbers. Use -1 to disable a pin or bus.

You can also modify the configuration from the CLI, for example "set imu_gizmo MPU6500" or "set imu_spi_bus 1", then
use "save" to save the config to eeprom and reboot to use the new config.

If things do not work as expected, have a good look at the startup messages!

========================================================================================================================*/

// define the board header to use, or comment out for none
#define MF_BOARD "brd/default.h"

const char madflight_config[] = R""(

//--- RCL --- Remote Controller Link (modify as required)
rcl_gizmo      CRSF  // options: NONE, MAVLINK, CRSF, SBUS, DSM, PPM
rcl_ser_bus    0  
rcl_num_ch     16
rcl_deadband   0
pin_rcl_ppm   -1

//flightmode mapping from 2/3/6-pos switch to flight mode (simulates a 3-pos switch: MANUAL/ROLL/FBWA)
rcl_flt0 PLANE_MANUAL
rcl_flt1 PLANE_MANUAL
rcl_flt2 PLANE_ROLL
rcl_flt3 PLANE_ROLL
rcl_flt4 PLANE_FBWA
rcl_flt5 PLANE_FBWA

//--- IMU --- Inertial Measurement Unit  (use spi -OR- i2c bus)
//imu_gizmo      NONE  // options: NONE, BMI270, ICM42688, ICM45686, MPU6000, MPU6050, MPU6500, MPU9150, MPU9250
//imu_bus_type   SPI   // options: SPI, I2C (not all combinations of gizmo and bus_type are supported)
//imu_align      CW0   // options: CW0, CW90, CW180, CW270, CW0FLIP, CW90FLIP, CW180FLIP, CW270FLIP
//imu_spi_bus    -1    // spi
//pin_imu_cs     -1    // spi
//pin_imu_int    -1    // spi and i2c
//imu_i2c_bus    -1    // i2c
//imu_i2c_adr     0    // i2c: enter decimal i2c address, not hex (use 0 for default i2c address)

// IMPORTANT: the IMU sensor should be the ONLY sensor on the selected bus, interrupt pin is required

//--- BAR --- Barometer
//bar_gizmo      NONE  // options: NONE, BMP280, BMP388, BMP390, BMP580, HP203B, MS5611
//bar_i2c_adr    0
//bar_i2c_bus   -1

//--- MAG --- Magnetometer
//mag_gizmo      NONE  // options: NONE, QMC6309, QMC5883L, QMC5883P, RM3100, BMM150

//mag_align      CW0   // options: CW0, CW90, CW180, CW270, CW0FLIP, CW90FLIP, CW180FLIP, CW270FLIP
//mag_i2c_adr    0
//mag_i2c_bus   -1

//--- BAT --- Battery Monitor  (use i2c bus -OR- adc pins)
//bat_gizmo      NONE  // options: NONE, ADC, INA226, INA228
//bat_i2c_adr    0     // i2c
//bat_i2c_bus   -1     // i2c
//pin_bat_i     -1     // adc
//pin_bat_v     -1     // adc
//bat_cal_v      1     // for ADC: voltage scale, value is: actual_voltage_in_v / adc_reading; for INA226/228: not used
//bat_cal_i,     1     // for ADC: current scale, value is: actual_current_in_a / adc_reading; for INA226/228: rshunt value in ohm

//--- GPS ---
//gps_gizmo      NONE  // options: NONE, UBLOX
//gps_baud       0     // use 0 for auto baud
//gps_ser_bus   -1

//--- BBX --- Black Box Data Logger  (use spi -OR- mmc)
//bbx_gizmo      NONE  // options: NONE, SDSPI, SDMMC, OPENLOG
//pin_bbx_cs    -1     // spi
//bbx_spi_bus   -1     // spi
//pin_mmc_dat   -1     // mmc
//pin_mmc_clk   -1     // mmc
//pin_mmc_cmd   -1     // mmc
//bbx_ser_bus   -1     // openlog
//bbx_baud       0     // openlog, use 0 for default 115200 baud

//--- RDR --- Radar (use serial bus -OR- trig+echo pins)
//rdr_gizmo      NONE  // options: NONE, DTS6012M, LD2411S, LD2413, SR04, USD1
//rdr_ser_bus   -1     // serial
//rdr_baud       0     // serial, use 0 for default baud
//pin_rdr_trig  -1     // trig+echo
//pin_rdr_echo  -1     // trig+echo

//--- OFL --- Optical FLow (use serial bus -OR- spi bus)
//ofl_gizmo      NONE  // options: NONE, PMW3901, PMW3901U
//ofl_ser_bus   -1     // serial
//ofl_baud       0     // serial, use 0 for default baud
//ofl_spi_bus   -1     // spi
//pin_ofl_cs    -1     // spi

//--- LED ---
//led_gizmo     NONE   // options: NONE, HIGH_IS_ON, LOW_IS_ON, RGB
//pin_led       -1

//--- AHR --- AHRS (keep MAHONY, unless you want to experiment)
//ahr_gizmo      MAHONY // options: MAHONY, MAHONY_BF, MADGWICK, VQF

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

//uncomment to disable receiver stick expo and center sensitivity. Then set trim + expo on transmitter as needed
//rcl_rol_expo 0
//rcl_pit_expo 0
//rcl_yaw_expo 0
//rcl_rol_sens 0
//rcl_pit_sens 0
//rcl_yaw_sens 0

//reset PID parameters to default (remove this section if you want to use your own settings)
pid_gizmo        BASIC
pid_angl_mult    -1.000000
pid_filt0_freq   -1.000000
pid_filt0_q      -1.000000
pid_filt0_type   NONE
pid_filt1_freq   -1.000000
pid_filt1_q      -1.000000
pid_filt1_type   NONE
pid_filt2_freq   -1.000000
pid_filt2_q      -1.000000
pid_filt2_type   NONE
pid_filt3_freq   -1.000000
pid_filt3_q      -1.000000
pid_filt3_type   NONE
pid_i_limit      -1.000000
pid_ka0          -1.000000
pid_ka1          -1.000000
pid_ka2          -1.000000
pid_ka3          -1.000000
pid_kb0          -1.000000
pid_kb1          -1.000000
pid_kb2          -1.000000
pid_kb3          -1.000000
pid_kd0          -1.000000
pid_kd1          -1.000000
pid_kd2          -1.000000
pid_kd3          -1.000000
pid_ki0          -1.000000
pid_ki1          -1.000000
pid_ki2          -1.000000
pid_ki3          -1.000000
pid_kp0          -1.000000
pid_kp1          -1.000000
pid_kp2          -1.000000
pid_kp3          -1.000000
pid_pit_angl_lim -1.000000
pid_pit_rate_lim -1.000000
pid_rol_angl_lim -1.000000
pid_rol_rate_lim -1.000000
pid_yaw_rate_lim -1.000000
)""; // End of madflight_config


//========================================================================================================================//
//                                               COMPILER OPTIONS                                                         //
//========================================================================================================================//

// Reset config eeprom to defaults (uncomment this, upload, execute, then comment out, and upload again)
//#define MF_CONFIG_CLEAR

// Uncomment to reduce startup delay
//#define MF_DEBUG
