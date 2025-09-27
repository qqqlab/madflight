/* configuration for madflight FC1 flight controller board

Specifications:

MCU: RP2350B with 48 GPIO, 32 GPIO accessible via pinheader
IMU: ICM-42688-P
BAR: HP203B
BAT: INA226
MAG: QMC6309
BBX: SDCARD (1-bit SPI or 4-bit SDIO)

*/
#include <Arduino.h> //needed for ARDUINO_PICO_MAJOR etc

#if !ARDUINO_ARCH_RP2040
  #error "Invalid Arduino Architecture: Select Arduino Pico"
#endif

#if (ARDUINO_PICO_MAJOR * 10000 + ARDUINO_PICO_MINOR * 100 + ARDUINO_PICO_REVISION) < 50100
  #error "Invalid Arduino Framework version: Install Arduino Pico version 5.1.0 or later"
#endif

#if !PICO_RP2350 || PICO_RP2350A
  #error "Invalid board: Select board Solder Party Stamp XL RP2350B"
#endif

#define MF_BOARD_NAME "madflight FC1"
#define MF_MCU_NAME "RP2350B (48 GPIO)"

const char madflight_board[] = R""(

//--- IMU --- Inertial Measurement Unit  (use spi -OR- i2c bus)
imu_gizmo     ICM42688    // options: NONE, BMI270, MPU6000, MPU6050, MPU6500, MPU9150, MPU9250, ICM45686, ICM42688
imu_bus_type  SPI     // options: SPI, I2C (not all combinations of gizmo and bus_type are supported)
imu_align     CW180   //board edge with pins is forward direction - options: CW0, CW90, CW180, CW270, CW0FLIP, CW90FLIP, CW180FLIP, CW270FLIP
imu_spi_bus   1 //spi
pin_imu_cs    29 //spi
pin_imu_int   27 //spi and i2c
imu_i2c_bus   -1 //i2c
imu_i2c_adr   0 //i2c: enter decimal i2c address, not hex (use 0 for default i2c address)

// IMPORTANT: the IMU sensor should be the ONLY sensor on the selected bus


//--- RCL --- Remote Controller Link  (use serial bus -OR- ppm pin)
rcl_gizmo     NONE  // options: NONE, MAVLINK, CRSF, SBUS, DSM, PPM
rcl_num_ch    8     // number of channels
rcl_deadband  0     // center stick deadband
rcl_ser_bus   0
pin_rcl_ppm   -1

//--- BAR --- Barometer
bar_gizmo     HP203B  // options: NONE, BMP390, BMP388, BMP280, MS5611, HP203B
bar_i2c_adr   118 //0x76
bar_i2c_bus   0
bar_rate      100

//--- MAG --- Magnetometer
mag_gizmo     QMC6309  // options: NONE, QMC5883, QMC6309, RM3100
mag_align     CW90   //board edge with pins is forward direction
mag_i2c_adr   124 //0x7C
mag_i2c_bus   0 //sample rate [Hz]

//--- BAT --- Battery Monitor  (use i2c bus -OR- adc pins)
bat_gizmo     INA226  // options: NONE, ADC, INA226, INA228
bat_i2c_adr   64 //0x40
bat_i2c_bus   0
pin_bat_i     -1
pin_bat_v     -1
bat_cal_v     1 //adc voltage scale, value is: actual_voltage_in_v / adc_reading
bat_cal_i     0.0005 //adc current scale, value is: actual_current_in_a / adc_reading; for ina226/228: rshunt value in ohm

//--- GPS ---
gps_gizmo     NONE  // options: NONE, UBLOX
gps_baud      0   // use 0 for auto baud
gps_ser_bus   1

//--- BBX --- Black Box Data Logger  (use spi -OR- mmc)
bbx_gizmo     SDMMC  // options: NONE, SDSPI, SDMMC
pin_bbx_cs    -1  // spi
bbx_spi_bus   -1  // spi
pin_mmc_dat   36  // mmc (uses 4-bit sdio mode on data pins 36,37,38,39)
pin_mmc_clk   34  // mmc
pin_mmc_cmd   35  // mmc

//--- RDR --- Radar (use serial bus -OR- trig+echo pins)
rdr_gizmo     NONE  // options: NONE, LD2411S, LD2413, USD1, SR04
rdr_baud      0
rdr_ser_bus   -1
pin_rdr_trig  -1
pin_rdr_echo  -1

//--- LED ---
led_gizmo     NONE // options: NONE, HIGH_IS_ON, LOW_IS_ON, RGB
pin_led       -1

//--- AHR --- AHRS (keep MAHONY, unless you want to experiment)
ahr_gizmo     MAHONY  // options: MAHONY, MAHONY_BF, MADGWICK, VQF

//--- Serial bus 0 ---
pin_ser0_rx   1
pin_ser0_tx   0

//--- Serial bus 1 ---
pin_ser1_rx   5
pin_ser1_tx   4 

//--- SPI bus 0 ---
pin_spi0_miso -1
pin_spi0_mosi -1
pin_spi0_sclk -1

//--- SPI bus 1 ---
pin_spi1_miso 28
pin_spi1_mosi 31
pin_spi1_sclk 30

//--- I2C Bus 0 ---
pin_i2c0_sda  32
pin_i2c0_scl  33

//--- I2C Bus 1 ---
pin_i2c1_sda  2
pin_i2c1_scl  3

//--- OUT Pins ---
pin_out0      7
pin_out1      9
pin_out2      11
pin_out3      13
pin_out4      15
pin_out5      17
pin_out6      19
pin_out7      21
pin_out8      -1
pin_out9      -1
pin_out10     -1
pin_out11     -1
pin_out12     -1
pin_out13     -1
pin_out14     -1
pin_out15     -1

)""; //end of madflight_board
