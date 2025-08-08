/* configuration for madflight FC1 flight controller board

Specifications:

MCU: ESP32-S3 with 4 MB flash and 2MB psram
IMU: ICM-42688-P (spi)
BAR: HP203B (spi)
BAT: INA226 (spi)
MAG: QMC6309 (spi)
BBX: SDCARD (mmc interface)

*/

#define MF_BOARD_NAME "madflight ESP-FC2"
#define MF_MCU_NAME "ESP32-S3"

const char madflight_board[] = R""(

//--- IMU --- Inertial Measurement Unit  (use spi -OR- i2c bus)
imu_gizmo      ICM42688    // options: NONE, BMI270, MPU6000, MPU6050, MPU6500, MPU9150, MPU9250, ICM45686, ICM42688
imu_bus_type   SPI     // options: SPI, I2C (not all combinations of gizmo and bus_type are supported)
imu_align      CW0     //board edge with pins is forward direction - options: CW0, CW90, CW180, CW270, CW0FLIP, CW90FLIP, CW180FLIP, CW270FLIP
imu_spi_bus    0 //spi
pin_imu_cs     17 //spi
pin_imu_int    13 //spi and i2c
imu_i2c_bus    -1 //i2c
imu_i2c_adr     0 //i2c: enter decimal i2c address, not hex (use 0 for default i2c address)

// IMPORTANT: the IMU sensor should be the ONLY sensor on the selected bus


//--- RCL --- Remote Controller Link  (use serial bus -OR- ppm pin)
rcl_gizmo      NONE  // options: NONE, MAVLINK, CRSF, SBUS, DSM, PPM
rcl_num_ch     8     // number of channels
rcl_deadband   0     // center stick deadband
rcl_ser_bus   -1
pin_rcl_ppm   -1

//--- BAR --- Barometer
bar_gizmo      HP203B  // options: NONE, BMP390, BMP388, BMP280, MS5611, HP203B
bar_i2c_adr    118  //always 118 (0x76) for HP203B on ESP-FC2
bar_i2c_bus    0

//--- MAG --- Magnetometer
mag_gizmo      QMC6309  // options: NONE, QMC5883, QMC6309, RM3100
mag_align      CW0   //board edge with pins is forward direction - options: CW0, CW90, CW180, CW270, CW0FLIP, CW90FLIP, CW180FLIP, CW270FLIP
mag_i2c_adr    124   //always 124 (0x7C) for QMC6309
mag_i2c_bus    0

//--- BAT --- Battery Monitor  (use i2c bus -OR- adc pins)
bat_gizmo      INA226  // options: NONE, ADC, INA226, INA228
bat_i2c_adr    64
bat_i2c_bus    0
pin_bat_i     -1
pin_bat_v     -1
bat_cal_v      1 //adc voltage scale, value is: actual_voltage_in_v / adc_reading
bat_cal_i      0.002 //adc current scale, value is: actual_current_in_a / adc_reading; for ina226/228: rshunt value in ohm

//--- GPS ---
gps_gizmo      NONE  // options: NONE, UBLOX
gps_baud       0   // use 0 for auto baud
gps_ser_bus   -1

//--- BBX --- Black Box Data Logger  (use spi -OR- mmc)
bbx_gizmo      SDMMC  // options: NONE, SDSPI, SDMMC
pin_bbx_cs    -1  // spi
bbx_spi_bus   -1  // spi
pin_mmc_dat   37  // mmc
pin_mmc_clk   36  // mmc
pin_mmc_cmd   35  // mmc

//--- RDR --- Radar (use serial bus -OR- trig+echo pins)
rdr_gizmo      NONE  // options: NONE, LD2411S, LD2413, USD1, SR04
rdr_baud       0
rdr_ser_bus   -1
pin_rdr_trig  -1
pin_rdr_echo  -1

//--- LED ---
led_gizmo       RGB // options: NONE, HIGH_IS_ON, LOW_IS_ON, RGB
pin_led         12

//--- AHR --- AHRS (keep MAHONY, unless you want to experiment)
ahr_gizmo      MAHONY  // options: MAHONY, MAHONY_BF, MADGWICK, VQF

//--- Serial bus 0 ---
pin_ser0_rx   -1
pin_ser0_tx   -1

//--- Serial bus 1 ---
pin_ser1_rx   -1
pin_ser1_tx   -1 

//--- SPI bus 0 ---
pin_spi0_miso 14
pin_spi0_mosi 15
pin_spi0_sclk 16

//--- SPI bus 1 ---
pin_spi1_miso -1
pin_spi1_mosi -1
pin_spi1_sclk -1

//--- I2C Bus 0 ---
pin_i2c0_sda  11
pin_i2c0_scl  10

//--- I2C Bus 1 ---
pin_i2c1_sda  -1
pin_i2c1_scl  -1

//--- OUT Pins ---
pin_out0      1
pin_out1      2
pin_out2      3
pin_out3      4
pin_out4      5
pin_out5      6
pin_out6      7
pin_out7      8
pin_out8      9
pin_out9      -1
pin_out10     -1
pin_out11     -1
pin_out12     -1
pin_out13     -1
pin_out14     -1
pin_out15     -1

)""; //end of madflight_board
