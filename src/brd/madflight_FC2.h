/* configuration for madflight FC1 flight controller board

Specifications:

MCU: ESP32-S3 with 4 MB flash and 2MB psram
IMU: ICM-42688-P (spi)
BAR: HP203B (spi)
BAT: INA226 (spi)
MAG: QMC6309 (spi)
BBX: SDCARD (mmc interface)

### Pinout for a Quadcopter

| Row | Pin1 | Pin2 | Pin3 | Pin4 | Pin5 | Pin6 | Pin7 | Pin8 | Pin9 |
|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
|**RowD**| | | | | | |48|47|46|
|**RowC**|GND|+5V|5 OUT7|8|26|38|41 OUT6|GND|+5V|
|**RowB**|GND|+5V|6 RX1 <- GPS TX|9 TX1 -> GPS RX|33 SDA|39 SCL|42 OUT5|GND|+5V|
|**RowA** (board edge)|GND|+5V|7 RX0 <- Receiver TX|21 TX0 -> Receiver TX|34|40|45 OUT4|GND|+5V|

S1-S4 (GPIO1-4) as ESC outputs

*/

#if !ARDUINO_ARCH_ESP32
  #error "Invalid Arduino Architecture: Select ESP32-Arduino"
#endif

#if !CONFIG_IDF_TARGET_ESP32S3
  #error "Invalid board: Select board ESP32S3 Dev Kit"
#endif

#define MF_BOARD_NAME "madflight FC2"
#define MF_MCU_NAME "ESP32-S3"

const char madflight_board[] = R""(

//==========================================//
// external components - modify accordingly //
//==========================================//

//--- RCL --- Remote Controller Link  (use serial bus -OR- ppm pin)
rcl_gizmo      NONE  // options: NONE, MAVLINK, CRSF, SBUS, DSM, PPM
rcl_num_ch     8     // number of channels
rcl_deadband   0     // center stick deadband
rcl_ser_bus    0
pin_rcl_ppm   -1

//--- GPS ---
gps_gizmo      NONE  // options: NONE, UBLOX
gps_baud       0   // use 0 for auto baud
gps_ser_bus    1

//--- RDR --- Radar (use serial bus -OR- trig+echo pins)
rdr_gizmo      NONE  // options: NONE, LD2411S, LD2413, USD1, SR04
rdr_baud       0
rdr_ser_bus   -1
pin_rdr_trig  -1
pin_rdr_echo  -1

//--- AHR --- AHRS (keep MAHONY, unless you want to experiment)
ahr_gizmo      MAHONY  // options: MAHONY, MAHONY_BF, MADGWICK, VQF

//--- Serial bus 0 ---
pin_ser0_rx    7
pin_ser0_tx   21

//--- Serial bus 1 ---
pin_ser1_rx    6
pin_ser1_tx    9 

//--- SPI bus 1 ---
pin_spi1_miso -1
pin_spi1_mosi -1
pin_spi1_sclk -1

//--- I2C Bus 1 ---
pin_i2c1_sda  33
pin_i2c1_scl  39

//--- OUT Pins ---
pin_out0       1 //S1 SMD pad at corner
pin_out1       2 //S2 SMD pad at corner
pin_out2       3 //S3 SMD pad at corner
pin_out3       4 //S4 SMD pad at corner
pin_out4      45 //pin with 5v and gnd next to it
pin_out5      42 //pin with 5v and gnd next to it
pin_out6      41 //pin with 5v and gnd next to it
pin_out7       5 //pin with 5v and gnd next to it
pin_out8      -1
pin_out9      -1
pin_out10     -1
pin_out11     -1
pin_out12     -1
pin_out13     -1
pin_out14     -1
pin_out15     -1

//=====================================//
// internal components - do not modify //
//=====================================//

//--- IMU --- Inertial Measurement Unit  (use spi -OR- i2c bus)
imu_gizmo      ICM42688    // options: NONE, BMI270, MPU6000, MPU6050, MPU6500, MPU9150, MPU9250, ICM45686, ICM42688
imu_bus_type   SPI     // options: SPI, I2C (not all combinations of gizmo and bus_type are supported)
imu_align      CW0     //board edge with pins is forward direction - options: CW0, CW90, CW180, CW270, CW0FLIP, CW90FLIP, CW180FLIP, CW270FLIP
imu_spi_bus     0 //spi
pin_imu_cs     17 //spi
pin_imu_int    13 //spi and i2c
imu_i2c_bus    -1 //i2c
imu_i2c_adr     0 //i2c: enter decimal i2c address, not hex (use 0 for default i2c address)

// IMPORTANT: the IMU sensor should be the ONLY sensor on the selected bus


//--- BAR --- Barometer
bar_gizmo      HP203B  // options: NONE, BMP390, BMP388, BMP280, MS5611, HP203B
bar_i2c_adr    118  //always 118 (0x76) for HP203B
bar_i2c_bus    0

//--- MAG --- Magnetometer
mag_gizmo      QMC6309  // options: NONE, QMC5883, QMC6309, RM3100
mag_align      CW180   //board edge with pins is forward direction - options: CW0, CW90, CW180, CW270, CW0FLIP, CW90FLIP, CW180FLIP, CW270FLIP
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

//--- BBX --- Black Box Data Logger  (use spi -OR- mmc)
bbx_gizmo      SDMMC  // options: NONE, SDSPI, SDMMC
pin_bbx_cs    -1  // spi
bbx_spi_bus   -1  // spi
pin_mmc_dat   37  // mmc
pin_mmc_clk   36  // mmc
pin_mmc_cmd   35  // mmc

//--- LED ---
led_gizmo       RGB // options: NONE, HIGH_IS_ON, LOW_IS_ON, RGB
pin_led         12

//--- SPI bus 0 ---
pin_spi0_miso 14
pin_spi0_mosi 15
pin_spi0_sclk 16

//--- I2C Bus 0 ---
pin_i2c0_sda  11
pin_i2c0_scl  10

)""; //end of madflight_board
