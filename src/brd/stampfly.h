/* configuration for M5Stamp Fly v1.0 / v1.1 quadcopter

Specifications and Pinout:

MCU: ESP32-S3 with 8 MB flash and 0 MB psram
IMU: BMI270
BAR: BMP280
BAT: INA3221AIRGVR (not implemented)
MAG: BMM150
BBX: none
LED: 2x WS2812 (RGB G39)
RDR: 2x VL53L3C (not implemented, needs update for enable+shutdown pins: INT_GI G6, INT_XSHUT G7, EXT_GI G8, EXT_XSHUT G9)
OFL: PMW3901MB-TXQT (not implemented, needs update for shared SPI bus with IMU: CS G12)
Motors: 4x 716-17600kv (G41, G42, G10, G5)
Buzzer: (not implemented, BEEP G40)
Button: (not implented, USER_A G0)

Setup:

Connect CRSF receiver to the RED Grove port: GND(black) - 5V(red) - ReceiverTX(white) - ReceiverRX(yellow)
Connect GPS/Openlog to the BLACK Grove port: GND(black) - 5V(red) - OpenlogTX(white)  - OpenlogRX(yellow)

*/

#if !ARDUINO_ARCH_ESP32
  #error "Invalid Arduino Architecture: Select ESP32-Arduino"
#elif !CONFIG_IDF_TARGET_ESP32S3
  #error "Invalid board: Select ESP32-S3 with 8MB flash"
#endif

#define MF_BOARD_NAME "StampFly"
#define MF_MCU_NAME "ESP32-S3"

const char madflight_board[] = R""(

//--- IMU --- Inertial Measurement Unit  (use spi -OR- i2c bus)
imu_gizmo      BMI270  // options: NONE, BMI270, ICM42688, ICM45686, MPU6000, MPU6050, MPU6500, MPU9150, MPU9250
imu_bus_type   SPI     // options: SPI, I2C (not all combinations of gizmo and bus_type are supported)
imu_align      CW90 // CW90FLIP     // options: CW0, CW90, CW180, CW270, CW0FLIP, CW90FLIP, CW180FLIP, CW270FLIP
imu_spi_bus    0       // spi
pin_imu_cs     46      // spi
pin_imu_int    11      // spi and i2c
imu_i2c_bus    -1      // i2c
imu_i2c_adr    0       // i2c: enter decimal i2c address, not hex (use 0 for default i2c address)

//--- RCL --- Remote Controller Link  (use serial bus -OR- ppm pin)
rcl_gizmo      NONE  // options: NONE, MAVLINK, CRSF, SBUS, DSM, PPM
rcl_ser_bus   -1     // serial
pin_rcl_ppm   -1     // ppm
rcl_num_ch     8     // serial and ppm: number of channels
rcl_deadband   0     // serial and ppm: center stick deadband

//--- BAR --- Barometer
bar_gizmo      BMP280  // options: NONE, BMP280, BMP388, BMP390, BMP580, HP203B, MS5611
bar_i2c_adr    0x76
bar_i2c_bus    0

//--- MAG --- Magnetometer
mag_gizmo      BMM150 // options: NONE, QMC6309, QMC5883L, QMC5883P, RM3100, BMM150
mag_align      CW0    // options: CW0, CW90, CW180, CW270, CW0FLIP, CW90FLIP, CW180FLIP, CW270FLIP
mag_i2c_adr    0x10
mag_i2c_bus    0

//--- BAT --- Battery Monitor  (use i2c bus -OR- adc pins)
bat_gizmo      NONE  // options: NONE, ADC, INA226, INA228
bat_i2c_adr    0     // i2c
bat_i2c_bus   -1     // i2c
pin_bat_i     -1     // adc
pin_bat_v     -1     // adc
bat_cal_v      1     // for ADC: voltage scale, value is: actual_voltage_in_v / adc_reading; for INA226/228: not used
bat_cal_i      1     // for ADC: current scale, value is: actual_current_in_a / adc_reading; for INA226/228: rshunt value in ohm

//--- GPS ---
gps_gizmo      NONE  // options: NONE, UBLOX
gps_baud       0     // use 0 for auto baud
gps_ser_bus   -1

//--- BBX --- Black Box Data Logger  (use spi -OR- mmc)
bbx_gizmo      NONE  // options: NONE, SDSPI, SDMMC, OPENLOG
pin_bbx_cs    -1     // spi
bbx_spi_bus   -1     // spi
pin_mmc_dat   -1     // mmc
pin_mmc_clk   -1     // mmc
pin_mmc_cmd   -1     // mmc
bbx_ser_bus   -1     // openlog
bbx_baud       0     // openlog, use 0 for default 115200 baud

//--- RDR --- Radar (use serial bus -OR- trig+echo pins)
rdr_gizmo      NONE  // options: NONE, DTS6012M, LD2411S, LD2413, SR04, USD1
rdr_ser_bus   -1     // serial
rdr_baud       0     // serial, use 0 for default baud
pin_rdr_trig  -1     // trig+echo
pin_rdr_echo  -1     // trig+echo

//--- OFL --- Optical FLow (use serial bus -OR- spi bus)
ofl_gizmo      NONE //XXX TODO PMW3901  // options: NONE, PMW3901, PMW3901U
ofl_ser_bus    -1       // serial
ofl_baud       0        // serial, use 0 for default baud
ofl_spi_bus    -1   //XXX TODO 0       // spi
pin_ofl_cs     -1   //XXX TODO 12       // spi

//--- LED ---
led_gizmo     RGB   // options: NONE, HIGH_IS_ON, LOW_IS_ON, RGB
pin_led       39

//--- AHR --- AHRS (keep MAHONY, unless you want to experiment)
ahr_gizmo     MAHONY // options: MAHONY, MAHONY_BF, MADGWICK, VQF

//--- Serial bus 0 ---
pin_ser0_rx   13 // RED Grove "SDA" with 4.7k pullup - connector pin 3 - white
pin_ser0_tx   15 // RED Grove "SCL" with 4.7k pullup - connector pin 4 - yellow

//--- Serial bus 1 ---
pin_ser1_rx   2 // BLACK Grove "O" - connector pin 3 - white
pin_ser1_tx   1 // BLACK Grove "I" - connector pin 4 - yellow

//--- SPI bus 0 ---
pin_spi0_miso 43
pin_spi0_mosi 14
pin_spi0_sclk 44

//--- SPI bus 1 ---
pin_spi1_miso -1
pin_spi1_mosi -1
pin_spi1_sclk -1

//--- I2C Bus 0 ---
pin_i2c0_sda  3
pin_i2c0_scl  4

//--- I2C Bus 1 ---
pin_i2c1_sda  -1
pin_i2c1_scl  -1

//--- OUT Pins ---
pin_out0      41 // R-Down (Right Rear Motor)
pin_out1      42 // L-Up (Right Front Motor)
pin_out2      10 // L-Down (Left Rear Motor)
pin_out3      5  // L-Up (Left Front Motor)
pin_out4      -1
pin_out5      -1
pin_out6      -1
pin_out7      -1
pin_out8      -1
pin_out9      -1
pin_out10     -1
pin_out11     -1
pin_out12     -1
pin_out13     -1
pin_out14     -1
pin_out15     -1

)""; // End of madflight_board
