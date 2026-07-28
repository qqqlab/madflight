# Config Parameters
|Parameter|Default|Options|Comment|
|-|-|-|-|
|ahr_gizmo|MAHONY|MAHONY, MAHONY_BF, MADGWICK, VQF|AHRS Algorithm|
|bar_gizmo|NONE|NONE, BMP280, BMP388, BMP390, MS5611, HP203B, BMP580, DPS310|Barometer Sensor Type|
|bar_i2c_adr|0||Barometer I2C Address (decimal number only, set to 0 for automatic)|
|bar_i2c_bus|-1||Barometer I2C Bus|
|bar_rate|100||Barometer sample rate in [Hz]|
|bat_cal_i|1||Battery ADC current scale calibration, value is actual_current_in_a / adc_reading (For INA226 and INA228 this is the Rshunt value in ohm)|
|bat_cal_v|1||Battery ADC voltage scale calibration, value is actual_voltage_in_v / adc_reading|
|bat_gizmo|NONE|NONE, ADC, INA226, INA228|Power Sensor Type|
|bat_i2c_adr|0||Power Sensor I2C Address (decimal number only, set to 0 for automatic)|
|bat_i2c_bus|-1||Power Sensor I2C Bus|
|bbx_baud|0||Black Box Logger Baud Rate (0=auto)|
|bbx_gizmo|NONE|NONE, SDSPI, SDMMC, OPENLOG|Black Box Logger Type|
|bbx_log_ahr|100||Max log interval in [Hz] for AHR|
|bbx_log_imu|100||Max log interval in [Hz] for IMU|
|bbx_log_out|100||Max log interval in [Hz] for OUT|
|bbx_log_rcl|100||Max log interval in [Hz] for RCL|
|bbx_ser_bus|-1||Black Box Logger Serial Bus|
|bbx_spi_bus|-1||Black Box SPI Bus|
|gps_baud|0||Baud Rate (0=auto baud)|
|gps_gizmo|NONE|NONE, UBLOX|GPS Sensor Type|
|gps_ser_bus|-1||GPS Serial Bus|
|imu_acc_lp|70||Accelerometer Low Pass Filter cutoff frequency in [Hz]|
|imu_align|CW0|CW0, CW90, CW180, CW270, CW0FLIP, CW90FLIP, CW180FLIP, CW270FLIP|Gyro/Acc Sensor Alignment|
|imu_bus_type|SPI|SPI, I2C|Gyro/Acc Bus Type|
|imu_cal_ax|0||Accel x zero offset calibration|
|imu_cal_ay|0||Accel y zero offset calibration|
|imu_cal_az|0||Accel z zero offset calibration|
|imu_cal_gx|0||Gyro x zero offset calibration|
|imu_cal_gy|0||Gyro y zero offset calibration|
|imu_cal_gz|0||Gyro z zero offset calibration|
|imu_gizmo|NONE|NONE, BMI270, MPU6000, MPU6050, MPU6500, MPU9150, MPU9250, ICM45686, ICM42688, ICM42688P, AUTO, LSM6DSV, LSM6DSO, LSM6DSV16B|Gyro/Acc Sensor Type|
|imu_gyr_lp|60||Gyro Low Pass Filter cutoff frequency in [Hz] |
|imu_i2c_adr|0||Gyro/Acc Sensor I2C Address (decimal number only, set to 0 for automatic)|
|imu_i2c_bus|-1||Gyro/Acc Sensor I2C Bus|
|imu_rate|1000||Gyro/Acc sample rate in [Hz]. NOTE: not all IMU drivers support all rates|
|imu_spi_bus|-1||Gyro/Acc Sensor SPI Bus|
|led_gizmo|NONE|NONE, HIGH_IS_ON, LOW_IS_ON, RGB|LED Type|
|mag_align|CW0|CW0, CW90, CW180, CW270, CW0FLIP, CW90FLIP, CW180FLIP, CW270FLIP|Magnetometer Alignment|
|mag_cal_sx|1||Magnetometer x scale calibration|
|mag_cal_sy|1||Magnetometer y scale calibration|
|mag_cal_sz|1||Magnetometer z scale calibration|
|mag_cal_x|0||Magnetometer x zero offset calibration|
|mag_cal_y|0||Magnetometer y zero offset calibration|
|mag_cal_z|0||Magnetometer z zero offset calibration|
|mag_gizmo|NONE|NONE, QMC5883, QMC6309, RM3100, QMC5883L, QMC5883P, MMC5603, BMM150|Magnetometer Sensor Type|
|mag_i2c_adr|0||Magnetometer I2C Address (decimal number only, set to 0 for automatic)|
|mag_i2c_bus|-1||Magnetometer I2C Bus|
|mag_lp|1e10||Magnetometer Gyro Low Pass Filter cutoff frequency in [Hz]. Set to 1e10Hz for no filtering|
|ofl_align|NE|NE, NW, ES, EN, SW, SE, WN, WS|Optical Flow XY-Axis Orientation. Example: ES means positive x-axis points East (right) and positive y-axis points South (back)|
|ofl_baud|0||Optical Flow Baud Rate. 0=auto|
|ofl_cal_rad|0||Optical Flow Manual Calibration Factor in [Radians/Pixels], leave at 0 to use gizmo defined calibration|
|ofl_gizmo|NONE|NONE, PMW3901, PMW3901U|Optical Flow Sensor Type|
|ofl_ser_bus|-1||Optical Flow Serial Bus|
|ofl_spi_bus|-1||Optical Flow SPI Bus|
|pin_bat_i|-1||Battery ADC Current Sense Pin|
|pin_bat_v|-1||Battery ADC Voltage Sense Pin|
|pin_bbx_cs|-1||SDCard SPI CS Pin|
|pin_i2c0_scl|-1||I2C0 SCL Pin|
|pin_i2c0_sda|-1||I2C0 SDA Pin|
|pin_i2c1_scl|-1||I2C1 SCL Pin|
|pin_i2c1_sda|-1||I2C1 SDA Pin|
|pin_i2c2_scl|-1||I2C2 SCL Pin|
|pin_i2c2_sda|-1||I2C2 SDA Pin|
|pin_i2c3_scl|-1||I2C3 SCL Pin|
|pin_i2c3_sda|-1||I2C3 SDA Pin|
|pin_imu_clkin|-1||Gyro/Acc CLKIN Pin. Only for ICM-42866-P. Only tested for RP2XXX targets.|
|pin_imu_cs|-1||IMU SPI Chip Select Pin|
|pin_imu_int|-1||IMU Interrupt Pin|
|pin_led|-1||LED Pin|
|pin_mmc_clk|-1||SDCard MMC CLK Pin|
|pin_mmc_cmd|-1||SDCard MMC CMD Pin|
|pin_mmc_dat|-1||SDCard MMC DAT Pin|
|pin_ofl_cs|-1||Optical Flow SPI CS Pin|
|pin_out0|-1||Output0 Pin|
|pin_out1|-1||Output1 Pin|
|pin_out10|-1||Output10 Pin|
|pin_out11|-1||Output11 Pin|
|pin_out12|-1||Output12 Pin|
|pin_out13|-1||Output13 Pin|
|pin_out14|-1||Output14 Pin|
|pin_out15|-1||Output15 Pin|
|pin_out2|-1||Output2 Pin|
|pin_out3|-1||Output3 Pin|
|pin_out4|-1||Output4 Pin|
|pin_out5|-1||Output5 Pin|
|pin_out6|-1||Output6 Pin|
|pin_out7|-1||Output7 Pin|
|pin_out8|-1||Output8 Pin|
|pin_out9|-1||Output9 Pin|
|pin_rcl_ppm|-1||Radio Receiver PPM Pin (need to set rcl_gizmo=PPM)|
|pin_rdr_echo|-1||Radar/Sonar Echo Pin|
|pin_rdr_trig|-1||Radar/Sonar Trigger Pin|
|pin_ser0_inv|-1||Serial0 Inverter Pin|
|pin_ser0_rx|-1||Serial0 RX Pin|
|pin_ser0_tx|-1||Serial0 TX Pin|
|pin_ser1_inv|-1||Serial1 Inverter Pin|
|pin_ser1_rx|-1||Serial1 RX Pin|
|pin_ser1_tx|-1||Serial1 TX Pin|
|pin_ser2_inv|-1||Serial2 Inverter Pin|
|pin_ser2_rx|-1||Serial2 RX Pin|
|pin_ser2_tx|-1||Serial2 TX Pin|
|pin_ser3_inv|-1||Serial3 Inverter Pin|
|pin_ser3_rx|-1||Serial3 RX Pin|
|pin_ser3_tx|-1||Serial3 TX Pin|
|pin_ser4_inv|-1||Serial4 Inverter Pin|
|pin_ser4_rx|-1||Serial4 RX Pin|
|pin_ser4_tx|-1||Serial4 TX Pin|
|pin_ser5_inv|-1||Serial5 Inverter Pin|
|pin_ser5_rx|-1||Serial5 RX Pin|
|pin_ser5_tx|-1||Serial5 TX Pin|
|pin_ser6_inv|-1||Serial6 Inverter Pin|
|pin_ser6_rx|-1||Serial6 RX Pin|
|pin_ser6_tx|-1||Serial6 TX Pin|
|pin_ser7_inv|-1||Serial7 Inverter Pin|
|pin_ser7_rx|-1||Serial7 RX Pin|
|pin_ser7_tx|-1||Serial7 TX Pin|
|pin_spi0_miso|-1||SPI0 MISO Pin|
|pin_spi0_mosi|-1||SPI0 MOSI Pin|
|pin_spi0_sclk|-1||SPI0 SCLK Pin|
|pin_spi1_miso|-1||SPI1 MISO Pin|
|pin_spi1_mosi|-1||SPI1 MOSI Pin|
|pin_spi1_sclk|-1||SPI1 SCLK Pin|
|pin_spi2_miso|-1||SPI2 MISO Pin|
|pin_spi2_mosi|-1||SPI2 MOSI Pin|
|pin_spi2_sclk|-1||SPI2 SCLK Pin|
|pin_spi3_miso|-1||SPI3 MISO Pin|
|pin_spi3_mosi|-1||SPI3 MOSI Pin|
|pin_spi3_sclk|-1||SPI3 SCLK Pin|
|rcl_arm_ch|5||Arm Channel (1-based) - default is AETR,arm,flightmode|
|rcl_arm_max|2500||Maximum PWM for ARMED|
|rcl_arm_min|1600||Minimum PWM for ARMED|
|rcl_baud|0||Radio Receiver Baud Rate. Set to 0 for automatic.|
|rcl_deadband|0||Radio Receiver Stick Deadband|
|rcl_flt_ch|6||Flight Mode Channel (1-based) - default is AETR,arm,flightmode|
|rcl_flt_max|1815||Flight Mode 6-pos Switch Highest PWM (flight mode 5)|
|rcl_flt_min|1165||Flight Mode 6-pos Switch Lowest PWM (flight mode 0)|
|rcl_gizmo|NONE|NONE, MAVLINK, CRSF, SBUS, SBUS_NOT_INV, DSM, PPM, IBUS|Radio Receiver Type|
|rcl_num_ch|8||Radio Receiver Nunber of Channels. Max 20|
|rcl_pit_ch|2||Pitch Channel (1-based) - default is AETR,arm,flightmode|
|rcl_pit_expo|0.0||Pitch Stick Expo: 0.0 (smooth transition from center sensitity to max sensitity) to 1.0 (center sensitivity until approx 50% deflection)|
|rcl_pit_mid|1500||Pitch Stick Centered PWM in [us] 500-2500|
|rcl_pit_pull|1100||Pitch Stick Pulled PWM in [us] 500-2500|
|rcl_pit_push|1900||Pitch Stick Pushed PWM in [us] 500-2500|
|rcl_pit_sens|0.1||Pitch Stick Center Sensitivity: 0.0 (very curvy) to 1.0 (straight line)|
|rcl_rol_ch|1||Roll Channel (1-based) - default is AETR,arm,flightmode|
|rcl_rol_expo|0.0||Roll Stick Expo: 0.0 (smooth transition from center sensitity to max sensitity) to 1.0 (center sensitivity until approx 50% deflection)|
|rcl_rol_left|1100||Roll Stick Left PWM in [us] 500-2500|
|rcl_rol_mid|1500||Roll Stick Center PWM in [us] 500-2500|
|rcl_rol_right|1900||Roll Stick Right PWM in [us] 500-2500|
|rcl_rol_sens|0.1||Roll Stick Center Sensitivity: 0.0 (very curvy) to 1.0 (straight line)|
|rcl_ser_bus|-1||Radio Receiver Serial Bus|
|rcl_thr_ch|3||Throttle Channel (1-based) - default is AETR,arm,flightmode|
|rcl_thr_mid|1500||Throttle Stick Centered PWM in [us] 500-2500|
|rcl_thr_pull|1100||Throttle Stick Pulled PWM in [us] 500-2500|
|rcl_thr_push|1900||Throttle Stick Pushed PWM in [us] 500-2500|
|rcl_vsp_expo|0.0||Vertical Speed Stick Expo: 0.0 (smooth transition from center sensitity to max sensitity) to 1.0 (center sensitivity until approx 50% deflection)|
|rcl_vsp_sens|0.1||Vertical Speed Stick Center Sensitivity: 0.0 (very curvy) to 1.0 (straight line)|
|rcl_yaw_ch|4||Yaw Channel (1-based) - default is AETR,arm,flightmode|
|rcl_yaw_expo|0.0||Yaw Stick Expo: 0.0 (smooth transition from center sensitity to max sensitity) to 1.0 (center sensitivity until approx 50% deflection)|
|rcl_yaw_left|1100||Yaw Stick Left PWM in [us] 500-2500|
|rcl_yaw_mid|1500||Yaw Stick Center PWM in [us] 500-2500|
|rcl_yaw_right|1900||Yaw Stick Right PWM in [us] 500-2500|
|rcl_yaw_sens|0.1||Yaw Stick Center Sensitivity: 0.0 (very curvy) to 1.0 (straight line)|
|rdr_baud|0||Radar/Sonar Baud Rate. Set to 0 for automatic.|
|rdr_gizmo|NONE|NONE, LD2411S, LD2413, USD1, SR04, DTS6012M, VL53L3CX|Radar/Sonar Sensor Type|
|rdr_i2c_adr|0||Optical Flow I2C Address (decimal number only, set to 0 for automatic)|
|rdr_i2c_bus|-1||Optical Flow I2C Bus|
|rdr_ser_bus|-1||Radar/Sonar Serial Bus|
