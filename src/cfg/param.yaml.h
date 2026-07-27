// DO NOT EDIT - GENERATED ON 2026-07-27 15:58:26.533543 BY extras/scripts/param_generator.py

#pragma once

#include <stdint.h>

namespace Cfg {
  const uint16_t param_cnt = 168; //number of parameters

  //enums for madflight library parameters (prefixed with 'mf_' to prevent macro expansion of global #defines like ADC or ICM45686)
  enum class ahr_gizmo_enum : uint32_t { mf_MAHONY,mf_MAHONY_BF,mf_MADGWICK,mf_VQF };
  enum class bar_gizmo_enum : uint32_t { mf_NONE,mf_BMP280,mf_BMP388,mf_BMP390,mf_MS5611,mf_HP203B,mf_BMP580,mf_DPS310 };
  enum class bat_gizmo_enum : uint32_t { mf_NONE,mf_ADC,mf_INA226,mf_INA228 };
  enum class bbx_gizmo_enum : uint32_t { mf_NONE,mf_SDSPI,mf_SDMMC,mf_OPENLOG };
  enum class gps_gizmo_enum : uint32_t { mf_NONE,mf_UBLOX };
  enum class imu_gizmo_enum : uint32_t { mf_NONE,mf_BMI270,mf_MPU6000,mf_MPU6050,mf_MPU6500,mf_MPU9150,mf_MPU9250,mf_ICM45686,mf_ICM42688,mf_ICM42688P,mf_AUTO,mf_LSM6DSV,mf_LSM6DSO,mf_LSM6DSV16B };
  enum class imu_align_enum : uint32_t { mf_CW0,mf_CW90,mf_CW180,mf_CW270,mf_CW0FLIP,mf_CW90FLIP,mf_CW180FLIP,mf_CW270FLIP };
  enum class led_gizmo_enum : uint32_t { mf_NONE,mf_HIGH_IS_ON,mf_LOW_IS_ON,mf_RGB };
  enum class mag_gizmo_enum : uint32_t { mf_NONE,mf_QMC5883,mf_QMC6309,mf_RM3100,mf_QMC5883L,mf_QMC5883P,mf_MMC5603,mf_BMM150 };
  enum class rcl_gizmo_enum : uint32_t { mf_NONE,mf_MAVLINK,mf_CRSF,mf_SBUS,mf_SBUS_NOT_INV,mf_DSM,mf_PPM,mf_IBUS };
  enum class rdr_gizmo_enum : uint32_t { mf_NONE,mf_LD2411S,mf_LD2413,mf_USD1,mf_SR04,mf_DTS6012M,mf_VL53L3CX };
  enum class imu_bus_type_enum : uint32_t { mf_SPI,mf_I2C };
  enum class mag_align_enum : uint32_t { mf_CW0,mf_CW90,mf_CW180,mf_CW270,mf_CW0FLIP,mf_CW90FLIP,mf_CW180FLIP,mf_CW270FLIP };
  enum class ofl_gizmo_enum : uint32_t { mf_NONE,mf_PMW3901,mf_PMW3901U };
  enum class ofl_align_enum : uint32_t { mf_NE,mf_NW,mf_ES,mf_EN,mf_SW,mf_SE,mf_WN,mf_WS };

  //list of parameters
  struct param_list_t {
    const char* name;
    const float defval;
    const char type;
    const char* options;
  };
  const param_list_t param_list[] = {
    { "imu_cal_ax", (float)0, 'f', "" },
    { "imu_cal_ay", (float)0, 'f', "" },
    { "imu_cal_az", (float)0, 'f', "" },
    { "imu_cal_gx", (float)0, 'f', "" },
    { "imu_cal_gy", (float)0, 'f', "" },
    { "imu_cal_gz", (float)0, 'f', "" },
    { "mag_cal_x", (float)0, 'f', "" },
    { "mag_cal_y", (float)0, 'f', "" },
    { "mag_cal_z", (float)0, 'f', "" },
    { "mag_cal_sx", (float)1, 'f', "" },
    { "mag_cal_sy", (float)1, 'f', "" },
    { "mag_cal_sz", (float)1, 'f', "" },
    { "bat_cal_v", (float)1, 'f', "" },
    { "bat_cal_i", (float)1, 'f', "" },
    { "rcl_thr_ch", (float)3, 'i', "" },
    { "rcl_thr_pull", (float)1100, 'i', "" },
    { "rcl_thr_mid", (float)1500, 'i', "" },
    { "rcl_thr_push", (float)1900, 'i', "" },
    { "rcl_rol_ch", (float)1, 'i', "" },
    { "rcl_rol_left", (float)1100, 'i', "" },
    { "rcl_rol_mid", (float)1500, 'i', "" },
    { "rcl_rol_right", (float)1900, 'i', "" },
    { "rcl_pit_ch", (float)2, 'i', "" },
    { "rcl_pit_pull", (float)1100, 'i', "" },
    { "rcl_pit_mid", (float)1500, 'i', "" },
    { "rcl_pit_push", (float)1900, 'i', "" },
    { "rcl_yaw_ch", (float)4, 'i', "" },
    { "rcl_yaw_left", (float)1100, 'i', "" },
    { "rcl_yaw_mid", (float)1500, 'i', "" },
    { "rcl_yaw_right", (float)1900, 'i', "" },
    { "rcl_arm_ch", (float)5, 'i', "" },
    { "rcl_arm_min", (float)1600, 'i', "" },
    { "rcl_arm_max", (float)2500, 'i', "" },
    { "rcl_flt_ch", (float)6, 'i', "" },
    { "rcl_flt_min", (float)1165, 'i', "" },
    { "rcl_flt_max", (float)1815, 'i', "" },
    { "pin_ser0_rx", (float)-1, 'p', "" },
    { "pin_ser1_rx", (float)-1, 'p', "" },
    { "pin_ser2_rx", (float)-1, 'p', "" },
    { "pin_ser3_rx", (float)-1, 'p', "" },
    { "pin_ser4_rx", (float)-1, 'p', "" },
    { "pin_ser5_rx", (float)-1, 'p', "" },
    { "pin_ser6_rx", (float)-1, 'p', "" },
    { "pin_ser7_rx", (float)-1, 'p', "" },
    { "pin_ser0_tx", (float)-1, 'p', "" },
    { "pin_ser1_tx", (float)-1, 'p', "" },
    { "pin_ser2_tx", (float)-1, 'p', "" },
    { "pin_ser3_tx", (float)-1, 'p', "" },
    { "pin_ser4_tx", (float)-1, 'p', "" },
    { "pin_ser5_tx", (float)-1, 'p', "" },
    { "pin_ser6_tx", (float)-1, 'p', "" },
    { "pin_ser7_tx", (float)-1, 'p', "" },
    { "pin_ser0_inv", (float)-1, 'p', "" },
    { "pin_ser1_inv", (float)-1, 'p', "" },
    { "pin_ser2_inv", (float)-1, 'p', "" },
    { "pin_ser3_inv", (float)-1, 'p', "" },
    { "pin_ser4_inv", (float)-1, 'p', "" },
    { "pin_ser5_inv", (float)-1, 'p', "" },
    { "pin_ser6_inv", (float)-1, 'p', "" },
    { "pin_ser7_inv", (float)-1, 'p', "" },
    { "pin_i2c0_sda", (float)-1, 'p', "" },
    { "pin_i2c1_sda", (float)-1, 'p', "" },
    { "pin_i2c2_sda", (float)-1, 'p', "" },
    { "pin_i2c3_sda", (float)-1, 'p', "" },
    { "pin_i2c0_scl", (float)-1, 'p', "" },
    { "pin_i2c1_scl", (float)-1, 'p', "" },
    { "pin_i2c2_scl", (float)-1, 'p', "" },
    { "pin_i2c3_scl", (float)-1, 'p', "" },
    { "pin_spi0_miso", (float)-1, 'p', "" },
    { "pin_spi1_miso", (float)-1, 'p', "" },
    { "pin_spi2_miso", (float)-1, 'p', "" },
    { "pin_spi3_miso", (float)-1, 'p', "" },
    { "pin_spi0_mosi", (float)-1, 'p', "" },
    { "pin_spi1_mosi", (float)-1, 'p', "" },
    { "pin_spi2_mosi", (float)-1, 'p', "" },
    { "pin_spi3_mosi", (float)-1, 'p', "" },
    { "pin_spi0_sclk", (float)-1, 'p', "" },
    { "pin_spi1_sclk", (float)-1, 'p', "" },
    { "pin_spi2_sclk", (float)-1, 'p', "" },
    { "pin_spi3_sclk", (float)-1, 'p', "" },
    { "pin_out0", (float)-1, 'p', "" },
    { "pin_out1", (float)-1, 'p', "" },
    { "pin_out2", (float)-1, 'p', "" },
    { "pin_out3", (float)-1, 'p', "" },
    { "pin_out4", (float)-1, 'p', "" },
    { "pin_out5", (float)-1, 'p', "" },
    { "pin_out6", (float)-1, 'p', "" },
    { "pin_out7", (float)-1, 'p', "" },
    { "pin_out8", (float)-1, 'p', "" },
    { "pin_out9", (float)-1, 'p', "" },
    { "pin_out10", (float)-1, 'p', "" },
    { "pin_out11", (float)-1, 'p', "" },
    { "pin_out12", (float)-1, 'p', "" },
    { "pin_out13", (float)-1, 'p', "" },
    { "pin_out14", (float)-1, 'p', "" },
    { "pin_out15", (float)-1, 'p', "" },
    { "pin_bbx_cs", (float)-1, 'p', "" },
    { "pin_bat_i", (float)-1, 'p', "" },
    { "pin_bat_v", (float)-1, 'p', "" },
    { "pin_imu_cs", (float)-1, 'p', "" },
    { "pin_imu_int", (float)-1, 'p', "" },
    { "pin_led", (float)-1, 'p', "" },
    { "pin_mmc_dat", (float)-1, 'p', "" },
    { "pin_mmc_clk", (float)-1, 'p', "" },
    { "pin_mmc_cmd", (float)-1, 'p', "" },
    { "pin_rcl_ppm", (float)-1, 'p', "" },
    { "ahr_gizmo", (float)Cfg::ahr_gizmo_enum::mf_MAHONY, 'e', "mf_MAHONY,mf_MAHONY_BF,mf_MADGWICK,mf_VQF" },
    { "bar_gizmo", (float)Cfg::bar_gizmo_enum::mf_NONE, 'e', "mf_NONE,mf_BMP280,mf_BMP388,mf_BMP390,mf_MS5611,mf_HP203B,mf_BMP580,mf_DPS310" },
    { "bar_i2c_bus", (float)-1, 'i', "" },
    { "bar_i2c_adr", (float)0, 'i', "" },
    { "bar_rate", (float)100, 'f', "" },
    { "bat_gizmo", (float)Cfg::bat_gizmo_enum::mf_NONE, 'e', "mf_NONE,mf_ADC,mf_INA226,mf_INA228" },
    { "bat_i2c_bus", (float)-1, 'i', "" },
    { "bat_i2c_adr", (float)0, 'i', "" },
    { "bbx_gizmo", (float)Cfg::bbx_gizmo_enum::mf_NONE, 'e', "mf_NONE,mf_SDSPI,mf_SDMMC,mf_OPENLOG" },
    { "bbx_spi_bus", (float)-1, 'i', "" },
    { "gps_gizmo", (float)Cfg::gps_gizmo_enum::mf_NONE, 'e', "mf_NONE,mf_UBLOX" },
    { "gps_ser_bus", (float)-1, 'i', "" },
    { "gps_baud", (float)0, 'i', "" },
    { "imu_gizmo", (float)Cfg::imu_gizmo_enum::mf_NONE, 'e', "mf_NONE,mf_BMI270,mf_MPU6000,mf_MPU6050,mf_MPU6500,mf_MPU9150,mf_MPU9250,mf_ICM45686,mf_ICM42688,mf_ICM42688P,mf_AUTO,mf_LSM6DSV,mf_LSM6DSO,mf_LSM6DSV16B" },
    { "imu_spi_bus", (float)-1, 'i', "" },
    { "imu_i2c_bus", (float)-1, 'i', "" },
    { "imu_i2c_adr", (float)0, 'i', "" },
    { "imu_align", (float)Cfg::imu_align_enum::mf_CW0, 'e', "mf_CW0,mf_CW90,mf_CW180,mf_CW270,mf_CW0FLIP,mf_CW90FLIP,mf_CW180FLIP,mf_CW270FLIP" },
    { "imu_rate", (float)1000, 'f', "" },
    { "imu_acc_lp", (float)70, 'f', "" },
    { "imu_gyr_lp", (float)60, 'f', "" },
    { "led_gizmo", (float)Cfg::led_gizmo_enum::mf_NONE, 'e', "mf_NONE,mf_HIGH_IS_ON,mf_LOW_IS_ON,mf_RGB" },
    { "mag_gizmo", (float)Cfg::mag_gizmo_enum::mf_NONE, 'e', "mf_NONE,mf_QMC5883,mf_QMC6309,mf_RM3100,mf_QMC5883L,mf_QMC5883P,mf_MMC5603,mf_BMM150" },
    { "mag_i2c_bus", (float)-1, 'i', "" },
    { "mag_i2c_adr", (float)0, 'i', "" },
    { "mag_lp", (float)1e10, 'f', "" },
    { "rcl_gizmo", (float)Cfg::rcl_gizmo_enum::mf_NONE, 'e', "mf_NONE,mf_MAVLINK,mf_CRSF,mf_SBUS,mf_SBUS_NOT_INV,mf_DSM,mf_PPM,mf_IBUS" },
    { "rcl_ser_bus", (float)-1, 'i', "" },
    { "rcl_baud", (float)0, 'i', "" },
    { "rcl_num_ch", (float)8, 'i', "" },
    { "rcl_deadband", (float)0, 'i', "" },
    { "rdr_gizmo", (float)Cfg::rdr_gizmo_enum::mf_NONE, 'e', "mf_NONE,mf_LD2411S,mf_LD2413,mf_USD1,mf_SR04,mf_DTS6012M,mf_VL53L3CX" },
    { "rdr_ser_bus", (float)-1, 'i', "" },
    { "rdr_baud", (float)0, 'i', "" },
    { "pin_rdr_trig", (float)-1, 'p', "" },
    { "pin_rdr_echo", (float)-1, 'p', "" },
    { "imu_bus_type", (float)Cfg::imu_bus_type_enum::mf_SPI, 'e', "mf_SPI,mf_I2C" },
    { "mag_align", (float)Cfg::mag_align_enum::mf_CW0, 'e', "mf_CW0,mf_CW90,mf_CW180,mf_CW270,mf_CW0FLIP,mf_CW90FLIP,mf_CW180FLIP,mf_CW270FLIP" },
    { "ofl_gizmo", (float)Cfg::ofl_gizmo_enum::mf_NONE, 'e', "mf_NONE,mf_PMW3901,mf_PMW3901U" },
    { "ofl_spi_bus", (float)-1, 'i', "" },
    { "pin_ofl_cs", (float)-1, 'p', "" },
    { "ofl_ser_bus", (float)-1, 'i', "" },
    { "ofl_baud", (float)0, 'i', "" },
    { "rdr_i2c_bus", (float)-1, 'i', "" },
    { "rdr_i2c_adr", (float)0, 'i', "" },
    { "ofl_align", (float)Cfg::ofl_align_enum::mf_NE, 'e', "mf_NE,mf_NW,mf_ES,mf_EN,mf_SW,mf_SE,mf_WN,mf_WS" },
    { "ofl_cal_rad", (float)0, 'f', "" },
    { "pin_imu_clkin", (float)-1, 'p', "" },
    { "bbx_log_imu", (float)100, 'i', "" },
    { "bbx_log_out", (float)100, 'i', "" },
    { "bbx_log_ahr", (float)100, 'i', "" },
    { "bbx_log_rcl", (float)100, 'i', "" },
    { "bbx_ser_bus", (float)-1, 'i', "" },
    { "bbx_baud", (float)0, 'i', "" },
    { "rcl_rol_sens", (float)0.1, 'f', "" },
    { "rcl_pit_sens", (float)0.1, 'f', "" },
    { "rcl_yaw_sens", (float)0.1, 'f', "" },
    { "rcl_vsp_sens", (float)0.1, 'f', "" },
    { "rcl_rol_expo", (float)0.0, 'f', "" },
    { "rcl_pit_expo", (float)0.0, 'f', "" },
    { "rcl_yaw_expo", (float)0.0, 'f', "" },
    { "rcl_vsp_expo", (float)0.0, 'f', "" },
  }; //const param_list_t param_list[]
}; //namespace Cfg

//all parameters with defaults
struct CfgParam {
  float imu_cal_ax = 0; // accel x zero offset calibration
  float imu_cal_ay = 0; // accel y zero offset calibration
  float imu_cal_az = 0; // accel z zero offset calibration
  float imu_cal_gx = 0; // gyro x zero offset calibration
  float imu_cal_gy = 0; // gyro y zero offset calibration
  float imu_cal_gz = 0; // gyro z zero offset calibration
  float mag_cal_x = 0; // magnetometer x zero offset calibration
  float mag_cal_y = 0; // magnetometer y zero offset calibration
  float mag_cal_z = 0; // magnetometer z zero offset calibration
  float mag_cal_sx = 1; // magnetometer x scale calibration
  float mag_cal_sy = 1; // magnetometer y scale calibration
  float mag_cal_sz = 1; // magnetometer z scale calibration
  float bat_cal_v = 1; // battery adc voltage scale calibration, value is actual_voltage_in_v / adc_reading
  float bat_cal_i = 1; // battery adc current scale calibration, value is actual_current_in_a / adc_reading, ina226: rshunt value in ohm
  int32_t rcl_thr_ch = 3; // 1-based channel number - default is AETR
  int32_t rcl_thr_pull = 1100;
  int32_t rcl_thr_mid = 1500;
  int32_t rcl_thr_push = 1900;
  int32_t rcl_rol_ch = 1;
  int32_t rcl_rol_left = 1100;
  int32_t rcl_rol_mid = 1500;
  int32_t rcl_rol_right = 1900;
  int32_t rcl_pit_ch = 2;
  int32_t rcl_pit_pull = 1100;
  int32_t rcl_pit_mid = 1500;
  int32_t rcl_pit_push = 1900;
  int32_t rcl_yaw_ch = 4;
  int32_t rcl_yaw_left = 1100;
  int32_t rcl_yaw_mid = 1500;
  int32_t rcl_yaw_right = 1900;
  int32_t rcl_arm_ch = 5;
  int32_t rcl_arm_min = 1600; // armed pwm range min
  int32_t rcl_arm_max = 2500; // armed pwm range max
  int32_t rcl_flt_ch = 6;
  int32_t rcl_flt_min = 1165; // 6-pos switch lowest pwm (flight mode 0)
  int32_t rcl_flt_max = 1815; // 6-pos switch lowest pwm (flight mode 5)
  // v2.0.0
  int32_t pin_ser0_rx = -1;
  int32_t pin_ser1_rx = -1;
  int32_t pin_ser2_rx = -1;
  int32_t pin_ser3_rx = -1;
  int32_t pin_ser4_rx = -1;
  int32_t pin_ser5_rx = -1;
  int32_t pin_ser6_rx = -1;
  int32_t pin_ser7_rx = -1;
  int32_t pin_ser0_tx = -1;
  int32_t pin_ser1_tx = -1;
  int32_t pin_ser2_tx = -1;
  int32_t pin_ser3_tx = -1;
  int32_t pin_ser4_tx = -1;
  int32_t pin_ser5_tx = -1;
  int32_t pin_ser6_tx = -1;
  int32_t pin_ser7_tx = -1;
  int32_t pin_ser0_inv = -1;
  int32_t pin_ser1_inv = -1;
  int32_t pin_ser2_inv = -1;
  int32_t pin_ser3_inv = -1;
  int32_t pin_ser4_inv = -1;
  int32_t pin_ser5_inv = -1;
  int32_t pin_ser6_inv = -1;
  int32_t pin_ser7_inv = -1; // I2C pins
  int32_t pin_i2c0_sda = -1;
  int32_t pin_i2c1_sda = -1;
  int32_t pin_i2c2_sda = -1;
  int32_t pin_i2c3_sda = -1;
  int32_t pin_i2c0_scl = -1;
  int32_t pin_i2c1_scl = -1;
  int32_t pin_i2c2_scl = -1;
  int32_t pin_i2c3_scl = -1; // SPI pins
  int32_t pin_spi0_miso = -1;
  int32_t pin_spi1_miso = -1;
  int32_t pin_spi2_miso = -1;
  int32_t pin_spi3_miso = -1;
  int32_t pin_spi0_mosi = -1;
  int32_t pin_spi1_mosi = -1;
  int32_t pin_spi2_mosi = -1;
  int32_t pin_spi3_mosi = -1;
  int32_t pin_spi0_sclk = -1;
  int32_t pin_spi1_sclk = -1;
  int32_t pin_spi2_sclk = -1;
  int32_t pin_spi3_sclk = -1; // OUT pins
  int32_t pin_out0 = -1;
  int32_t pin_out1 = -1;
  int32_t pin_out2 = -1;
  int32_t pin_out3 = -1;
  int32_t pin_out4 = -1;
  int32_t pin_out5 = -1;
  int32_t pin_out6 = -1;
  int32_t pin_out7 = -1;
  int32_t pin_out8 = -1;
  int32_t pin_out9 = -1;
  int32_t pin_out10 = -1;
  int32_t pin_out11 = -1;
  int32_t pin_out12 = -1;
  int32_t pin_out13 = -1;
  int32_t pin_out14 = -1;
  int32_t pin_out15 = -1; // Other pins
  int32_t pin_bbx_cs = -1;
  int32_t pin_bat_i = -1;
  int32_t pin_bat_v = -1;
  int32_t pin_imu_cs = -1;
  int32_t pin_imu_int = -1;
  int32_t pin_led = -1;
  int32_t pin_mmc_dat = -1;
  int32_t pin_mmc_clk = -1;
  int32_t pin_mmc_cmd = -1;
  int32_t pin_rcl_ppm = -1; // AHR - AHRS
  Cfg::ahr_gizmo_enum ahr_gizmo = Cfg::ahr_gizmo_enum::mf_MAHONY;
  Cfg::bar_gizmo_enum bar_gizmo = Cfg::bar_gizmo_enum::mf_NONE;
  int32_t bar_i2c_bus = -1;
  int32_t bar_i2c_adr = 0;
  float bar_rate = 100; // Barometer sample rate in Hz (default 100)
  Cfg::bat_gizmo_enum bat_gizmo = Cfg::bat_gizmo_enum::mf_NONE;
  int32_t bat_i2c_bus = -1;
  int32_t bat_i2c_adr = 0;
  Cfg::bbx_gizmo_enum bbx_gizmo = Cfg::bbx_gizmo_enum::mf_NONE;
  int32_t bbx_spi_bus = -1; // GPS
  Cfg::gps_gizmo_enum gps_gizmo = Cfg::gps_gizmo_enum::mf_NONE;
  int32_t gps_ser_bus = -1;
  int32_t gps_baud = 0;
  Cfg::imu_gizmo_enum imu_gizmo = Cfg::imu_gizmo_enum::mf_NONE;
  int32_t imu_spi_bus = -1;
  int32_t imu_i2c_bus = -1;
  int32_t imu_i2c_adr = 0;
  Cfg::imu_align_enum imu_align = Cfg::imu_align_enum::mf_CW0;
  float imu_rate = 1000; // IMU sample rate in Hz (default 1000) NOTE: not all IMU drivers support a different rate
  float imu_acc_lp = 70; // Accelerometer Low Pass Filter cutoff frequency in Hz
  float imu_gyr_lp = 60; // Gyro Low Pass Filter cutoff frequency in Hz
  Cfg::led_gizmo_enum led_gizmo = Cfg::led_gizmo_enum::mf_NONE;
  Cfg::mag_gizmo_enum mag_gizmo = Cfg::mag_gizmo_enum::mf_NONE;
  int32_t mag_i2c_bus = -1;
  int32_t mag_i2c_adr = 0;
  float mag_lp = 1e10; // Magnetometer Gyro Low Pass Filter cutoff frequency in Hz (default 1e10Hz, i.e. no filtering)
  Cfg::rcl_gizmo_enum rcl_gizmo = Cfg::rcl_gizmo_enum::mf_NONE;
  int32_t rcl_ser_bus = -1;
  int32_t rcl_baud = 0;
  int32_t rcl_num_ch = 8; // max 20
  int32_t rcl_deadband = 0;
  Cfg::rdr_gizmo_enum rdr_gizmo = Cfg::rdr_gizmo_enum::mf_NONE;
  int32_t rdr_ser_bus = -1;
  int32_t rdr_baud = 0;
  // v2.0.1
  int32_t pin_rdr_trig = -1;
  int32_t pin_rdr_echo = -1;
  Cfg::imu_bus_type_enum imu_bus_type = Cfg::imu_bus_type_enum::mf_SPI;
  // v2.1.1
  Cfg::mag_align_enum mag_align = Cfg::mag_align_enum::mf_CW0;
  // v2.1.3
  Cfg::ofl_gizmo_enum ofl_gizmo = Cfg::ofl_gizmo_enum::mf_NONE;
  int32_t ofl_spi_bus = -1;
  int32_t pin_ofl_cs = -1;
  int32_t ofl_ser_bus = -1;
  int32_t ofl_baud = 0;
  int32_t rdr_i2c_bus = -1;
  int32_t rdr_i2c_adr = 0;
  Cfg::ofl_align_enum ofl_align = Cfg::ofl_align_enum::mf_NE; // xy-axis orientation. Example: ES means positive x-axis points East (right) and positive y-axis points South (back)
  // v2.1.4
  float ofl_cal_rad = 0; // manual calibration factor from pixels to radians, leave at 0 to use calibration from gizmo
  // v2.2.0
  int32_t pin_imu_clkin = -1; // CLKIN pin for ICM-42866-P - only tested for RP2 targets
  // v2.3.0
  int32_t bbx_log_imu = 100; // Max log interval in [Hz] for IMU
  int32_t bbx_log_out = 100; // Max log interval in [Hz] for OUT
  int32_t bbx_log_ahr = 100; // Max log interval in [Hz] for AHR
  int32_t bbx_log_rcl = 100; // Max log interval in [Hz] for RCL
  // v2.3.2
  int32_t bbx_ser_bus = -1;
  int32_t bbx_baud = 0;
  // v2.3.3
  float rcl_rol_sens = 0.1; // stick center sensitivity 0.0 (very curvy) - 1.0 (straight line)
  float rcl_pit_sens = 0.1;
  float rcl_yaw_sens = 0.1;
  float rcl_vsp_sens = 0.1;
  float rcl_rol_expo = 0.0; // stick expo 0.0 (smooth transition from center sensitity to max sensitity) - 1.0 (center sensitivity until approx 50% deflection)
  float rcl_pit_expo = 0.0;
  float rcl_yaw_expo = 0.0;
  float rcl_vsp_expo = 0.0;
}; //struct CfgParam

