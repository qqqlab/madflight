/*==========================================================================================
MIT License

Copyright (c) 2023-2025 https://madflight.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
===========================================================================================*/

#pragma once

//=========================================================================================
// Parameter List
//=========================================================================================
// This list is used to generate the cfg parametes, e.g. cfg.imu_cal_ax
//
// IMPORTANT
//
//  - ONLY add new config values at end of this list, when reading an old config without the new value, the new value will be set to the default defined here
//  - NEVER rename, delete, insert or move values
//  - USE float or to 24 bit integers to stay compatible with MAVLink which always uses float (a float can represent all 24 bit integers plus sign: i.e. -16,777,216 to +16,777,216 inclusive)
//  - ADD 'mf_' prefix to options to prevent macro expansion of common terms like ADC or ICM45686
//
// MF_PARAM(name, defval, type, options)
//   name:    parameter (and cfg.variable) name - max 16 char long
//   defval:  default value
//   datatype: float, int32_t
//   type:    f=float, i=int32_t, e=enum, p=pin (numeric for ESP32/RP2, pinname e.g. PB3 for STM32)
//   options: comma separated list of options for e(enum) - no spaces, max 16 char per option
#define MF_PARAM_LIST \
  MF_PARAM( imu_cal_ax,        0, float, 'f') /*accel x zero offset calibration*/ \
  MF_PARAM( imu_cal_ay,        0, float, 'f') /*accel y zero offset calibration*/ \
  MF_PARAM( imu_cal_az,        0, float, 'f') /*accel z zero offset calibration*/ \
  MF_PARAM( imu_cal_gx,        0, float, 'f') /*gyro x zero offset calibration*/ \
  MF_PARAM( imu_cal_gy,        0, float, 'f') /*gyro y zero offset calibration*/ \
  MF_PARAM( imu_cal_gz,        0, float, 'f') /*gyro z zero offset calibration*/ \
\
  MF_PARAM( mag_cal_x,         0, float, 'f') /*magnetometer x zero offset calibration*/ \
  MF_PARAM( mag_cal_y,         0, float, 'f') /*magnetometer y zero offset calibration*/ \
  MF_PARAM( mag_cal_z,         0, float, 'f') /*magnetometer z zero offset calibration*/ \
  MF_PARAM( mag_cal_sx,        1, float, 'f') /*magnetometer x scale calibration*/ \
  MF_PARAM( mag_cal_sy,        1, float, 'f') /*magnetometer y scale calibration*/ \
  MF_PARAM( mag_cal_sz,        1, float, 'f') /*magnetometer z scale calibration*/ \
 \
  MF_PARAM( bat_cal_v,         1, float, 'f') /*battery adc voltage scale calibration, value is actual_voltage_in_v / adc_reading*/ \
  MF_PARAM( bat_cal_i,         1, float, 'f') /*battery adc current scale calibration, value is actual_current_in_a / adc_reading, ina226: rshunt value in ohm*/ \
\
  MF_PARAM( rcl_thr_ch,        1, int32_t, 'i') /*1-based channel number*/ \
  MF_PARAM( rcl_thr_pull,   1100, int32_t, 'i') \
  MF_PARAM( rcl_thr_mid,    1500, int32_t, 'i') \
  MF_PARAM( rcl_thr_push,   1900, int32_t, 'i') \
\
  MF_PARAM( rcl_rol_ch,        2, int32_t, 'i') \
  MF_PARAM( rcl_rol_left,   1100, int32_t, 'i') \
  MF_PARAM( rcl_rol_mid,    1500, int32_t, 'i') \
  MF_PARAM( rcl_rol_right,  1900, int32_t, 'i') \
\
  MF_PARAM( rcl_pit_ch,        3, int32_t, 'i') \
  MF_PARAM( rcl_pit_pull,   1100, int32_t, 'i') \
  MF_PARAM( rcl_pit_mid,    1500, int32_t, 'i') \
  MF_PARAM( rcl_pit_push,   1900, int32_t, 'i') \
\
  MF_PARAM( rcl_yaw_ch,        4, int32_t, 'i') \
  MF_PARAM( rcl_yaw_left,   1100, int32_t, 'i') \
  MF_PARAM( rcl_yaw_mid,    1500, int32_t, 'i') \
  MF_PARAM( rcl_yaw_right,  1900, int32_t, 'i') \
\
  MF_PARAM( rcl_arm_ch,        5, int32_t, 'i') \
  MF_PARAM( rcl_arm_min,    1600, int32_t, 'i') /*armed pwm range min*/ \
  MF_PARAM( rcl_arm_max,    2500, int32_t, 'i') /*armed pwm range max*/ \
\
 /*flightmode 6 position switch - Ardupilot switch pwm: 1165,1295,1425,1555,1685,1815 (spacing 130)*/ \
 /*EdgeTx 3-pos SA + 2-pos SB setup: */ \
 /* Source:SA Weight:52 Offset:0 + Source:SB Weight:13 Offset:-1 Multiplex: add */ \
 /*-OR- Source:SA Weight:26 Offset:-40 Switch:SBdown + Source:SA Weight:26 Offset:36 Switch:SBup Multiplex:Replace*/ \
  MF_PARAM( rcl_flt_ch,        6, int32_t, 'i') \
  MF_PARAM( rcl_flt_min,    1165, int32_t, 'i') /*6-pos switch lowest pwm (flight mode 0)*/ \
  MF_PARAM( rcl_flt_max,    1815, int32_t, 'i') /*6-pos switch lowest pwm (flight mode 5)*/ \
\
  /*========== new parameters v2.0.0 ==============*/ \
\
  /*Serial pins*/ \
  MF_PARAM( pin_ser0_rx,      -1, int32_t, 'p') \
  MF_PARAM( pin_ser1_rx,      -1, int32_t, 'p') \
  MF_PARAM( pin_ser2_rx,      -1, int32_t, 'p') \
  MF_PARAM( pin_ser3_rx,      -1, int32_t, 'p') \
  MF_PARAM( pin_ser4_rx,      -1, int32_t, 'p') \
  MF_PARAM( pin_ser5_rx,      -1, int32_t, 'p') \
  MF_PARAM( pin_ser6_rx,      -1, int32_t, 'p') \
  MF_PARAM( pin_ser7_rx,      -1, int32_t, 'p') \
\
  MF_PARAM( pin_ser0_tx,      -1, int32_t, 'p') \
  MF_PARAM( pin_ser1_tx,      -1, int32_t, 'p') \
  MF_PARAM( pin_ser2_tx,      -1, int32_t, 'p') \
  MF_PARAM( pin_ser3_tx,      -1, int32_t, 'p') \
  MF_PARAM( pin_ser4_tx,      -1, int32_t, 'p') \
  MF_PARAM( pin_ser5_tx,      -1, int32_t, 'p') \
  MF_PARAM( pin_ser6_tx,      -1, int32_t, 'p') \
  MF_PARAM( pin_ser7_tx,      -1, int32_t, 'p') \
\
  MF_PARAM( pin_ser0_inv,     -1, int32_t, 'p') \
  MF_PARAM( pin_ser1_inv,     -1, int32_t, 'p') \
  MF_PARAM( pin_ser2_inv,     -1, int32_t, 'p') \
  MF_PARAM( pin_ser3_inv,     -1, int32_t, 'p') \
  MF_PARAM( pin_ser4_inv,     -1, int32_t, 'p') \
  MF_PARAM( pin_ser5_inv,     -1, int32_t, 'p') \
  MF_PARAM( pin_ser6_inv,     -1, int32_t, 'p') \
  MF_PARAM( pin_ser7_inv,     -1, int32_t, 'p') \
\
  /*I2C pins*/ \
  MF_PARAM( pin_i2c0_sda,     -1, int32_t, 'p') \
  MF_PARAM( pin_i2c1_sda,     -1, int32_t, 'p') \
  MF_PARAM( pin_i2c2_sda,     -1, int32_t, 'p') \
  MF_PARAM( pin_i2c3_sda,     -1, int32_t, 'p') \
\
  MF_PARAM( pin_i2c0_scl,     -1, int32_t, 'p') \
  MF_PARAM( pin_i2c1_scl,     -1, int32_t, 'p') \
  MF_PARAM( pin_i2c2_scl,     -1, int32_t, 'p') \
  MF_PARAM( pin_i2c3_scl,     -1, int32_t, 'p') \
\
  /*SPI pins*/ \
  MF_PARAM( pin_spi0_miso,    -1, int32_t, 'p') \
  MF_PARAM( pin_spi1_miso,    -1, int32_t, 'p') \
  MF_PARAM( pin_spi2_miso,    -1, int32_t, 'p') \
  MF_PARAM( pin_spi3_miso,    -1, int32_t, 'p') \
\
  MF_PARAM( pin_spi0_mosi,    -1, int32_t, 'p') \
  MF_PARAM( pin_spi1_mosi,    -1, int32_t, 'p') \
  MF_PARAM( pin_spi2_mosi,    -1, int32_t, 'p') \
  MF_PARAM( pin_spi3_mosi,    -1, int32_t, 'p') \
\
  MF_PARAM( pin_spi0_sclk,    -1, int32_t, 'p') \
  MF_PARAM( pin_spi1_sclk,    -1, int32_t, 'p') \
  MF_PARAM( pin_spi2_sclk,    -1, int32_t, 'p') \
  MF_PARAM( pin_spi3_sclk,    -1, int32_t, 'p') \
\
  /*OUT pins*/ \
  MF_PARAM( pin_out0,         -1, int32_t, 'p') \
  MF_PARAM( pin_out1,         -1, int32_t, 'p') \
  MF_PARAM( pin_out2,         -1, int32_t, 'p') \
  MF_PARAM( pin_out3,         -1, int32_t, 'p') \
  MF_PARAM( pin_out4,         -1, int32_t, 'p') \
  MF_PARAM( pin_out5,         -1, int32_t, 'p') \
  MF_PARAM( pin_out6,         -1, int32_t, 'p') \
  MF_PARAM( pin_out7,         -1, int32_t, 'p') \
  MF_PARAM( pin_out8,         -1, int32_t, 'p') \
  MF_PARAM( pin_out9,         -1, int32_t, 'p') \
  MF_PARAM( pin_out10,        -1, int32_t, 'p') \
  MF_PARAM( pin_out11,        -1, int32_t, 'p') \
  MF_PARAM( pin_out12,        -1, int32_t, 'p') \
  MF_PARAM( pin_out13,        -1, int32_t, 'p') \
  MF_PARAM( pin_out14,        -1, int32_t, 'p') \
  MF_PARAM( pin_out15,        -1, int32_t, 'p') \
\
  /*Other pins*/ \
  MF_PARAM( pin_bbx_cs,       -1, int32_t, 'p') \
  MF_PARAM( pin_bat_i,        -1, int32_t, 'p') \
  MF_PARAM( pin_bat_v,        -1, int32_t, 'p') \
  MF_PARAM( pin_imu_cs,       -1, int32_t, 'p') \
  MF_PARAM( pin_imu_int,      -1, int32_t, 'p') \
  MF_PARAM( pin_led,          -1, int32_t, 'p') \
  MF_PARAM( pin_mmc_dat,      -1, int32_t, 'p') \
  MF_PARAM( pin_mmc_clk,      -1, int32_t, 'p') \
  MF_PARAM( pin_mmc_cmd,      -1, int32_t, 'p') \
  MF_PARAM( pin_rcl_ppm,      -1, int32_t, 'p') \
\
  /*AHR - AHRS*/ \
  MF_PARAM( ahr_gizmo,         0, int32_t, 'e', mf_MAHONY,mf_MAHONY_BF,mf_MADGWICK,mf_VQF) \
\
  /*BAR - Barometer*/ \
  MF_PARAM( bar_gizmo,         0, int32_t, 'e', mf_NONE,mf_BMP280,mf_BMP388,mf_BMP390,mf_MS5611,mf_HP203B) \
  MF_PARAM( bar_i2c_bus,      -1, int32_t, 'i') \
  MF_PARAM( bar_i2c_adr,       0, int32_t, 'i') \
  MF_PARAM( bar_rate,       1000, float, 'f') /*Barometer sample rate in Hz (default 100)*/ \
\
  /*BAT - Battery Monitor*/ \
  MF_PARAM( bat_gizmo,         0, int32_t, 'e', mf_NONE,mf_ADC,mf_INA226,mf_INA228) \
  MF_PARAM( bat_i2c_bus,      -1, int32_t, 'i') \
  MF_PARAM( bat_i2c_adr,       0, int32_t, 'i') \
\
  /*BBX - Black Box Data Logger*/ \
  MF_PARAM( bbx_gizmo,         0, int32_t, 'e', mf_NONE,mf_SDSPI,mf_SDMMC) \
  MF_PARAM( bbx_spi_bus,      -1, int32_t, 'i') \
\
  /*GPS*/ \
  MF_PARAM( gps_gizmo,         0, int32_t, 'e', mf_NONE,mf_UBLOX) \
  MF_PARAM( gps_ser_bus,      -1, int32_t, 'i') \
  MF_PARAM( gps_baud,          0, int32_t, 'i') \
\
  /*IMU - Inertial Measurement Unit (acc/gyro)*/ \
  MF_PARAM( imu_gizmo,         0, int32_t, 'e', mf_NONE,mf_BMI270,mf_MPU6000,mf_MPU6050,mf_MPU6500,mf_MPU9150,mf_MPU9250,mf_ICM45686) \
  MF_PARAM( imu_spi_bus,      -1, int32_t, 'i') \
  MF_PARAM( imu_i2c_bus,      -1, int32_t, 'i') \
  MF_PARAM( imu_i2c_adr,       0, int32_t, 'i') \
  MF_PARAM( imu_align,         0, int32_t, 'e', mf_CW0,mf_CW90,mf_CW180,mf_CW270,mf_CW0FLIP,mf_CW90FLIP,mf_CW180FLIP,mf_CW270FLIP) \
  MF_PARAM( imu_rate,       1000, float, 'f') /*IMU sample rate in Hz (default 1000) NOTE: not all IMU drivers support a different rate*/ \
  MF_PARAM( imu_acc_lp,       70, float, 'f') /*Accelerometer Low Pass Filter cutoff frequency in Hz */ \
  MF_PARAM( imu_gyr_lp,       60, float, 'f') /*Gyro Low Pass Filter cutoff frequency in Hz */ \
\
  /*LED*/ \
  MF_PARAM( led_on,            0, int32_t, 'e', mf_LOW_IS_ON,mf_HIGH_IS_ON) \
\
  /*MAG - Magnetometer*/ \
  MF_PARAM( mag_gizmo,         0, int32_t, 'e', mf_NONE,mf_QMC5883,mf_QMC6309) \
  MF_PARAM( mag_i2c_bus,      -1, int32_t, 'i') \
  MF_PARAM( mag_i2c_adr,       0, int32_t, 'i') \
  MF_PARAM( mag_lp,         1e10, float, 'f') /*Magnetometer Gyro Low Pass Filter cutoff frequency in Hz (default 1e10Hz, i.e. no filtering) */ \
\
  /*RCL - Remote Control Link*/ \
  MF_PARAM( rcl_gizmo,         0, int32_t, 'e', mf_NONE,mf_MAVLINK,mf_CRSF,mf_SBUS,mf_SBUS_NOT_INV,mf_DSM,mf_PPM) \
  MF_PARAM( rcl_ser_bus,      -1, int32_t, 'i') \
  MF_PARAM( rcl_baud,          0, int32_t, 'i') \
  MF_PARAM( rcl_num_ch,        8, int32_t, 'i') /*max 20*/ \
  MF_PARAM( rcl_deadband,      0, int32_t, 'i') \
\
  /*RDR - Radar*/ \
  MF_PARAM( rdr_gizmo,         0, int32_t, 'e', mf_NONE,mf_LD2411S,mf_LD2413,mf_USD1) \
  MF_PARAM( rdr_ser_bus,      -1, int32_t, 'i') \
  MF_PARAM( rdr_baud,          0, int32_t, 'i') \
//end MF_PARAM_LIST


#include <Arduino.h> //String

//=========================================================================================
// Generate parameter count, enums, lookup table, and CfgParam member variables from MF_PARAM_LIST
//=========================================================================================
namespace Cfg {
  //count number of parameters, generated from MF_PARAM_LIST
  #define MF_PARAM(name, defval, datatype, type, ...) + 1
    const uint16_t param_cnt = 0 MF_PARAM_LIST ;
  #undef MF_PARAM
  
  //enums for madflight library parameters, generated from MF_PARAM_LIST
  #define MF_PARAM(name, defval, datatype, type, ...) enum class name##_enum { __VA_ARGS__ };
    MF_PARAM_LIST
  #undef MF_PARAM
  
  //list of parameters (generate from MF_PARAM_LIST)
  #define MF_PARAM(name, defval, datatype, type, ...) {#name, defval, type, #__VA_ARGS__},
    struct param_list_t {
      const char* name;
      const float defval;
      const char type;
      const char* options;
    };
    const param_list_t param_list[] = { MF_PARAM_LIST };
  #undef MF_PARAM
};

//struct CfgParam for parameters, generated from MF_PARAM_LIST
#define MF_PARAM(name, defval, datatype, type, ...) datatype name = defval;
struct CfgParam {
  union {
    struct {
      MF_PARAM_LIST
    };
    float param_float[Cfg::param_cnt];
    int32_t param_int32_t[Cfg::param_cnt];
  };
};
#undef MF_PARAM



#define CFG_HDR0 'm'
#define CFG_HDR1 'a'
#define CFG_HDR2 'd'
#define CFG_HDR3 '2'

class CfgClass : public CfgParam {
private:
  //keep CfgHeader 40 bytes long!!!
  struct __attribute__((packed)) CfgHeader {
    uint8_t header0 = CFG_HDR0;
    uint8_t header1 = CFG_HDR1;
    uint8_t header2 = CFG_HDR2;
    uint8_t header3 = CFG_HDR3;
    uint16_t len = 0; //number of bytes for hdr+param+crc
    uint16_t _reserved0;
    uint32_t madflight_param_crc;
    uint8_t _reserved1[28] = {0};
  } hdr;

public:
  CfgClass();
  void begin();
  uint16_t paramCount(); //get number of parameters
  bool getNameAndValue(uint16_t index, String* name, float* value); //get parameter name and value for index
  void list(const char* filter = nullptr); //CLI print all config values
  bool setParam(String namestr, String val); //CLI set a parameter value, returns true on success
  bool setParamMavlink(String namestr, float val); //set a parameter value, returns true on success
  int getIndex(String namestr); //get parameter index for a parameter name
  void clear(); //load defaults from param_list
  void loadFromEeprom(); //read parameters from eeprom/flash
  void loadFromString(const char *batch); //load text unconditional
  bool load_madflight_param(const char *batch); //load text if crc is different, returns true when loaded
  void writeToEeprom(); //write config to flash
  float getValue(String namestr, float default_value);

  //print
  void printParamOption(const int32_t* param);
  bool getOptionString(uint16_t param_idx, int32_t param_val, char out_option[20]);
  void printPins();
  void printModule(const char* module_name);
  void printNameAndValue(uint16_t i, const char* comment = nullptr);
  void printValue(uint16_t i);

private:
  bool load_cmdline(String cmdline);
  int get_enum_index(const char* k, const char* values);
};

extern CfgClass cfg;