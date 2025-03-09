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

//cfg.h - madflight configuration from eeprom/flash

//#include "../hal/hal.h" //TODO

#pragma once

//=========================================================================================
// Parameter List
//=========================================================================================
// This list is used to generate the cfg parametes, e.g. cfg.imu_cal_ax
// ONLY add new config values at end of this list, when reading an old config without the new value, the new value will be set to the default defined here
// NEVER rename, delete, insert or move values
// Parameters are stored as float, which can represent 24 bit integers plus sign, i.e. -16,777,216 to +16,777,216 inclusive
//
// MF_PARAM(name, defval, type, options)
//   name:    parameter (and cfg.variable) name
//   defval:  default value
//   type:    f=float, e=enum, p=pin (numeric for ESP32/RP2, pinname e.g. PB3 for STM32)
//   options: comma separated list of options for e(enum)
#define MF_PARAM_LIST() \
  MF_PARAM( imu_cal_ax ,0, 'f', "") /*accel x zero offset calibration*/ \
  MF_PARAM( imu_cal_ay ,0, 'f', "") /*accel y zero offset calibration*/ \
  MF_PARAM( imu_cal_az ,0, 'f', "") /*accel z zero offset calibration*/ \
  MF_PARAM( imu_cal_gx ,0, 'f', "") /*gyro x zero offset calibration*/ \
  MF_PARAM( imu_cal_gy ,0, 'f', "") /*gyro y zero offset calibration*/ \
  MF_PARAM( imu_cal_gz ,0, 'f', "") /*gyro z zero offset calibration*/ \
\
  MF_PARAM( mag_cal_x ,0, 'f', "") /*magnetometer x zero offset calibration*/ \
  MF_PARAM( mag_cal_y ,0, 'f', "") /*magnetometer y zero offset calibration*/ \
  MF_PARAM( mag_cal_z ,0, 'f', "") /*magnetometer z zero offset calibration*/ \
  MF_PARAM( mag_cal_sx ,1, 'f', "") /*magnetometer x scale calibration*/ \
  MF_PARAM( mag_cal_sy ,1, 'f', "") /*magnetometer y scale calibration*/ \
  MF_PARAM( mag_cal_sz ,1, 'f', "") /*magnetometer z scale calibration*/ \
 \
  MF_PARAM( bat_cal_v ,1, 'f', "") /*battery adc voltage scale calibration, value is actual_voltage_in_v / adc_reading*/ \
  MF_PARAM( bat_cal_i ,1, 'f', "") /*battery adc current scale calibration, value is actual_current_in_a / adc_reading, ina226: rshunt value in ohm*/ \
\
  MF_PARAM( rcin_thr_ch ,1, 'i', "") /*1-based channel number*/ \
  MF_PARAM( rcin_thr_pull ,1100, 'i', "") \
  MF_PARAM( rcin_thr_mid ,1500, 'i', "") \
  MF_PARAM( rcin_thr_push ,1900, 'i', "") \
\
  MF_PARAM( rcin_rol_ch ,2, 'i', "") \
  MF_PARAM( rcin_rol_left ,1100, 'i', "") \
  MF_PARAM( rcin_rol_mid ,1500, 'i', "") \
  MF_PARAM( rcin_rol_right,1900, 'i', "") \
\
  MF_PARAM( rcin_pit_ch ,3, 'i', "") \
  MF_PARAM( rcin_pit_pull ,1100, 'i', "") \
  MF_PARAM( rcin_pit_mid ,1500, 'i', "") \
  MF_PARAM( rcin_pit_push ,1900, 'i', "") \
\
  MF_PARAM( rcin_yaw_ch ,4, 'i', "") \
  MF_PARAM( rcin_yaw_left ,1100, 'i', "") \
  MF_PARAM( rcin_yaw_mid ,1500, 'i', "") \
  MF_PARAM( rcin_yaw_right,1900, 'i', "") \
\
  MF_PARAM( rcin_arm_ch ,5, 'i', "") \
  MF_PARAM( rcin_arm_min ,1600, 'i', "") /*armed pwm range min*/ \
  MF_PARAM( rcin_arm_max ,2500, 'i', "") /*armed pwm range max*/ \
\
 /*flightmode 6 position switch - Ardupilot switch pwm: 1165,1295,1425,1555,1685,1815 (spacing 130)*/ \
 /*EdgeTx 3-pos SA + 2-pos SB setup: */ \
 /* Source:SA Weight:52 Offset:0 + Source:SB Weight:13 Offset:-1 Multiplex: add */ \
 /*-OR- Source:SA Weight:26 Offset:-40 Switch:SBdown + Source:SA Weight:26 Offset:36 Switch:SBup Multiplex:Replace*/ \
  MF_PARAM( rcin_flt_ch ,6, 'i', "") \
  MF_PARAM( rcin_flt_min ,1165, 'i', "") /*6-pos switch lowest pwm (flight mode 0)*/ \
  MF_PARAM( rcin_flt_max ,1815, 'i', "") /*6-pos switch lowest pwm (flight mode 5)*/ \
\
  /*========== new parameters v2.0.0 ==============*/ \
\
  /*Serial pins*/ \
  MF_PARAM( pin_ser1_rx, -1, 'p', "") \
  MF_PARAM( pin_ser1_tx, -1, 'p', "") \
  MF_PARAM( pin_ser1_inv, -1, 'p', "") \
  MF_PARAM( pin_ser2_rx, -1, 'p', "") \
  MF_PARAM( pin_ser2_tx, -1, 'p', "") \
  MF_PARAM( pin_ser2_inv, -1, 'p', "") \
  MF_PARAM( pin_ser3_rx, -1, 'p', "") \
  MF_PARAM( pin_ser3_tx, -1, 'p', "") \
  MF_PARAM( pin_ser3_inv, -1, 'p', "") \
  MF_PARAM( pin_ser4_rx, -1, 'p', "") \
  MF_PARAM( pin_ser4_tx, -1, 'p', "") \
  MF_PARAM( pin_ser4_inv, -1, 'p', "") \
  MF_PARAM( pin_ser5_rx, -1, 'p', "") \
  MF_PARAM( pin_ser5_tx, -1, 'p', "") \
  MF_PARAM( pin_ser5_inv, -1, 'p', "") \
  MF_PARAM( pin_ser6_rx, -1, 'p', "") \
  MF_PARAM( pin_ser6_tx, -1, 'p', "") \
  MF_PARAM( pin_ser6_inv, -1, 'p', "") \
  MF_PARAM( pin_ser7_rx, -1, 'p', "") \
  MF_PARAM( pin_ser7_tx, -1, 'p', "") \
  MF_PARAM( pin_ser7_inv, -1, 'p', "") \
  MF_PARAM( pin_ser8_rx, -1, 'p', "") \
  MF_PARAM( pin_ser8_tx, -1, 'p', "") \
  MF_PARAM( pin_ser8_inv, -1, 'p', "") \
\
  /*SPI pins*/ \
  MF_PARAM( pin_spi1_miso, -1, 'p', "") \
  MF_PARAM( pin_spi1_mosi, -1, 'p', "") \
  MF_PARAM( pin_spi1_sclk, -1, 'p', "") \
  MF_PARAM( pin_spi2_miso, -1, 'p', "") \
  MF_PARAM( pin_spi2_mosi, -1, 'p', "") \
  MF_PARAM( pin_spi2_sclk, -1, 'p', "") \
  MF_PARAM( pin_spi3_miso, -1, 'p', "") \
  MF_PARAM( pin_spi3_mosi, -1, 'p', "") \
  MF_PARAM( pin_spi3_sclk, -1, 'p', "") \
  MF_PARAM( pin_spi4_miso, -1, 'p', "") \
  MF_PARAM( pin_spi4_mosi, -1, 'p', "") \
  MF_PARAM( pin_spi4_sclk, -1, 'p', "") \
\
  /*I2C pins*/ \
  MF_PARAM( pin_i2c1_sda, -1, 'p', "") \
  MF_PARAM( pin_i2c1_scl, -1, 'p', "") \
  MF_PARAM( pin_i2c2_sda, -1, 'p', "") \
  MF_PARAM( pin_i2c2_scl, -1, 'p', "") \
  MF_PARAM( pin_i2c3_sda, -1, 'p', "") \
  MF_PARAM( pin_i2c3_scl, -1, 'p', "") \
  MF_PARAM( pin_i2c4_sda, -1, 'p', "") \
  MF_PARAM( pin_i2c4_scl, -1, 'p', "") \
\
  /*OUT pins*/ \
  MF_PARAM( pin_out1, -1, 'p', "") \
  MF_PARAM( pin_out2, -1, 'p', "") \
  MF_PARAM( pin_out3, -1, 'p', "") \
  MF_PARAM( pin_out4, -1, 'p', "") \
  MF_PARAM( pin_out5, -1, 'p', "") \
  MF_PARAM( pin_out6, -1, 'p', "") \
  MF_PARAM( pin_out7, -1, 'p', "") \
  MF_PARAM( pin_out8, -1, 'p', "") \
  MF_PARAM( pin_out9, -1, 'p', "") \
  MF_PARAM( pin_out10, -1, 'p', "") \
  MF_PARAM( pin_out11, -1, 'p', "") \
  MF_PARAM( pin_out12, -1, 'p', "") \
  MF_PARAM( pin_out13, -1, 'p', "") \
  MF_PARAM( pin_out14, -1, 'p', "") \
  MF_PARAM( pin_out15, -1, 'p', "") \
  MF_PARAM( pin_out16, -1, 'p', "") \
\
  /*Other pins*/ \
  MF_PARAM( pin_led, -1, 'p', "") \
  MF_PARAM( pin_imu_cs, -1, 'p', "") \
  MF_PARAM( pin_imu_exti, -1, 'p', "") \
  MF_PARAM( pin_bat_i, -1, 'p', "") \
  MF_PARAM( pin_bat_v, -1, 'p', "") \
  MF_PARAM( pin_bb_cs, -1, 'p', "") \
  MF_PARAM( pin_sdmmc_data, -1, 'p', "") \
  MF_PARAM( pin_sdmmc_clk, -1, 'p', "") \
  MF_PARAM( pin_sdmmc_cmd, -1, 'p', "") \
\
  /*Serial Busses*/ \
  MF_PARAM( bus_ser_rcin, 1, 'i', "") \
  MF_PARAM( bus_ser_gps,  2, 'i', "") \
  MF_PARAM( bus_ser_rdr,  3, 'i', "") \
\
  /*SPI Busses*/ \
  MF_PARAM( bus_spi_imu,  1, 'i', "") \
  MF_PARAM( bus_spi_bb,   2, 'i', "") \
\
  /*I2C Busses*/ \
  MF_PARAM( bus_i2c_baro, 1, 'i', "") \
  MF_PARAM( bus_i2c_mag,  1, 'i', "") \
  MF_PARAM( bus_i2c_bat,  1, 'i', "") \
  MF_PARAM( bus_i2c_imu,  2, 'i', "") \
\
  /*RCIN - Radio Receiver*/ \
  MF_PARAM( rcin_type, 0, 'e', "NONE,MAVLINK,CRSF,SBUS,DSM,PPM,PWM") \
  MF_PARAM( rcin_num_channels, 8, 'i', "") \
  MF_PARAM( rcin_deadband, 0, 'i', "") \
\
  /*IMU - Inertial Measurement Unit (acc/gyro)*/ \
  MF_PARAM( imu_type, 0, 'e', "NONE,BMI270,MPU6000,MPU6050,MPU6500,MPU9150,MPU9250,ICM45686") \
  MF_PARAM( imu_bus_type, 0, 'e', "SPI,I2C") \
  MF_PARAM( imu_i2c_adr, 0x00, 'i', "") \
  MF_PARAM( imu_align, 0, 'e', "CW0,CW90,CW180,CW270,CW0FLIP,CW90FLIP,CW180FLIP,CW270FLIP") \
  MF_PARAM( imu_rate, 1000, 'i', "") /*IMU sample rate in Hz (default 1000) NOTE: not all IMU drivers support a different rate*/ \
  MF_PARAM( imu_acc_lp, 70, 'f', "") /*Accelerometer Low Pass Filter cutoff frequency in Hz */ \
  MF_PARAM( imu_gyr_lp, 60, 'f', "") /*Gyro Low Pass Filter cutoff frequency in Hz */ \
\
  /*GPS*/ \
  MF_PARAM( gps_type, 0, 'e', "NONE,UBLOX,NMEA") \
  MF_PARAM( gps_baud, 115200, 'i', "") \
\
  /*BARO - Barometer*/ \
  MF_PARAM( baro_type, 0, 'e', "NONE,BMP280,BMP388,BMP390,MS5611") \
  MF_PARAM( baro_i2c_adr, 0x00, 'i', "") \
  MF_PARAM( baro_rate, 1000, 'i', "") /*Barometer sample rate in Hz (default 100)*/ \
\
  /*MAG - Magnetometer*/ \
  MF_PARAM( mag_type, 0, 'e', "NONE,QMC5883L,MS5611") \
  MF_PARAM( mag_i2c_adr, 0x00, 'i', "") \
  MF_PARAM( mag_lp, 1e10, 'f', "") /*Magnetometer Gyro Low Pass Filter cutoff frequency in Hz (default 1e10Hz, i.e. no filtering) */ \
\
  /*BAT - Battery Monitor*/ \
  MF_PARAM( bat_type, 0, 'e', "NONE,ADC,INA226,INA228") \
  MF_PARAM( bat_i2c_adr, 0x00, 'i', "") \
\
  /*BB - Black Box Data Logger*/ \
  MF_PARAM( bb_type, 0, 'e', "NONE,SD,SDMMC") \
\
  /*AHRS - Attitude Heading Reference System*/ \
  MF_PARAM( ahrs_type, 0, 'e', "MAHONY, MAHONY_BF, MADGWICK, VQF") \
\
  /*RDR - Radar*/ \
  MF_PARAM( rdr_type, 0, 'e', "NONE,LM2413") \
  MF_PARAM( rdr_serial, -1, 'i', "") \
  MF_PARAM( rdr_baud, -1, 'i', "") \
//end MF_PARAM_LIST()


//list of parameters (generate from MF_PARAM_LIST)
struct cfg_param_list_t {
  const char* name;
  const float defval;
  const char type;
  const char* options;
} _cfg_param_list[] = {
  #define MF_PARAM(name, defval, type, options) {#name, defval, type, options},
  MF_PARAM_LIST()
  #undef MF_PARAM
};

#define CFG_LEN_OFFSET 6 //offset in bytes to _len
#define CFG_PARAM_OFFSET 8 //offset to first parameter

class Config {
protected:
  uint8_t _header0 = 'm'; //0x6D
  uint8_t _header1 = 'a'; //0x61
  uint8_t _header2 = 'd'; //0x64
  uint8_t _header3 = 'f'; //0x66
  uint16_t _crc = 0; //crc starting from _len
  uint16_t _len = 0; //sizeof(Config)

public:
  //define parameters as variables (generate from MF_PARAM_LIST)
  #define MF_PARAM(name, defval, type, options) float name = defval;
  MF_PARAM_LIST()
  #undef MF_PARAM

  Config() {
    _len = sizeof(Config);
  }

  void begin() {
    hw_eeprom_begin();
    read();
  }

  //get number of parameters
  uint16_t valueCount() {
    return (sizeof(Config) - CFG_PARAM_OFFSET) / sizeof(float);
  }

  union value_t {
    float f;
    uint32_t u;
    int32_t i;
  };

  //get parameter name and value for index
  bool getNameAndValue(uint16_t index, String* name, float* value) {
    if(index>=valueCount()) return false;
    *name = _cfg_param_list[index].name;
    *value = (&imu_cal_ax)[index];
    return true;
  }

private:
  void printValue(int i) {
    const char *name = _cfg_param_list[i].name;
    float v = (&imu_cal_ax)[i];
    int vi = (int)v;
    if(v==vi) {
      Serial.printf("set %s %d\n", name, vi);
    }else{
      Serial.printf("set %s %f\n", name, v);
    }
  }

public:
  //CLI print all config values
  void list() {
    for(int i=0;i<valueCount();i++) {
      printValue(i);
    }
  }

  //CLI set a parameter value, returns parameter index
  int set(String namestr, String val) {
    int i = getIndex(namestr);
    if(i >= 0) {
      val.trim();
      float *fvalues = ((float*)&(this->imu_cal_ax));
      switch(_cfg_param_list[i].type) {
        case 'e': //enum
          fvalues[i] = get_enum_index(val.c_str(), _cfg_param_list[i].options);
          break;
        case 'f': //float
          fvalues[i] = val.toFloat();
          break;
        case 'i': //integer
          fvalues[i] = val.toInt();
          break;
        case 'p': //pinnumber/pinname
          fvalues[i] = hw_get_pin_number(val);
          break;
        default:
          //assume 'f'
          fvalues[i] = val.toFloat();
      }
      printValue(i);
    }else{
      Serial.printf("ERROR %s not found\n", namestr.c_str());
    }
    return i;
  }

  //quietly set a parameter value, returns parameter index
  int set(String namestr, float val) {
    int idx = getIndex(namestr);
    if(idx>=0) {
      float *fvalues = ((float*)&(this->imu_cal_ax));
      fvalues[idx] = val;
    }
    return idx;
  }

  //get parameter index for a parameter name
  int getIndex(String namestr) {
    namestr.trim();
    namestr.toLowerCase();
    const char *name = namestr.c_str();
    for(uint16_t i=0;i<valueCount();i++) {
      if(strcmp(_cfg_param_list[i].name, name) == 0) {
        return i;
      }
    }
    return -1;
  }

  //load defaults
  void clear() {
    Config cfg2;
    memcpy(this, &cfg2, sizeof(Config));
  }

  //read parameters from flash
  void read() {
    //create a new config and read "eeprom" data into it
    Config cfg2;
    uint8_t *buf = (uint8_t *)&cfg2;
    uint16_t n = sizeof(Config);
    //Serial.printf("eeprom_read[%d]:", n);
    for(uint16_t i=0; i<n; i++) {
      buf[i] = hw_eeprom_read(i);
      //Serial.printf("%02X ",buf[i]);
    }
    //Serial.println();
    
    //check header & crc
    if(cfg2._header0 == 'm' && cfg2._header1 == 'a' && cfg2._header2 == 'd' && cfg2._header3 == 'f' && cfg2.crc() == cfg2.crcCalc()) {
      uint16_t len = cfg2.len();
      if(len > sizeof(Config)) len = sizeof(Config);
      memcpy(this, &cfg2, len); //copy only valid part of cfg2, this keeps missing parameters to default values
      Serial.printf("CFG:  Config read. len=%d crc=%04X (matched)\n", (int)len, (int)cfg2.crc());
    }else{
      Serial.printf("CFG:  EEPROM Config invalid, using defaults. len=%u crc=%04X crc_expected=%04X\n", (int)cfg2.len(), (int)cfg2.crc(), (int)cfg2.crcCalc());
    }
  }

  //write config to flash
  void write() {
    _len = sizeof(Config);
    _crc = crcCalc();
    uint8_t *buf = (uint8_t *)this;
    uint16_t n = sizeof(Config);
    Serial.printf("eeprom_write[%d]:", n);
    for(int i=0; i<n; i++) {
      hw_eeprom_write(i, buf[i]);
      Serial.printf("%02X ",buf[i]);
    }
    hw_eeprom_commit();
    Serial.println();
  }

  uint16_t len() {
    return _len;
  }

  uint16_t crc() {
    return _crc;
  }

  //returns 0x10000 on fail, 16 bit crc on success
  uint32_t crcCalc() {
    if( _len <= CFG_LEN_OFFSET || _len > sizeof(Config) ) return 0x10000;
    
    uint8_t *Buffer = ((uint8_t*)this) + CFG_LEN_OFFSET; // skip crc
    int Length = _len - CFG_LEN_OFFSET; // skip crc

    // 16-bit CCITT CRC calculation
    uint16_t retVal = 0xFFFF; //initial value
    for (int i = 0; i < Length; i++)
    {
        retVal ^= Buffer[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if ( retVal & (1 << 15) )
            {
                retVal = (retVal << 1) ^ 0x1021;
            }
            else
            {
                retVal <<= 1;
            }
        }
    }
    return retVal;
  }

public:
  void read_from_string(const char *batch) {
    int pos = 0;
    int start = 0;
    while(1) {
      char c = batch[pos];
      if ( c=='\r' || c=='\n' || c==0 ) { //end of line, or end of string
        int len = pos - start;
        char line[len+1];
        strncpy(line, batch+start, len);
        line[len] = 0;
        String cmdline = String(line);
        read_line(cmdline);
        start = pos+1;
      }
      pos++;
    }
  }

private:
  void read_line(String cmdline) {
    //remove comment
    int comment_pos = cmdline.indexOf('#');
    if(comment_pos >= 0) cmdline = cmdline.substring(0,comment_pos);
    //split name/value
    int space_pos = cmdline.indexOf(' ');
    String name = cmdline.substring(0, space_pos);
    String value = cmdline.substring(space_pos, space_pos+1);
  }

private: 
  int get_enum_index(const char* k, const char* values) {
    int klen = strlen(k);
    int len = strlen(values);
    int pos = 0;
    int i = 0;
    while(pos<len) {
      if(strncmp(values+pos,k,klen)==0 && (pos+klen>=len || values[pos+klen] == ',')) {
        return i;
      }
      i++;
      while(pos<len && values[pos]!=',') pos++;
      pos++; //skip comma
    }
    return -1;
  }


};

extern Config cfg;
