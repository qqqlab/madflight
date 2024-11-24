/*==========================================================================================
cfg.h - madflight configuration from eeprom/flash

MIT License

Copyright (c) 2023-2024 https://madflight.com

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


//this header needs hw_eeprom_begin(), hw_eeprom_read(int adr), hw_eeprom_write(int adr, uint8_t val), and hw_eeprom_commit()

#pragma once

/*strange.... 

on RP2040:

const char* _cfg_names[]
Sketch uses 158796 bytes (7%) of program storage space. Maximum is 2093056 bytes.
Global variables use 35972 bytes (13%) of dynamic memory, leaving 226172 bytes for local variables. Maximum is 262144 bytes.

static const char* _cfg_names[]
Sketch uses 159004 bytes (7%) of program storage space. Maximum is 2093056 bytes.
Global variables use 35764 bytes (13%) of dynamic memory, leaving 226380 bytes for local variables. Maximum is 262144 bytes.

-->static has +208 flash, -208 ram, 208=52*4 for 53 entries 

on ESP32S3:

const char* _cfg_names[]
Sketch uses 398221 bytes (30%) of program storage space. Maximum is 1310720 bytes.
Global variables use 22000 bytes (6%) of dynamic memory, leaving 305680 bytes for local variables. Maximum is 327680 bytes.

static const char* _cfg_names[]
Sketch uses 398221 bytes (30%) of program storage space. Maximum is 1310720 bytes.
Global variables use 21792 bytes (6%) of dynamic memory, leaving 305888 bytes for local variables. Maximum is 327680 bytes.

-->static has same flash, -208 ram
*/


//list of parameter names
static const char* _cfg_names[] = {
  "IMU_CAL_AX", //accel X zero offset calibration
  "IMU_CAL_AY", //accel Y zero offset calibration
  "IMU_CAL_AZ", //accel Z zero offset calibration
  "IMU_CAL_GX", //gyro X zero offset calibration
  "IMU_CAL_GY", //gyro Y zero offset calibration
  "IMU_CAL_GZ", //gyro Z zero offset calibration

  "MAG_CAL_X",  //magnetometer X zero offset calibration
  "MAG_CAL_Y",  //magnetometer Y zero offset calibration
  "MAG_CAL_Z",  //magnetometer Z zero offset calibration
  "MAG_CAL_SX", //magnetometer X scale calibration
  "MAG_CAL_SY", //magnetometer Y scale calibration
  "MAG_CAL_SZ", //magnetometer Z scale calibration

  "BAT_CAL_V",  //battery ADC voltage scale calibration, value is actual_voltage_in_V / adc_reading
  "BAT_CAL_I",  //battery ADC current scale calibration, value is actual_current_in_A / adc_reading, INA226: Rshunt value in Ohm

  "RCIN_THR_CH",   //1-based channel number
  "RCIN_THR_PULL", //pmw when stick pulled toward you
  "RCIN_THR_MID",  //pmw mid stick
  "RCIN_THR_PUSH", //pwm when stick pushed away from you

  "RCIN_ROL_CH",
  "RCIN_ROL_LEFT", //pmw when stick left
  "RCIN_ROL_MID",
  "RCIN_ROL_RIGHT",//pmw when stick right

  "RCIN_PIT_CH",
  "RCIN_PIT_PULL",
  "RCIN_PIT_MID",
  "RCIN_PIT_PUSH",

  "RCIN_YAW_CH",
  "RCIN_YAW_LEFT",
  "RCIN_YAW_MID",
  "RCIN_YAW_RIGHT",

  "RCIN_ARM_CH",
  "RCIN_ARM_MIN", //armed pwm range min
  "RCIN_ARM_MAX", //armed pwm range max

  "RCIN_FLT_CH",
  "RCIN_FLT_MIN", //6-pos switch lowest pwm (flight mode 0)
  "RCIN_FLT_MAX", //6-pos switch lowest pwm (flight mode 5)
};

#define CFG_LEN_OFFSET 6 //offset in bytes to _len
#define CFG_PARAM_OFFSET 8 //offset to first parameter

class Config {
  //Only add new config values (use only float) at end of this list, when reading an old config without the new value, the new value will be set to the default defined here
  //NEVER rename, delete, insert or move values
  //float can hold 24 bit signed integers, i.e. approx: +/-8,000,000
protected:
  uint8_t _header0 = 'm'; //0x6D
  uint8_t _header1 = 'a'; //0x61
  uint8_t _header2 = 'd'; //0x64
  uint8_t _header3 = 'f'; //0x66
  uint16_t _crc = 0; //crc starting from _len
  uint16_t _len = 0; //sizeof(Config)
public:
  float IMU_CAL_AX     = 0; //accel X zero offset calibration
  float IMU_CAL_AY     = 0; //accel Y zero offset calibration
  float IMU_CAL_AZ     = 0; //accel Z zero offset calibration
  float IMU_CAL_GX     = 0; //gyro X zero offset calibration
  float IMU_CAL_GY     = 0; //gyro Y zero offset calibration
  float IMU_CAL_GZ     = 0; //gyro Z zero offset calibration
  
  float MAG_CAL_X      = 0; //magnetometer X zero offset calibration
  float MAG_CAL_Y      = 0; //magnetometer Y zero offset calibration
  float MAG_CAL_Z      = 0; //magnetometer Z zero offset calibration
  float MAG_CAL_SX     = 1; //magnetometer X scale calibration
  float MAG_CAL_SY     = 1; //magnetometer Y scale calibration
  float MAG_CAL_SZ     = 1; //magnetometer Z scale calibration
  
  float BAT_CAL_V      = 1; //battery ADC voltage scale calibration, value is actual_voltage_in_V / adc_reading
  float BAT_CAL_I      = 1; //battery ADC current scale calibration, value is actual_current_in_A / adc_reading, INA226: Rshunt value in Ohm

  float RCIN_THR_CH    = 1; //1-based channel number
  float RCIN_THR_PULL  = 1100;
  float RCIN_THR_MID   = 1500;
  float RCIN_THR_PUSH  = 1900;

  float RCIN_ROL_CH    = 2;
  float RCIN_ROL_LEFT  = 1100;
  float RCIN_ROL_MID   = 1500;
  float RCIN_ROL_RIGHT = 1900;

  float RCIN_PIT_CH    = 3;
  float RCIN_PIT_PULL  = 1100;
  float RCIN_PIT_MID   = 1500;
  float RCIN_PIT_PUSH  = 1900;

  float RCIN_YAW_CH    = 4;
  float RCIN_YAW_LEFT  = 1100;
  float RCIN_YAW_MID   = 1500;
  float RCIN_YAW_RIGHT = 1900;

  float RCIN_ARM_CH    = 5;
  float RCIN_ARM_MIN   = 1600; //armed pwm range min
  float RCIN_ARM_MAX   = 2500; //armed pwm range max

  //flightmode 6 position switch - Ardupilot switch pwm: 1165,1295,1425,1555,1685,1815 (spacing 130)
  //EdgeTx 3-pos SA + 2-pos SB setup: 
  //     Source:SA Weight:52 Offset:0                 + Source:SB Weight:13 Offset:-1             Multiplex: add 
  //-OR- Source:SA Weight:26 Offset:-40 Switch:SBdown + Source:SA Weight:26 Offset:36 Switch:SBup Multiplex:Replace

  float RCIN_FLT_CH    = 6;
  float RCIN_FLT_MIN   = 1165; //6-pos switch lowest pwm (flight mode 0)
  float RCIN_FLT_MAX   = 1815; //6-pos switch lowest pwm (flight mode 5)
 

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
    *name = _cfg_names[index];
    *value = (&IMU_CAL_AX)[index];
    return true;
  }

private:
  void printValue(int i) {
    const char *name = _cfg_names[i];
    float v = (&IMU_CAL_AX)[i];
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


  //CLI set a parameter value
  void set(String namestr, String val) {
    namestr.toUpperCase();
    const char *name = namestr.c_str();
    float *fvalues = ((float*)&(this->IMU_CAL_AX));
    for(uint16_t i=0;i<valueCount();i++) {
      if(strcmp(_cfg_names[i], name) == 0) {
        fvalues[i] = val.toFloat();
        printValue(i);
        return;
      }
    }
    Serial.printf("ERROR %s not found\n", name);
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
      memcpy(this, &cfg2, sizeof(Config)); //copy sizeof(Config) not actual number of bytes read, this sets missing parameters to default values
      Serial.printf("CFG: Config read. len=%d crc=%04X (matched)\n", (int)cfg2.len(), (int)cfg2.crc());
    }else{
      Serial.printf("CFG: EEPROM Config invalid, using defaults. len=%u crc=%04X crc_expected=%04X\n", (int)cfg2.len(), (int)cfg2.crc(), (int)cfg2.crcCalc());
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



};

Config cfg;
