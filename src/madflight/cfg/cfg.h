//this header needs hw_eeprom_begin(), hw_eeprom_read(int adr), hw_eeprom_write(int adr, uint8_t val), and hw_eeprom_commit()

const String _cfg_names[] = {
  "imu_cal_ax",
  "imu_cal_ay",
  "imu_cal_az",
  "imu_cal_gx",
  "imu_cal_gy",
  "imu_cal_gz",
  
  "mag_cal_x",
  "mag_cal_y",
  "mag_cal_z",
  "mag_cal_sx",
  "mag_cal_sy",
  "mag_cal_sz",
  
  "bat_cal_v",
  "bat_cal_i",
  ""}; //last entry empty

#define CFG_LEN_OFFSET 6 //offset in bytes to _len

class Config {
  //Only add new config values (use only float) at end of this list, when reading an old config without the new value, the new value will be set to the default defined here
  //NEVER rename, delete, insert or move values
  //float can hold 24 bit signed integers, i.e. approx: +/-8,000,000
protected:
  uint8_t _header0 = 'm'; //0x6D
  uint8_t _header1 = 'a'; //0x61
  uint8_t _header2 = 'd'; //0x64
  uint8_t _header3 = 'f'; //0x66
  uint16_t _crc = 0;
  uint16_t _len = 0;
public:
  float imu_cal_ax = 0; //accel X zero offset calibration
  float imu_cal_ay = 0; //accel Y zero offset calibration
  float imu_cal_az = 0; //accel Z zero offset calibration
  float imu_cal_gx = 0; //gyro X zero offset calibration
  float imu_cal_gy = 0; //gyro Y zero offset calibration
  float imu_cal_gz = 0; //gyro Z zero offset calibration
  
  float mag_cal_x = 0; //magnetometer X zero offset calibration
  float mag_cal_y = 0; //magnetometer Y zero offset calibration
  float mag_cal_z = 0; //magnetometer Z zero offset calibration
  float mag_cal_sx = 1; //magnetometer X scale calibration
  float mag_cal_sy = 1; //magnetometer Y scale calibration
  float mag_cal_sz = 1; //magnetometer Z scale calibration
  
  float bat_cal_v = 1; //battery ADC voltage scale calibration, value is actual_voltage_in_V / adc_reading
  float bat_cal_i = 1; //battery ADC current scale calibration, value is actual_current_in_A / adc_reading, INA226: Rshunt value in Ohm

  Config() {
    _len = lenExpected();
  }

  void begin() {
    hw_eeprom_begin();
    read();
  }

  //print all config values
  void list() {
    float *values = ((float*)&(this->imu_cal_ax));
    int i = 0;
    while(_cfg_names[i] != "") {
      Serial.printf("set %s %f\n", _cfg_names[i].c_str(), values[i]);
      i++;
    }
  }

  //set a config value
  void set(String name, String val) {
    name.toLowerCase();
    float *values = ((float*)&(this->imu_cal_ax));
    int i = 0;
    while(_cfg_names[i] != "") {
      if(_cfg_names[i] == name) {
        values[i] = val.toFloat();
        Serial.printf("set %s %f\n", _cfg_names[i].c_str(), values[i]);
        return;
      }
      i++;
    }
    Serial.printf("ERROR %s not found\n", name.c_str());
  }

  //load defaults
  void clear() {
    Config cfg2;
    memcpy(this, &cfg2, lenExpected());
  }

  //read config from flash
  void read() {
    //create a new config and read "eeprom" data into it
    Config cfg2;
    uint8_t *buf = (uint8_t *)&cfg2;
    uint16_t n = lenExpected();
    Serial.printf("eeprom_read[%d]:", n);
    for(uint16_t i=0; i<n; i++) {
      buf[i] = hw_eeprom_read(i);
      Serial.printf("%02X ",buf[i]);
    }
    Serial.println();
    
    //check header & crc
    if(cfg2._header0 == 'm' && cfg2._header1 == 'a' && cfg2._header2 == 'd' && cfg2._header3 == 'f' && cfg2.crc() == cfg2.crcCalc()) {
      memcpy(this, &cfg2, lenExpected());
      Serial.printf("CFG: Config read. len=%d crc=%04X (matched)\n", (int)cfg2.len(), (int)cfg2.crc());
    }else{
      Serial.printf("CFG: EEPROM Config invalid, using defaults. len=%u crc=%04X crc_expected=%04X\n", (int)cfg2.len(), (int)cfg2.crc(), (int)cfg2.crcCalc());
    }
  }

  //write config to flash
  void write() {
    _len = lenExpected();
    _crc = crcCalc();
    uint8_t *buf = (uint8_t *)this;
    uint16_t n = lenExpected();
    Serial.printf("eeprom_write[%d]:", n);
    for(int i=0; i<n; i++) {
      hw_eeprom_write(i, buf[i]);
      Serial.printf("%02X ",buf[i]);
    }
    hw_eeprom_commit();
    Serial.println();
  }

  uint16_t lenExpected() {
    return sizeof(Config);
  }

  uint16_t len() {
    return _len;
  }

  uint16_t crc() {
    return _crc;
  }

  //returns 0x10000 on fail, 16 bit crc on success
  uint32_t crcCalc() {
    if( _len <= CFG_LEN_OFFSET || _len > lenExpected() ) return 0x10000;
    
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
