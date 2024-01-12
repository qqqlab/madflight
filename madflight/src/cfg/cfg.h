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


class Config {
  //Only add new config values (use only float) at end of this list, when reading an old config without the new value, the new value will be set to the default defined here
  //NEVER rename, delete, insert or move values
  //float can hold 24 bit signed integers, i.e. approx: +/-8,000,000
private:
  uint16_t _crc = 0;
  uint16_t _len = 0;
public:
  float imu_cal_ax = 0;
  float imu_cal_ay = 0;
  float imu_cal_az = 0;
  float imu_cal_gx = 0;
  float imu_cal_gy = 0;
  float imu_cal_gz = 0;
  
  float mag_cal_x = 0;
  float mag_cal_y = 0;
  float mag_cal_z = 0;
  float mag_cal_sx = 1;
  float mag_cal_sy = 1;
  float mag_cal_sz = 1;
  
  float bat_cal_v = 1;
  float bat_cal_i = 1;

  void print() {
    float *values = ((float*)this) + 1; //move past crc+len
    int i = 0;
    while(_cfg_names[i] != "") {
      Serial.printf("set %s %f\n", _cfg_names[i].c_str(), values[i]);
      i++;
    }
  }

  void set(String name, String val) {
    name.toLowerCase();
    float *values = ((float*)this) + 1; //move past crc+len
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

  Config() {
    _len = lenExpected();
  }

  void begin() {
    hw_eeprom_begin();
    eeprom_read_and_check_config();
  }

  void save() {
    _len = lenExpected();
    _crc = crcCalc();
    uint8_t *data = (uint8_t *)this;
    for(int i=0; i<_len; i++) {
      hw_eeprom_write(i, data[i]);
    }
    hw_eeprom_commit();
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

  uint16_t crcCalc() {
    uint8_t *Buffer = ((uint8_t*)this) + 2; //skip crc
    uint16_t Length = _len - 2; //skip crc

    // 16-bit CCITT CRC calculation
    uint16_t retVal = 0xFFFF; //initial value
    for (uint16_t i = 0; i < Length; i++)
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

private:

  void eeprom_read_and_check_config() {
    //create a dummy config to read eeprom data into
    Config cfg_test;
    eeprom_read_conf(&cfg_test);

    //check len is expected size or smaller, then check crc
    if(cfg_test.len() <= lenExpected() && cfg_test.crc() == cfg_test.crcCalc()) {
      eeprom_read_conf(this);
      //recalc len and crc so that len() and crc() return correct values
      _len = lenExpected();
      _crc = crcCalc();
    }
  }

  //read to config
  void eeprom_read_conf(Config *conf) {
    eeprom_read((uint8_t *)conf, conf->len());
  }

  //read to buffer
  void eeprom_read(uint8_t *buf, uint16_t size) {
    for(uint16_t i=0;i<size;i++) {
      buf[i] = hw_eeprom_read(i);
    }  
  }

};

Config cfg;
