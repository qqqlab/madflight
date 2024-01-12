#if defined ARDUINO_ARCH_STM32
  #include <EEPROM.h>
  #if defined(DATA_EEPROM_BASE)
    void hw_eeprom_begin() {
      //EEPROM.begin(); //STM does not use size in begin() call
    }

    uint8_t hw_eeprom_read(uint32_t adr) {
      return EEPROM.read(adr);
    }

    void hw_eeprom_write(uint32_t adr, uint8_t val) {
      EEPROM.update(adr, val); //update only writes when changed
    }

    void hw_eeprom_commit() {
      //EEPROM.commit();  //STM does not use commit(), write() also executes commit()
    }
  #else
    void hw_eeprom_begin() {
      Serial.println("START reading from flash");Serial.flush();
      eeprom_buffer_fill(); //Copy the data from the flash to the buffer
      Serial.println("DONE reading");Serial.flush();
    }

    uint8_t hw_eeprom_read(uint32_t adr) {  
      uint8_t val = eeprom_buffered_read_byte(adr); //read from buffer
      //Serial.printf("hw_eeprom_read(%d)=%d\n",adr,val);Serial.flush();
      return val;
    }

    void hw_eeprom_write(uint32_t adr, uint8_t val) {
      //Serial.printf("hw_eeprom_write(%d,%d)\n",adr,val);Serial.flush();
      eeprom_buffered_write_byte(adr, val); //write to buffer
    }

    void hw_eeprom_commit() {
      Serial.println("START writing to flash");Serial.flush();
      eeprom_buffer_flush(); //Copy the data from the buffer to the flash
      Serial.println("DONE writing");Serial.flush();
    }  
  #endif
#else
  #include <EEPROM.h>

  void hw_eeprom_begin() {
    EEPROM.begin(2048);
  }

  uint8_t hw_eeprom_read(uint32_t adr) {
    return EEPROM.read(adr);
  }

  void hw_eeprom_write(uint32_t adr, uint8_t val) {
    EEPROM.write(adr, val);
  }

  void hw_eeprom_commit() {
    EEPROM.commit();
  }
#endif

#include "cfg.h"

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("--------------------");
}

void loop() {
  cfg.begin();
  Serial.printf("crc=%d crcCalc=%d len=%d v=%f i=%f  press a key to increase i by 1.1\n", cfg.crc(), cfg.crcCalc(), cfg.len(), cfg.bat_cal_v, cfg.bat_cal_i);
  if(Serial.available()) {
    cfg.bat_cal_i += 1.1;
    cfg.save();
    while(Serial.available()) Serial.read();
  }
  delay(1000); 
}
