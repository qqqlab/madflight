#if defined ARDUINO_ARCH_STM32



#if defined(DATA_EEPROM_BASE)
  //----------------------------------------------------------------------------------------------------------
  //unbuffered write - very slow because writes whole flash page for each byte, i.e. 1 second per changed byte
  //----------------------------------------------------------------------------------------------------------
  #include <EEPROM.h>

  void hw_eeprom_begin() {
    Serial.println("EEPROM: Unbuffered IO");
    //EEPROM.begin(); //STM does not use size in begin() call
  }

  uint8_t hw_eeprom_read(uint32_t adr) {   
    uint8_t val = EEPROM.read(adr);
    //Serial.printf("EEPROM.read(%d) = 0x%02X\n", adr, val);
    return val;
  }

  void hw_eeprom_write(uint32_t adr, uint8_t val) {   
    EEPROM.update(adr, val); //update only writes when changed
    //Serial.printf("EEPROM.write(%d, 0x%02X)\n", adr, val);
  }

  void hw_eeprom_commit() {
    //EEPROM.commit();  //STM does not use commit(), write() also executes commit()
  }
#else
  //----------------------------------------------------------------------------------------------------------
  //buffered write - takes approx. 1 second to write full config
  //----------------------------------------------------------------------------------------------------------
  #include <EEPROM.h>

  void hw_eeprom_begin() {
    (void)(EEPROM); //keep compiler happy
    Serial.println("EEPROM: Buffered IO");
    //Serial.println("START reading from flash");Serial.flush();
    eeprom_buffer_fill(); //Copy the data from the flash to the buffer
    //Serial.println("DONE reading");Serial.flush();
  }

  uint8_t hw_eeprom_read(uint32_t adr) {
    uint8_t val = eeprom_buffered_read_byte(adr); //read from buffer
    //Serial.printf("hw_eeprom_read(%d) = 0x%02X\n", adr, val);Serial.flush();
    return val;
  }

  void hw_eeprom_write(uint32_t adr, uint8_t val) {
    //Serial.printf("hw_eeprom_write(%d, 0x%02X)\n", adr, val);Serial.flush();
    eeprom_buffered_write_byte(adr, val); //write to buffer
  }

  void hw_eeprom_commit() {
    //Serial.println("START writing to flash");Serial.flush();
    eeprom_buffer_flush(); //Copy the data from the buffer to the flash
    //Serial.println("DONE writing");Serial.flush();
  } 
#endif

//=====================================================================
#else // #if defined ARDUINO_ARCH_STM32
  //NON STM32
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
//=====================================================================

#include "cfg.h"

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("--------------------");
}

void loop() {
  cfg.begin();
  Serial.printf("crc=%d crcCalc=%d len=%d v=%f i=%f  press a key to increase i by 1.1\n", cfg.crc(), cfg.crcCalc(), cfg.len(), cfg.BAT_CAL_V, cfg.BAT_CAL_I);
  if(Serial.available()) {
    cfg.BAT_CAL_I += 1.1;
    cfg.write();

    while(Serial.available()) Serial.read();
  }
  delay(1000); 
}

