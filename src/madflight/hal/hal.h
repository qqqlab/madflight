#pragma once

#include "MF_I2C.h"
#include "MF_Serial.h"
#include <SPI.h> //TODO remove this

#if defined ARDUINO_ARCH_ESP32
  #include "ESP32/hal_ESP32.h"
#elif defined ARDUINO_ARCH_RP2040
  #include "RP2040/hal_RP2040.h"
#elif defined ARDUINO_ARCH_STM32
  #include "STM32/hal_STM32.h"
#else 
  #error "HAL: Unknown hardware architecture, madflight runs on ESP32 / RP2040 / STM32"
#endif


void hal_setup();
void hal_eeprom_begin();
uint8_t hal_eeprom_read(uint32_t adr);
void hal_eeprom_write(uint32_t adr, uint8_t val);
void hal_eeprom_commit();
void hal_reboot();
uint32_t hal_get_core_num();
int hal_get_pin_number(String val);
void hal_print_pin_name(int pinnum);
MF_I2C* hal_get_i2c_bus(int bus_id); //get I2C bus for 1-based bus_id
MF_Serial* hal_get_ser_bus(int bus_id); //get Serial bus for 1-based bus_id
SPIClass* hal_get_spi_bus(int bus_id); //get SPI bus for 1-based bus_id