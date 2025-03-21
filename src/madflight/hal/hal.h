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