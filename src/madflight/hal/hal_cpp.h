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

// Make sure this file is includes from madflight.h and not from somewhere else
#ifndef MF_ALLOW_INCLUDE_CCP_H
  #error "Only include this file from madflight.h"
#endif
//#pragma once //don't use here, we want to get an error if included twice

#if defined ARDUINO_ARCH_ESP32
  #include "ESP32/hal_ESP32_cpp.h"
#elif defined ARDUINO_ARCH_RP2040
  #include "RP2040/hal_RP2040_cpp.h"
#elif defined ARDUINO_ARCH_STM32
  #include "STM32/hal_STM32_cpp.h"
#else 
  #error "HAL: Unknown hardware architecture, expected ESP32 / RP2040 / STM32"
#endif


MF_I2C* hal_get_i2c_bus(int bus_id) { 
  if(bus_id < 0 || bus_id >= HAL_I2C_NUM) return nullptr;
  MF_I2C *i2c_bus = hal_i2c[bus_id];
  if(!i2c_bus) return nullptr;
  return i2c_bus;
}

MF_Serial* hal_get_ser_bus(int bus_id) {
  if(bus_id < 0 || bus_id >= HAL_SER_NUM) return nullptr;
  MF_Serial *ser_bus = hal_ser[bus_id];
  if(!ser_bus) return nullptr;
  return ser_bus;
}

SPIClass* hal_get_spi_bus(int bus_id) {
  if(bus_id < 0 || bus_id >= HAL_SPI_NUM) return nullptr;
  SPIClass *spi_bus = hal_spi[bus_id];
  if(!spi_bus) return nullptr;
  return spi_bus;
}