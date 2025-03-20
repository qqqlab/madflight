#ifndef MF_ALLOW_INCLUDE_CCP_H
  #error "Only include this file from madflight.h"
#endif

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