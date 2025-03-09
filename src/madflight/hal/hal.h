//include hardware specific code & default board pinout
#if defined ARDUINO_ARCH_ESP32
  #include "ESP32/hal_ESP32.h"
#elif defined ARDUINO_ARCH_RP2040
  #include "RP2040/hal_RP2040.h"
#elif defined ARDUINO_ARCH_STM32
  #include "STM32/hal_STM32.h"
#else 
  #error "HAL: Unknown hardware architecture, expected ESP32 / RP2040 / STM32"
#endif