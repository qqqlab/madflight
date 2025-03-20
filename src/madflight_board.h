#if defined ARDUINO_ARCH_ESP32

  #ifdef CONFIG_IDF_TARGET_ESP32S3
    #include "madflight_board__ESP32-S3.h"
  #else
    #include "madflight_board__ESP32.h"
  #endif



#elif defined ARDUINO_ARCH_RP2040

  #include "madflight_board__RP2040.h"



#elif defined ARDUINO_ARCH_STM32

  #include "madflight_board__STM32.h"



#else
  #error "HAL: Unknown hardware architecture, expected ESP32 / RP2040 / STM32"
#endif