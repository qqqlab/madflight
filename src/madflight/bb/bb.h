/*==========================================================================================
BB: madflight black box data logger

MIT License

Copyright (c) 2024 https://madflight.com

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

#define BB_USE_NONE    0
#define BB_USE_SD      1 //SDCARD with 1-bit SPI interface
#define BB_USE_SDMMC   2 //SDCARD with 1-bit MMC interface (ESP32/ESP32-S3)
#define BB_USE_SDDEBUG 3 //print log to Serial

#include "bb_interface.h" //defines class BlackBox

#ifndef BB_USE
  #define BB_USE BB_USE_NONE
#endif

//check BB_USE setting is supported and has required pins defined
#if BB_USE == BB_USE_SD
  #if !defined ARDUINO_ARCH_RP2040 && !defined ARDUINO_ARCH_ESP32
    #warning BB_USE_SD not available for this processor
    #undef BB_USE
    #define BB_USE BB_USE_NONE
  #endif
  #if !defined HW_PIN_SPI2_SCLK || !defined HW_PIN_SPI2_MISO || !defined HW_PIN_SPI2_MOSI || !defined HW_PIN_BB_CS
    #warning BB_USE_SD needs HW_PIN_SPI2_SCLK, HW_PIN_SPI2_MISO, HW_PIN_SPI2_MOSI, HW_PIN_BB_CS defined
    #undef BB_USE
    #define BB_USE BB_USE_NONE
  #endif
#elif BB_USE == BB_USE_SDMMC
  #if !defined ARDUINO_ARCH_ESP32
    #warning BB_USE_SDMMC not available for this processor
    #undef BB_USE
    #define BB_USE BB_USE_NONE
  #endif
  #if !defined HW_PIN_SDMMC_CLK || !defined HW_PIN_SDMMC_CMD || !defined HW_PIN_SDMMC_DATA
    #warning BB_USE_SDMMC needs HW_PIN_SDMMC_CLK, HW_PIN_SDMMC_CMD, HW_PIN_SDMMC_DATA defined
    #undef BB_USE
    #define BB_USE BB_USE_NONE
  #endif
#endif

//=====================================================================================================================
// No Logging
//=====================================================================================================================
#if BB_USE == BB_USE_NONE

#include "bb_none/bb_none.h"

//=====================================================================================================================
// Logging to SDCARD (SPI or MMC interface)
//=====================================================================================================================
#else

#include "bb_sdcard/bb_sdcard.h"

#endif
