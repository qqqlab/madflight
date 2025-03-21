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

// bbx_cpp.h -  madflight black box data logger

// Make sure this file is includes from madflight.h and not from somewhere else
#ifndef MF_ALLOW_INCLUDE_CCP_H
  #error "Only include this file from madflight.h"
#endif
//#pragma once //don't use here, we want to get an error if included twice

#define BBX_USE_NONE    0
#define BBX_USE_SD      1 //SDCARD with 1-bit SPI interface
#define BBX_USE_SDMMC   2 //SDCARD with 1-bit MMC interface (ESP32/ESP32-S3)
#define BBX_USE_SDDEBUG 3 //print log to Serial

#include "bbx.h" //defines class BlackBox

#ifndef BBX_USE
  #define BBX_USE BBX_USE_NONE
#endif

//check BBX_USE setting is supported and has required pins defined
#if BBX_USE == BBX_USE_SD
  #if !defined ARDUINO_ARCH_RP2040 && !defined ARDUINO_ARCH_ESP32
    #warning BBX_USE_SD not available for this processor
    #undef BBX_USE
    #define BBX_USE BBX_USE_NONE
  #endif
  #if !defined HW_PIN_SPI2_SCLK || !defined HW_PIN_SPI2_MISO || !defined HW_PIN_SPI2_MOSI || !defined HW_PIN_BBX_CS
    #warning BBX_USE_SD needs HW_PIN_SPI2_SCLK, HW_PIN_SPI2_MISO, HW_PIN_SPI2_MOSI, HW_PIN_BBX_CS defined
    #undef BBX_USE
    #define BBX_USE BBX_USE_NONE
  #endif
#elif BBX_USE == BBX_USE_SDMMC
  #if !defined ARDUINO_ARCH_ESP32
    #warning BBX_USE_SDMMC not available for this processor
    #undef BBX_USE
    #define BBX_USE BBX_USE_NONE
  #endif
  #if !defined HW_PIN_SDMMC_CLK || !defined HW_PIN_SDMMC_CMD || !defined HW_PIN_SDMMC_DATA
    #warning BBX_USE_SDMMC needs HW_PIN_SDMMC_CLK, HW_PIN_SDMMC_CMD, HW_PIN_SDMMC_DATA defined
    #undef BBX_USE
    #define BBX_USE BBX_USE_NONE
  #endif
#endif

//=====================================================================================================================
// No Logging
//=====================================================================================================================
#if BBX_USE == BBX_USE_NONE

#include "bbx_none/bbx_none.h"

//=====================================================================================================================
// Logging to SDCARD (SPI or MMC interface)
//=====================================================================================================================
#else

#include "bbx_sdcard/bbx_sdcard.h"

#endif
