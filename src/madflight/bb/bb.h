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

#define BB_USE_NONE 1
//bb_sdcard.h
#define BB_USE_SD 2 //SDCARD with 1-bit SPI interface
#define BB_USE_SDMMC 3 //SDCARD with 1-bit MMC interface (ESP32/ESP32-S3)
#define BB_USE_SDDEBUG 4 //print log to Serial


#include "../interface.h" //defines class BlackBox

//=====================================================================================================================
// No Logging
//=====================================================================================================================
#if !defined BB_USE || BB_USE == BB_USE_NONE

BlackBox bb_instance;
BlackBox &bb = bb_instance;

//=====================================================================================================================
// Logging to SDCARD (SPI or MMC interface)
//=====================================================================================================================
#else

#include "bb_sdcard/bb_sdcard.h"

#endif
