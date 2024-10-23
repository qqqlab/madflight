/*==========================================================================================
BB: madflight black box data logger

MIT License

Copyright (c) 2024 qqqlab - https://github.com/qqqlab

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
//bb_flash.h
#define BB_USE_INTFLASH 101 //internal QSPI/OSPI flash
#define BB_USE_FLASH 102 //external SPI flash
#define BB_USE_RAM 103 //internal RAM (or PSRAM on ESP32)
//bb_sdcard.h
#define BB_USE_SD 201 //SDCARD with 1-bit SPI interface
#define BB_USE_SDMMC 202 //SDCARD with 1-bit MMC interface (ESP32/ESP32-S3)
#define BB_USE_SDDEBUG 203 //print log to Serial


//=====================================================================================================================
// No Logging
//=====================================================================================================================
#if !defined BB_USE || BB_USE == BB_USE_NONE

//black box public interface
class BlackBox {
  public:
    //loggers
    void log_baro() {}
    void log_bat() {}
    void log_gps() {}
    void log_imu() {}
    void log_mode(uint8_t fm, const char* name) {}
    void log_msg(const char* msg) {}
    void log_parm(const char* name, float value, float default_value) {}
    void log_pid() {}

    //Blackbox Interface
    void setup() {}
    void start() {}
    void stop() {}
    void erase() {}
    void dir() {}
    void bench() {}
    void info() {}
    void csvDump(int fileno) {}
};

BlackBox bb;

//=====================================================================================================================
// Logging to flash (internal or external)
//=====================================================================================================================
#elif BB_USE < BB_USE_SD

#include "bb_flash/bb_flash.h"

//=====================================================================================================================
// Logging to SDCARD (SPI or MMC interface)
//=====================================================================================================================
#else

#include "bb_sdcard/bb_sdcard.h"

#endif
