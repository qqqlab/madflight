/*==========================================================================================
Driver for ICM-45686 gyroscope/accelometer

MIT License

Copyright (c) 2025 https://madflight.com

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

#include <SPI.h>
#include "../MPUxxxx/MPU_interface.h"

class ICM45686 {
public:
  int sampleRateActual = 0;
  volatile float gyr[3] = {};
  volatile float acc[3] = {};
  const float tmp_scale = 1.0 / 128.0;
  const float acc_scale = 32.0 / 524288.0; //Accel scale +/-32g, 20bit, 16384 LSB/g (actual resolution 19bit, i.e. 8192 LSB/g)
  const float gyr_scale = 4000.0 / 524288.0; //Gyro scale +/-4000dps, 20bit

  int begin(SPIClass *spi, int cs_pin, int sample_rate, bool use_clkin); //returns negative error code, positive warning code, or 0 on success
  int read(); //returns number of samples, or -1 on error

  ~ICM45686() {
    delete dev;
  }
private:
  MPU_Interface* dev = nullptr;
  int error_code = -2; //default to "not initalized error"

  //packet to read from address ICM45686_REG_FIFO_COUNTH
  struct __packed ICM45686_FIFOData {
      uint8_t tx_reg;   // transmit buffer starts here, preserves this byte between calls thus saves a couple nanoseconds
      uint8_t rx_start; // receive buffer starts here, first byte is dummy byte returned when sending reg address
      uint16_t n_samples;
      uint8_t header;
      uint8_t acc[6];
      uint8_t gyr[6];
      int16_t temperature;
      uint16_t timestamp;
      uint8_t gx : 4, ax : 4;
      uint8_t gy : 4, ay : 4;
      uint8_t gz : 4, az : 4;
  };

  ICM45686_FIFOData fifobuf;

  bool write_reg(uint16_t reg, uint8_t mask, uint8_t value);
  uint8_t read_bank(uint16_t addr);
  void write_bank(uint16_t addr, uint8_t val);
};
