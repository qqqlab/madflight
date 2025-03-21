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

#include "tbx_crc.h"

// 16-bit CCITT CRC calculation
uint16_t tbx_crc16(const uint8_t *buf, uint32_t len, uint16_t crc) { //initial value = 0xFFFF
  for (uint32_t i = 0; i < len; i++) {
    crc ^= buf[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      if ( crc & (1 << 15) ) {
          crc = (crc << 1) ^ 0x1021;
      }else{
          crc <<= 1;
      }
    }
  }
  return crc;
}

uint32_t tbx_crc32(const uint8_t *buf, uint32_t len, uint32_t crc) { //initial value = 0xFFFFFFFF
   for(uint32_t i=0; i<len; i++) {
      uint32_t byte = buf[i];
      crc = crc ^ byte;
      for (int j = 7; j >= 0; j--) {
         uint32_t mask = -(crc & 1);
         crc = (crc >> 1) ^ (0xEDB88320 & mask);
      }
      i = i + 1;
   }
   return ~crc;
}
