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
