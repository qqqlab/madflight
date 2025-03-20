#pragma once

#include <stdint.h>

uint16_t tbx_crc16(const uint8_t *buf, uint32_t len, uint16_t crc = 0xFFFF); // 16-bit CCITT CRC
uint32_t tbx_crc32(const uint8_t *buf, uint32_t len, uint32_t crc = 0xFFFFFFFF);
