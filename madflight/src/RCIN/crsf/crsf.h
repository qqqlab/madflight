/*==========================================================================================
crsf.h - Hardware platform agnostic CRSF / ELRS receiver library

MIT License

Copyright (c) 2023 https://github.com/qqqlab

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
===========================================================================================

CRSF/ELRS receiver uses two wire full duplex uart connection.

420000 baud
not inverted
8 Bit
1 Stop bit
Big endian

Frame structure: <Device address><length><Type><Payload><CRC>

Device address: uint8_t
length:         uint8_t length of <Type><Payload><CRC>
Type:           uint8_t
Payload:        uint8_t[len-1])
CRC:            uint8_t

Max frame size is 64 bytes

Sample RC data received from an ELSR receiver:
da le ty  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 15 17 18 19 20 21 22 crc
C8 18 16 E0 03 1F 2B C0 F7 0B E2 B0 02 7C E0 63 1F FB D8 07 00 00 4C 7C E2 77 --> 0x18:len 24 bytes  (frame len is 26 bytes), 0x16=type RC data

Sample link stats received from an ELSR receiver:
da le ty  1  2  3  4  5  6  7  8  9 10 crc
C8 0C 14 33 00 64 0D 00 04 01 00 00 00 96 --> 0x0c:len 12 bytes (frame len is 14 byte), 0x14=type link statistics
==========================================================================================*/

#pragma once

#define CRSF_BAUD 420000
#define CRSF_FRAME_SIZE_MAX 64 //max number of bytes of a frame
#define CRSF_FRAME_LEN_MAX (CRSF_FRAME_SIZE_MAX-2) //max value of the <length> field
#define CRSF_BUFPOS_LEN 1 //buffer position frame length byte
#define CRSF_BUFPOS_TYPE 2 //buffer position frame type byte
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8 //Flight Controller

#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_PAYLOADLEN_RC_CHANNELS_PACKED 22  //length of the RC data packed bytes frame. 16 channels in 11 bits each.


class CRSF {
public:
    uint32_t channel_ts; //last channel data received millisecond timestamp
    uint16_t channel[16]; //channel data. values: 988-2012
    uint32_t timeout; //lost connection timeout in milliseconds (default 2000)

    CRSF() {
        buf_i = 0;
        timeout = 2000;
        channel_ts = millis() - timeout;
        for(int i=0;i<16;i++) channel[i] = 1500;
    }

    bool is_connected() {
        return millis() - channel_ts < timeout;
    }

    bool update(uint8_t c) {
        bool received_channels = false;
        if (buf_i == 0) { //device address
            if (c == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
                buf[buf_i++] = c;
            }
        }else if (buf_i == CRSF_BUFPOS_LEN) { //frame length
            if (c>=2 && c<=CRSF_FRAME_LEN_MAX) {
                buf[buf_i++] = c;
                buf_len = c + 2; //total frame length: device address + len + type + payload + crc
            } else  {
                buf_i = 0;
            }
        }
        else if (buf_i > 1 && buf_i < buf_len-1) { //type, payload
            buf[buf_i++] = c;
        }
        else if (buf_i == buf_len-1) { //crc
            buf[buf_i++] = c;
            uint8_t crc = crsf_crc8(buf+2, buf[1] - 1);
            if (crc == buf[buf_len-1]) {
                received_channels = decode();
                buf_i = 0;
            }
        } else { //should not get here
            buf_i = 0;
        }
        return received_channels;
    }

private:
    uint8_t buf[CRSF_FRAME_SIZE_MAX];
    uint8_t buf_i;
    uint8_t buf_len;

    bool decode() {
        if (buf[CRSF_BUFPOS_TYPE] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED && buf_len == CRSF_PAYLOADLEN_RC_CHANNELS_PACKED + 4) {
            channel_ts = millis();
            channel[0] = convert_channel_value((buf[3] | buf[4] << 8) & 0x07FF);
            channel[1] = convert_channel_value((buf[4] >> 3 | buf[5] << 5) & 0x07FF);
            channel[2] = convert_channel_value((buf[5] >> 6 | buf[6] << 2 | buf[7] << 10) & 0x07FF);
            channel[3] = convert_channel_value((buf[7] >> 1 | buf[8] << 7) & 0x07FF);
            channel[4] = convert_channel_value((buf[8] >> 4 | buf[9] << 4) & 0x07FF);
            channel[5] = convert_channel_value((buf[9] >> 7 | buf[10] << 1 | buf[11] << 9) & 0x07FF);
            channel[6] = convert_channel_value((buf[11] >> 2 | buf[12] << 6) & 0x07FF);
            channel[7] = convert_channel_value((buf[12] >> 5 | buf[13] << 3) & 0x07FF);
            channel[8] = convert_channel_value((buf[14] | buf[15] << 8) & 0x07FF);
            channel[9] = convert_channel_value((buf[15] >> 3 | buf[16] << 5) & 0x07FF);
            channel[10] = convert_channel_value((buf[16] >> 6 | buf[17] << 2 | buf[18] << 10) & 0x07FF);
            channel[11] = convert_channel_value((buf[18] >> 1 | buf[19] << 7) & 0x07FF);
            channel[12] = convert_channel_value((buf[19] >> 4 | buf[20] << 4) & 0x07FF);
            channel[13] = convert_channel_value((buf[20] >> 7 | buf[21] << 1 | buf[22] << 9) & 0x07FF);
            channel[14] = convert_channel_value((buf[22] >> 2 | buf[23] << 6) & 0x07FF);
            channel[15] = convert_channel_value((buf[23] >> 5 | buf[24] << 3) & 0x07FF);
            return true;
        }
        return false;
    }

    uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) const
    {
        static const uint8_t crsf_crc8tab[256] = {
            0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
            0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
            0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
            0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
            0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
            0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
            0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
            0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
            0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
            0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
            0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
            0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
            0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
            0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
            0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
            0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

        uint8_t crc = 0;
        for (uint8_t i = 0; i < len; i++)
        {
            crc = crsf_crc8tab[crc ^ *ptr++];
        }
        return crc;
    }
    
    uint16_t convert_channel_value(unsigned chan_value)
    {
        /*
         *       RC     PWM
         * min  172 ->  988us
         * mid  992 -> 1500us
         * max 1811 -> 2012us
         */
        static constexpr float scale = (2012.f - 988.f) / (1811.f - 172.f);
        static constexpr float offset = 988.f - 172.f * scale;
        return (scale * chan_value) + offset;
    }
};
