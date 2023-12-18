//telemetry_xxx() fills buffer with crsf command and returns length



//based on: https://github.com/PX4/PX4-Autopilot

/*
BSD 3-Clause License

Copyright (c) 2012 - 2023, PX4 Development Team
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#define CRSF_SYNC_BYTE 0xC8

enum class crsf_frame_type_t : uint8_t {
	gps = 0x02,
	battery_sensor = 0x08,
	link_statistics = 0x14,
	rc_channels_packed = 0x16,
	attitude = 0x1E,
	flight_mode = 0x21,

	// Extended Header Frames, range: 0x28 to 0x96
	device_ping = 0x28,
	device_info = 0x29,
	parameter_settings_entry = 0x2B,
	parameter_read = 0x2C,
	parameter_write = 0x2D,
	command = 0x32
};

enum class crsf_payload_size_t : uint8_t {
	gps = 15,
	battery_sensor = 8,
	link_statistics = 10,
	rc_channels = 22, ///< 11 bits per channel * 16 channels = 22 bytes.
	attitude = 6,
};

class CRSF_Telemetry {
public:


static int telemetry_battery(uint8_t *buf, uint16_t voltage, uint16_t current, int fuel, uint8_t remaining)
{
	int offset = 0;
	write_frame_header(buf, offset, crsf_frame_type_t::battery_sensor, (uint8_t)crsf_payload_size_t::battery_sensor);
	write_uint16_t(buf, offset, voltage);
	write_uint16_t(buf, offset, current);
	write_uint24_t(buf, offset, fuel);
	write_uint8_t(buf, offset, remaining);
	write_frame_crc(buf, offset, sizeof(buf));
	return offset;
}

static int telemetry_gps(uint8_t *buf, int32_t latitude, int32_t longitude, uint16_t groundspeed, uint16_t gps_heading, uint16_t altitude, uint8_t num_satellites)
{
	int offset = 0;
	write_frame_header(buf, offset, crsf_frame_type_t::gps, (uint8_t)crsf_payload_size_t::gps);
	write_int32_t(buf, offset, latitude);
	write_int32_t(buf, offset, longitude);
	write_uint16_t(buf, offset, groundspeed);
	write_uint16_t(buf, offset, gps_heading);
	write_uint16_t(buf, offset, altitude);
	write_uint8_t(buf, offset, num_satellites);
	write_frame_crc(buf, offset, sizeof(buf));
	return offset;
}

static int telemetry_attitude(uint8_t *buf, int16_t pitch, int16_t roll, int16_t yaw)
{
	int offset = 0;
	write_frame_header(buf, offset, crsf_frame_type_t::attitude, (uint8_t)crsf_payload_size_t::attitude);
	write_uint16_t(buf, offset, pitch);
	write_uint16_t(buf, offset, roll);
	write_uint16_t(buf, offset, yaw);
	write_frame_crc(buf, offset, sizeof(buf));
	return offset;
}

static int telemetry_flight_mode(uint8_t *buf, const char *flight_mode)
{
	const int max_length = 16;
	int length = strlen(flight_mode) + 1;
	if (length > max_length) length = max_length;

	int offset = 0;
	write_frame_header(buf, offset, crsf_frame_type_t::flight_mode, length);
	memcpy(buf + offset, flight_mode, length);
	offset += length;
	buf[offset - 1] = 0; // ensure null-terminated string
	write_frame_crc(buf, offset, length + 4);
	return offset;
}


private:

/**
 * write an uint8_t value to a buffer at a given offset and increment the offset
 */
static inline void write_uint8_t(uint8_t *buf, int &offset, uint8_t value)
{
	buf[offset++] = value;
}
/**
 * write an uint16_t value to a buffer at a given offset and increment the offset
 */
static inline void write_uint16_t(uint8_t *buf, int &offset, uint16_t value)
{
	// Big endian
	buf[offset] = value >> 8;
	buf[offset + 1] = value & 0xff;
	offset += 2;
}
/**
 * write an uint24_t value to a buffer at a given offset and increment the offset
 */
static inline void write_uint24_t(uint8_t *buf, int &offset, int value)
{
	// Big endian
	buf[offset] = value >> 16;
	buf[offset + 1] = (value >> 8) & 0xff;
	buf[offset + 2] = value & 0xff;
	offset += 3;
}

/**
 * write an int32_t value to a buffer at a given offset and increment the offset
 */
static inline void write_int32_t(uint8_t *buf, int &offset, int32_t value)
{
	// Big endian
	buf[offset] = value >> 24;
	buf[offset + 1] = (value >> 16) & 0xff;
	buf[offset + 2] = (value >> 8) & 0xff;
	buf[offset + 3] = value & 0xff;
	offset += 4;
}

static inline void write_frame_header(uint8_t *buf, int &offset, crsf_frame_type_t type, uint8_t payload_size)
{
	write_uint8_t(buf, offset, CRSF_SYNC_BYTE); // this got changed from the address to the sync byte
	write_uint8_t(buf, offset, payload_size + 2);
	write_uint8_t(buf, offset, (uint8_t)type);
}
static inline void write_frame_crc(uint8_t *buf, int &offset, int buf_size)
{
	// CRC does not include the address and length
	write_uint8_t(buf, offset, crc8_dvb_s2_buf(buf + 2, buf_size - 3));

	// check correctness of buffer size (only needed during development)
	//if (buf_size != offset) { PX4_ERR("frame size mismatch (%i != %i)", buf_size, offset); }
}

static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a)
{
	crc ^= a;
	for (int i = 0; i < 8; ++i) {
		if (crc & 0x80) {
			crc = (crc << 1) ^ 0xD5;
		} else {
			crc = crc << 1;
		}
	}
	return crc;
}

static uint8_t crc8_dvb_s2_buf(uint8_t *buf, int len)
{
	uint8_t crc = 0;
	for (int i = 0; i < len; ++i) {
		crc = crc8_dvb_s2(crc, buf[i]);
	}
	return crc;
}

};