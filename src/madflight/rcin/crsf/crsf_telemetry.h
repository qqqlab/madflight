//telemetry_xxx() fills buffer with crsf command and returns length


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


/*
0x08 Battery sensor
Payload:
uint16_t    Voltage ( 0.1 V )
uint16_t    Current ( 0.1 A )
uint24_t    Fuel ( drawn mAh )
uint8_t     Battery remaining ( percent )
*/
static int telemetry_battery(uint8_t *buf, float voltage_V, float current_A, int32_t fuel_mAh, uint8_t remaining)
{
	int offset = 0;
	write_frame_header(buf, offset, crsf_frame_type_t::battery_sensor, (uint8_t)crsf_payload_size_t::battery_sensor);
	write_uint16_t(buf, offset, voltage_V * 10);
	write_uint16_t(buf, offset, current_A * 10);
	write_uint24_t(buf, offset, fuel_mAh);
	write_uint8_t(buf, offset, remaining);
	write_frame_crc(buf, offset);
	return offset;
}

/*
0x02 GPS
Payload:
int32_t     Latitude ( degree / 10`000`000 )
int32_t     Longitude (degree / 10`000`000 )
uint16_t    Groundspeed ( km/h / 10 )
uint16_t    GPS heading ( degree / 100 )
uint16      Altitude ( meter ­1000m offset )
uint8_t     Satellites in use ( counter )
*/
static int telemetry_gps(uint8_t *buf, int32_t latitude, int32_t longitude, uint16_t groundspeed, uint16_t gps_heading, uint16_t altitude, uint8_t num_satellites)
{
	int offset = 0;
	write_frame_header(buf, offset, crsf_frame_type_t::gps, (uint8_t)crsf_payload_size_t::gps);
	write_int32_t(buf, offset, latitude);
	write_int32_t(buf, offset, longitude);
	write_uint16_t(buf, offset, groundspeed);
	write_uint16_t(buf, offset, gps_heading);
	write_uint16_t(buf, offset, altitude + 1000);
	write_uint8_t(buf, offset, num_satellites);
	write_frame_crc(buf, offset);
	return offset;
}

// convert angle in degrees to radians/10000 with reducing angle to +/-180 degree range
static int16_t degrees2Radians10000(float angle_deg)
{
    while (angle_deg > 180) {
        angle_deg -= 360;
    }
    while (angle_deg < -180) {
        angle_deg += 360;
    }
    return (int16_t)(174.532925f * angle_deg);
}

/*
0x1E Attitude
Payload:
int16_t     Pitch angle ( rad / 10000 )
int16_t     Roll angle ( rad / 10000 )
int16_t     Yaw angle ( rad / 10000 )
*/
static int telemetry_attitude(uint8_t *buf, float pitch, float roll, float yaw)
{
	int offset = 0;
	write_frame_header(buf, offset, crsf_frame_type_t::attitude, (uint8_t)crsf_payload_size_t::attitude);
	write_uint16_t(buf, offset, degrees2Radians10000(pitch));
	write_uint16_t(buf, offset, degrees2Radians10000(roll));
	write_uint16_t(buf, offset, degrees2Radians10000(yaw));
	write_frame_crc(buf, offset);
	return offset;
}
/*
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
*/
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
	write_frame_crc(buf, offset);
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
static inline void write_frame_crc(uint8_t *buf, int &offset)
{
	// CRC does not include the address and length
	write_uint8_t(buf, offset, crc8_dvb_s2_buf(buf + 2, offset - 2));
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