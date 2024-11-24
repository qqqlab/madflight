/*==========================================================================================
telem.h - madflight RC radio receiver telemetry

MIT License

Copyright (c) 2023-2024 https://madflight.com

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

#include "../interface.h"

/* INTERFACE

#define RCIN_USE_NONE  0
#define RCIN_USE_CRSF  1
#define RCIN_USE_SBUS  2
#define RCIN_USE_DSM   3
#define RCIN_USE_PPM   4
#define RCIN_USE_PWM   5
#define RCIN_USE_DEBUG 6

void rcin_telemetry_gps(int32_t latitude, int32_t longitude, uint16_t groundspeed, uint16_t gps_heading, uint16_t altitude, uint8_t num_satellites);
void rcin_telemetry_flight_mode(const char *flight_mode);
void rcin_telemetry_attitude(float pitch, float roll, float yaw);
void rcin_telemetry_battery(float voltage_V, float current_A, int fuel_mAh, uint8_t remaining);
*/
//========================================================================================================================
//  CRSF
//========================================================================================================================
#if RCIN_USE == RCIN_USE_CRSF

#include "crsf/crsf_telemetry.h"
void rcin_telemetry_gps(int32_t lat, int32_t lon, uint16_t sog_kmh, uint16_t cog_deg, uint16_t alt_m, uint8_t sats) {
    uint8_t buf[65];
    int len = CRSF_Telemetry::telemetry_gps(buf, lat, lon, sog_kmh, cog_deg, alt_m, sats);
    rcin_Serial->write(buf, len);
}

void rcin_telemetry_flight_mode(const char *flight_mode) {
    uint8_t buf[65];
    int len = CRSF_Telemetry::telemetry_flight_mode(buf, flight_mode);
    rcin_Serial->write(buf, len);
    //Serial.printf("\nFM(len=%d) ",len);
    //for(int i=0;i<len;i++) Serial.printf("%02X ",buf[i]);
}

void rcin_telemetry_attitude(float pitch, float roll, float yaw) {
    uint8_t buf[65];
    int len = CRSF_Telemetry::telemetry_attitude(buf, pitch, roll, yaw);
    rcin_Serial->write(buf, len);
    //Serial.printf("\natt(len=%d) ",len);
    //for(int i=0;i<len;i++) Serial.printf("%02X ",buf[i]);
}

void rcin_telemetry_battery(float voltage_V, float current_A, int fuel_mAh, uint8_t remaining) {
    uint8_t buf[65];
    int len = CRSF_Telemetry::telemetry_battery(buf, voltage_V, current_A, fuel_mAh, remaining);
    rcin_Serial->write(buf, len);
}

//========================================================================================================================
//  OTHERS
//========================================================================================================================
#else

void rcin_telemetry_gps(int32_t latitude, int32_t longitude, uint16_t groundspeed, uint16_t gps_heading, uint16_t altitude, uint8_t num_satellites) {
}

void rcin_telemetry_flight_mode(const char *flight_mode) {
}

void rcin_telemetry_attitude(float pitch, float roll, float yaw) {
}

void rcin_telemetry_battery(float voltage_V, float current_A, int fuel_mAh, uint8_t remaining) {
}

#endif
