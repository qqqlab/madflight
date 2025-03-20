#pragma once

#include "../hal/MF_Serial.h"
#include "../cfg/cfg.h"

/// GPS fix codes.  These are kept aligned with MAVLink
enum GPS_Status {
    NO_GPS = 0,                  ///< No GPS connected/detected
    NO_FIX = 1,                  ///< Receiving valid GPS messages but no lock
    GPS_OK_FIX_2D = 2,           ///< Receiving valid messages and 2D lock
    GPS_OK_FIX_3D = 3,           ///< Receiving valid messages and 3D lock
    GPS_OK_FIX_3D_DGPS = 4,      ///< Receiving valid messages and 3D lock with differential improvements
    GPS_OK_FIX_3D_RTK_FLOAT = 5, ///< Receiving valid messages and 3D RTK Float
    GPS_OK_FIX_3D_RTK_FIXED = 6, ///< Receiving valid messages and 3D RTK Fixed
};

struct GpsState {
  public:
    // all the following fields must all be filled by the backend driver
    GPS_Status fix = NO_GPS;        // driver fix status
    uint8_t sat = 0;                //  Number of visible satellites
    uint16_t time_week = 0;         // GPS week number 
    uint32_t time = 0;              // GPS time (milliseconds from start of GPS week)
    int32_t lat = 0;                // last fix location in 1E-7 degrees
    int32_t lon = 0;                // last fix location in 1E-7 degrees
    int32_t alt = 0;                // last fix altitude msl in mm 
    int32_t sog = 0;                // ground speed in mm/s
    int32_t cog = 0;                // ground course in 1E-5 degrees
    uint32_t gps_yaw_time_ms = 0;   // timestamp of last GPS yaw reading
    uint16_t hdop = 9999;           // horizontal dilution of precision, scaled by a factor of 100 (155 means the HDOP value is 1.55)
    uint16_t vdop = 9999;           // vertical dilution of precision, scaled by a factor of 100 (155 means the VDOP value is 1.55)
    int32_t veln = 0;               // 3D velocity in mm/s, in NED format
    int32_t vele = 0;               // 3D velocity in mm/s, in NED format
    int32_t veld = 0;               // 3D velocity in mm/s, in NED format
    int32_t vel_acc = 999999;       // 3D velocity RMS accuracy estimate in mm/s
    int32_t hacc = 999999;          // horizontal RMS accuracy estimate in mm
    int32_t vacc = 999999;          // vertical RMS accuracy estimate in mm
    int32_t undulation = 0;         // height that WGS84 is above AMSL at the current location in mm
    uint32_t last_gps_time_ms = 0;  // the system time we got the last GPS timestamp, milliseconds
    bool have_veld = false;         // does GPS give vertical velocity? Set to true only once available.
    bool have_vel_acc = false;      // does GPS give speed accuracy? Set to true only once available.
    bool have_hacc = false;         // does GPS give horizontal position accuracy? Set to true only once available.
    bool have_vacc = false;         // does GPS give vertical position accuracy? Set to true only once available.
    bool have_undulation = false;   // do we have a value for the undulation
};

struct GpsConfig {
  public:
    Cfg::gps_gizmo_enum gizmo = Cfg::gps_gizmo_enum::mf_NONE; //the gizmo to use
    MF_Serial *ser_bus = nullptr; //Serial bus
    int baud = 0; //baud rate. 0=autobaud
};

class GpsGizmo {
  public:
    virtual ~GpsGizmo() {}
    virtual bool update() = 0; //returns true if new sample was taken
};

class Gps : public GpsState {
  public:
    GpsConfig config;

    GpsGizmo *gizmo = nullptr;

    int setup();      // Use config to setup gizmo, returns 0 on success, or error code
    bool update();    // Returns true if state was updated
    bool installed() {return (gizmo != nullptr); } // Returns true if a gizmo was setup
};

//Global module instance
extern Gps gps;
