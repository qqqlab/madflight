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

//=================================================================================================
// VEH - Vehicle info
//=================================================================================================

#pragma once

#include <stdint.h> //uint8_t

//Vehicle type (corresponding to mavlink MAV_TYPE)
#define VEH_TYPE_GENERIC 0 //0 MAV_TYPE_GENERIC	Generic micro air vehicle
#define VEH_TYPE_PLANE 1   //1 MAV_TYPE_FIXED_WING	Fixed wing aircraft
#define VEH_TYPE_COPTER 2  //2	MAV_TYPE_QUADROTOR	Quadrotor
#define VEH_TYPE_ROVER 10  //10	MAV_TYPE_GROUND_ROVER	Ground rover
#define VEH_TYPE_SUB 12    //12	MAV_TYPE_SUBMARINE	Submarine

//-------------------------------------------------------
// ArduPilot flight modes
//-------------------------------------------------------

// this we got from ArduCopter/mode.h
typedef enum {
    AP_COPTER_FLIGHTMODE_STABILIZE =     0,  // manual airframe angle with manual throttle
    AP_COPTER_FLIGHTMODE_ACRO =          1,  // manual body-frame angular rate with manual throttle
    AP_COPTER_FLIGHTMODE_ALT_HOLD =      2,  // manual airframe angle with automatic throttle
    AP_COPTER_FLIGHTMODE_AUTO =          3,  // fully automatic waypoint control using mission commands
    AP_COPTER_FLIGHTMODE_GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    AP_COPTER_FLIGHTMODE_LOITER =        5,  // automatic horizontal acceleration with automatic throttle
    AP_COPTER_FLIGHTMODE_RTL =           6,  // automatic return to launching point
    AP_COPTER_FLIGHTMODE_CIRCLE =        7,  // automatic circular flight with automatic throttle
    AP_COPTER_FLIGHTMODE_LAND =          9,  // automatic landing with horizontal position control
    AP_COPTER_FLIGHTMODE_DRIFT =        11,  // semi-autonomous position, yaw and throttle control
    AP_COPTER_FLIGHTMODE_SPORT =        13,  // manual earth-frame angular rate control with manual throttle
    AP_COPTER_FLIGHTMODE_FLIP =         14,  // automatically flip the vehicle on the roll axis
    AP_COPTER_FLIGHTMODE_AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
    AP_COPTER_FLIGHTMODE_POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
    AP_COPTER_FLIGHTMODE_BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
    AP_COPTER_FLIGHTMODE_THROW =        18,  // throw to launch mode using inertial/GPS system, no pilot input
    AP_COPTER_FLIGHTMODE_AVOID_ADSB =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
    AP_COPTER_FLIGHTMODE_GUIDED_NOGPS = 20,  // guided mode but only accepts attitude and altitude
    AP_COPTER_FLIGHTMODE_SMART_RTL =    21,  // SMART_RTL returns to home by retracing its steps
    AP_COPTER_FLIGHTMODE_FLOWHOLD  =    22,  // FLOWHOLD holds position with optical flow without rangefinder
    AP_COPTER_FLIGHTMODE_FOLLOW    =    23,  // follow attempts to follow another vehicle or ground station
    AP_COPTER_FLIGHTMODE_ZIGZAG    =    24,  // ZIGZAG mode is able to fly in a zigzag manner with predefined point A and point B
    AP_COPTER_FLIGHTMODE_SYSTEMID  =    25,  // System ID mode produces automated system identification signals in the controllers
    AP_COPTER_FLIGHTMODE_AUTOROTATE =   26,  // Autonomous autorotation
    AP_COPTER_FLIGHTMODE_AUTO_RTL =     27,  // Auto RTL, this is not a true mode, AUTO will report as this mode if entered to perform a DO_LAND_START Landing sequence
    AP_COPTER_FLIGHTMODE_TURTLE =       28,  // Flip over after crash
} AP_COPTER_FLIGHTMODE_ENUM;


// this we got from ArduPlane/mode.h
typedef enum {
    AP_PLANE_FLIGHTMODE_MANUAL        = 0,
    AP_PLANE_FLIGHTMODE_CIRCLE        = 1,
    AP_PLANE_FLIGHTMODE_STABILIZE     = 2,
    AP_PLANE_FLIGHTMODE_TRAINING      = 3,
    AP_PLANE_FLIGHTMODE_ACRO          = 4,
    AP_PLANE_FLIGHTMODE_FLY_BY_WIRE_A = 5,
    AP_PLANE_FLIGHTMODE_FLY_BY_WIRE_B = 6,
    AP_PLANE_FLIGHTMODE_CRUISE        = 7,
    AP_PLANE_FLIGHTMODE_AUTOTUNE      = 8,
    AP_PLANE_FLIGHTMODE_AUTO          = 10,
    AP_PLANE_FLIGHTMODE_RTL           = 11,
    AP_PLANE_FLIGHTMODE_LOITER        = 12,
    AP_PLANE_FLIGHTMODE_TAKEOFF       = 13,
    AP_PLANE_FLIGHTMODE_AVOID_ADSB    = 14,
    AP_PLANE_FLIGHTMODE_GUIDED        = 15,
    AP_PLANE_FLIGHTMODE_INITIALISING  = 16,
    AP_PLANE_FLIGHTMODE_QSTABILIZE    = 17,
    AP_PLANE_FLIGHTMODE_QHOVER        = 18,
    AP_PLANE_FLIGHTMODE_QLOITER       = 19,
    AP_PLANE_FLIGHTMODE_QLAND         = 20,
    AP_PLANE_FLIGHTMODE_QRTL          = 21,
    AP_PLANE_FLIGHTMODE_QAUTOTUNE     = 22,
    AP_PLANE_FLIGHTMODE_QACRO         = 23,
    AP_PLANE_FLIGHTMODE_THERMAL       = 24,
    AP_PLANE_FLIGHTMODE_LOITER_ALT_QLAND = 25,
} AP_PLANE_FLIGHTMODE_ENUM;

// this we got from Rover/mode.h
typedef enum {
    AP_ROVER_FLIGHTMODE_MANUAL       = 0,
    AP_ROVER_FLIGHTMODE_ACRO         = 1,
    AP_ROVER_FLIGHTMODE_STEERING     = 3,
    AP_ROVER_FLIGHTMODE_HOLD         = 4,
    AP_ROVER_FLIGHTMODE_LOITER       = 5,
    AP_ROVER_FLIGHTMODE_FOLLOW       = 6,
    AP_ROVER_FLIGHTMODE_SIMPLE       = 7,
    AP_ROVER_FLIGHTMODE_DOCK         = 8,
    AP_ROVER_FLIGHTMODE_AUTO         = 10,
    AP_ROVER_FLIGHTMODE_RTL          = 11,
    AP_ROVER_FLIGHTMODE_SMART_RTL    = 12,
    AP_ROVER_FLIGHTMODE_GUIDED       = 15,
    AP_ROVER_FLIGHTMODE_INITIALISING = 16,
} AP_ROVER_FLIGHTMODE_ENUM;


// this we got from ArduSub/defines.h
typedef enum {
    AP_SUB_FLIGHTMODE_STABILIZE =     0,  // manual angle with manual depth/throttle
    AP_SUB_FLIGHTMODE_ACRO =          1,  // manual body-frame angular rate with manual depth/throttle
    AP_SUB_FLIGHTMODE_ALT_HOLD =      2,  // manual angle with automatic depth/throttle
    AP_SUB_FLIGHTMODE_AUTO =          3,  // fully automatic waypoint control using mission commands
    AP_SUB_FLIGHTMODE_GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    AP_SUB_FLIGHTMODE_CIRCLE =        7,  // automatic circular flight with automatic throttle
    AP_SUB_FLIGHTMODE_SURFACE =       9,  // automatically return to surface, pilot maintains horizontal control
    AP_SUB_FLIGHTMODE_POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
    AP_SUB_FLIGHTMODE_MANUAL =       19,  // Pass-through input with no stabilization
    AP_SUB_FLIGHTMODE_MOTOR_DETECT = 20   // Automatically detect motors orientation
} AP_SUB_FLIGHTMODE_ENUM;

class Veh {
  public:
    static const uint8_t mav_type; //mavlink vehicle type
    static const uint8_t flightmode_ap_ids[6]; //mapping from flightmode to ArduPilot flight mode id
    static const char* flightmode_names[6]; //define flightmode name strings for telemetry
  
    bool setFlightmode(uint8_t flightmode); //returns true if flightmode changed
    uint8_t getFlightmode();
    uint8_t flightmode_ap_id();
    const char* flightmode_name();

  private:
    uint8_t _flightmode = 0; //current flight mode index
    const char* flightmode_name_unknown = "???";
};



extern Veh veh;
