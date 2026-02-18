//RclGizmoMavlink.h - MAVlink interface for madflight

/*==========================================================================================
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


/*
MF_TODO
MAVLINK_MSG_ID_VFR_HUD (climb, airspeed, groundspeed, heading)
MAVLINK_MSG_ID_HOME_POSITION
MAVLINK_MSG_ID_ALTITUDE // send the terrain message to Yaapu Telemetry Script
MAVLINK_MSG_ID_HIGH_LATENCY2 // send the waypoint message to Yaapu Telemetry Script
MAVLINK_MSG_ID_REQUEST_DATA_STREAM

SERIAL_CONTROL (126)  //Qgroundcontrol mavlink console
*/

#pragma once

#include "rcl.h" //RclGizmo
#include "../hal/hal.h" //MF_Serial, SemaphoreHandle_t

//-------------------------------------------------------------------------------------------
// include mavlink library
//-------------------------------------------------------------------------------------------
// The MAVLink protocol code generator does its own alignment, so alignment cast warnings can be ignored
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
#if defined(__GNUC__) && __GNUC__ >= 9
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#endif

// Prevent use of channel buffers ->> don't call any mavlink helpers with uint8_t chan argument
#define MAVLINK_COMM_NUM_BUFFERS 0
#define MAVLINK_GET_CHANNEL_BUFFER
#define MAVLINK_GET_CHANNEL_STATUS
#include "mavlink_c_library_v2/mavlink_types.h"
mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan);
mavlink_status_t* mavlink_get_channel_status(uint8_t chan);

// Finally, include mavlink.h
#include "mavlink_c_library_v2/ardupilotmega/mavlink.h"
//-------------------------------------------------------------------------------------------

class RclGizmoMavlink : public RclGizmo {
  public:
    RclGizmoMavlink(MF_Serial *ser_bus, int32_t baud, uint16_t* pwm);
    bool update() override;
    bool telem_statustext(uint8_t severity, char *text);
    uint16_t *pwm = nullptr;

    enum process_result_enum {
      NONE,
      MSG,
      PWM
    };

    process_result_enum process_char(uint8_t c); //returns true if a valid message was received
  private:
    mavlink_message_t mav_rxmsg; 
    mavlink_status_t mav_status;

    MF_Serial *ser_bus;
    bool receive();
    
    //TELEM
    SemaphoreHandle_t tx_mux; //UART TX mutex
    void telem_update();
    bool telem_send(mavlink_message_t *pmsg, uint16_t timeout_ms);
    
    //scheduled messages
    bool telem_heartbeat();
    bool telem_attitude();
    bool telem_param_list();
    bool telem_global_position_int();
    bool telem_gps_raw_int();
    bool telem_battery_status();

    //parameter micro service
    void telem_param_value_enable();
    bool telem_param_value(uint16_t param_index);
    uint16_t telem_param_list_index = 0; //next param_value to send

    //scheduler
    typedef bool (RclGizmoMavlink::*sched_func_t)();

    struct telem_sched_t {
      constexpr telem_sched_t(sched_func_t func, uint32_t interval_ms) : func(func), interval_ms(interval_ms), last_ms(0) {}
      sched_func_t func;
      uint32_t interval_ms;
      uint32_t last_ms = 0;
    };

    uint8_t telem_sched_idx = 0;

    //messages to send with period in ms (0 is off)
    telem_sched_t telem_sched[6] = {
      {&RclGizmoMavlink::telem_param_list,             0}, //telem_param_list needs to be telem_sched[0]
      {&RclGizmoMavlink::telem_heartbeat,           1000},
      {&RclGizmoMavlink::telem_global_position_int, 1000},
      {&RclGizmoMavlink::telem_gps_raw_int,         1000},
      {&RclGizmoMavlink::telem_battery_status,      2000},
      {&RclGizmoMavlink::telem_attitude,             250},
    };
};
