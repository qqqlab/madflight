/*==========================================================================================
rcin_mavlink.h - MAVlink interface for madflight

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
TODO
MAVLINK_MSG_ID_VFR_HUD (climb, airspeed, groundspeed, heading)
MAVLINK_MSG_ID_HOME_POSITION
MAVLINK_MSG_ID_ALTITUDE // send the terrain message to Yaapu Telemetry Script
MAVLINK_MSG_ID_HIGH_LATENCY2 // send the waypoint message to Yaapu Telemetry Script
MAVLINK_MSG_ID_REQUEST_DATA_STREAM

SERIAL_CONTROL (126)  //Qgroundcontrol mavlink console

*/

#pragma once

//include all module interfaces
#include "../../ahrs/ahrs_interface.h"
#include "../../alt/alt_interface.h"
#include "../../baro/baro_interface.h"
#include "../../bat/bat_interface.h"
#include "../../bb/bb_interface.h"
#include "../../cfg/cfg_interface.h"
//#include "../../cli/cli_interface.h"
#include "../../gps/gps_interface.h"
#include "../../imu/imu_interface.h"
#include "../../led/led_interface.h"
#include "../../mag/mag_interface.h"
#include "../../out/out_interface.h"
#include "../../pid/pid_interface.h"
#include "../../rcin/rcin_interface.h"
#include "../../veh/veh_interface.h"

//-------------------------------------------------------------------------------------------
// include mavlink library
//-------------------------------------------------------------------------------------------
//The MAVLink protocol code generator does its own alignment, so alignment cast warnings can be ignored
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
#if defined(__GNUC__) && __GNUC__ >= 9
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#endif

//reduce RAM footprint
#define MAVLINK_COMM_NUM_BUFFERS 1

#include "mavlink_c_library_v2/ardupilotmega/mavlink.h"
//-------------------------------------------------------------------------------------------

#include "../../cfg/cfg_interface.h"

class RcinMavlink : public Rcin {
  public:
    void _setup();
    bool _update();
    bool telem_statustext(uint8_t severity, char *text);
    uint16_t pwm_instance[18];

  private:
    bool receive();
    
    //TELEM
    SemaphoreHandle_t tx_mux; //UART TX mutex
    void telem_update();
    bool telem_send(mavlink_message_t *msg, uint16_t timeout_ms = 0);
    
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
    typedef bool (RcinMavlink::*sched_func_t)();

    struct telem_sched_t {
      sched_func_t func;
      uint32_t interval_ms;
      uint32_t last_ms = 0;
    };

    static telem_sched_t telem_sched[]; 
    uint8_t telem_sched_idx;
};

#ifndef INT16_MAX
  #define INT16_MAX 32767
#endif



void RcinMavlink::_setup() {
  Serial.println("RCIN: RCIN_USE_MAVLINK");
  pwm = pwm_instance;
  tx_mux = xSemaphoreCreateMutex();
  rcin_Serial->begin(460800);
}

//process received mavlink data
bool RcinMavlink::_update() {
  bool rv = receive(); //do receive first, so that it has a greater chance to have enough txbuf space to send a reply if needed
  telem_update();
  return rv;
}

//receive mavlink data
bool RcinMavlink::receive() {
  int packetSize = rcin_Serial->available();
  if (!packetSize) return false;

  mavlink_message_t msg;
  mavlink_status_t status;
  for (int i = 0; i < packetSize; i++) {
    uint8_t c;
    rcin_Serial->read(&c, 1);
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: { //0
          //Serial.println("Received HEARTBEAT");
          break;
        }

        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: { //70
          mavlink_rc_channels_override_t m;
          mavlink_msg_rc_channels_override_decode(&msg, &m);
          //Serial.printf("Received RC_CHANNELS_OVERRIDE: ch1:%d ch2:%d ch3:%d ch4:%d\n", (int)m.chan1_raw, (int)m.chan2_raw, (int)m.chan3_raw, (int)m.chan4_raw);
          pwm_instance[0] = m.chan1_raw;
          pwm_instance[1] = m.chan2_raw;
          pwm_instance[2] = m.chan3_raw;
          pwm_instance[3] = m.chan4_raw;
          pwm_instance[4] = m.chan5_raw;
          pwm_instance[5] = m.chan6_raw;
          pwm_instance[6] = m.chan7_raw;
          pwm_instance[7] = m.chan8_raw;
          pwm_instance[8] = m.chan9_raw;
          pwm_instance[9] = m.chan10_raw;
          pwm_instance[10] = m.chan11_raw;
          pwm_instance[11] = m.chan12_raw;
          pwm_instance[12] = m.chan13_raw;
          pwm_instance[13] = m.chan14_raw;
          pwm_instance[14] = m.chan15_raw;
          pwm_instance[15] = m.chan16_raw;
          pwm_instance[16] = m.chan17_raw;
          pwm_instance[17] = m.chan18_raw;
          return true;
        }

        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: { //21
          //Serial.println("Received PARAM_REQUEST_LIST");
          telem_param_value_enable();
          break; 
        }

        case MAVLINK_MSG_ID_PARAM_REQUEST_READ: { //20
          mavlink_param_request_read_t m;
          mavlink_msg_param_request_read_decode(&msg, &m);
          if(m.param_index>=0) {
            telem_param_value(m.param_index);
          }else{
            char name[17];
            memcpy(name, m.param_id, 16);
            name[16] = 0;
            int param_index = cfg.getIndex(String(name));
            if(param_index >= 0) telem_param_value(param_index);
          }
          break;
        }

        case MAVLINK_MSG_ID_PARAM_SET: { //23
          mavlink_param_set_t m;
          mavlink_msg_param_set_decode(&msg, &m);
          char name[17];
          memcpy(name, m.param_id, 16);
          name[16] = 0;
          int param_index = cfg.set(String(name), m.param_value);
          if(param_index >= 0) telem_param_value(param_index);
          break;
        }

        case MAVLINK_MSG_ID_RADIO_STATUS: { //109
          //mavlink_radio_status_t m;
          //mavlink_msg_radio_status_decode(&msg, &m);
          //Serial.printf("Received RADIO_STATUS: rssi:%d\n",(int)m.rssi);
          break; 
        }

        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: { //66
          //TODO
          break;
        }

        default: {
          //Serial.printf("MAV: %d\n",msg.msgid);
          break; 
        }
      }
    }
  }
  return false;
}



//==================================================================================================
// TELEM
//==================================================================================================


//messages to send with period in ms (0 is off)
RcinMavlink::telem_sched_t RcinMavlink::telem_sched[] = {
  {&RcinMavlink::telem_param_list,             0}, //telem_param_list needs to be telem_sched[0]
  {&RcinMavlink::telem_heartbeat,           1000},
  {&RcinMavlink::telem_attitude,             250},
  {&RcinMavlink::telem_global_position_int, 1000},
  {&RcinMavlink::telem_gps_raw_int,         1000},
  {&RcinMavlink::telem_battery_status,      2000},
};


bool RcinMavlink::telem_send(mavlink_message_t *msg, uint16_t timeout_ms) {
  bool rv = false;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
  
  //take mutex
  uint32_t start = millis();
  if(pdTRUE == xSemaphoreTake(tx_mux, pdMS_TO_TICKS(timeout_ms))) {
    do {
      if(rcin_Serial->availableForWrite() >= len) {
        rcin_Serial->write(buf, len);
        rv = true;
        break;
      }
    } while(millis() - start < timeout_ms);
    xSemaphoreGive(tx_mux);
  }
  return rv;
}


bool RcinMavlink::telem_heartbeat() {
  //Note: ArduPilot also sets bits MAV_MODE_FLAG_STABILIZE_ENABLED and/or MAV_MODE_FLAG_GUIDED_ENABLED (for RTL) depending on flightmode. Not done here, but works fine with yaapu/MP/QGC.
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg
    , veh.mav_type // uint8_t type; /*<  Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type.*/
    , MAV_AUTOPILOT_ARDUPILOTMEGA // uint8_t autopilot; /*<  Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.*/
    , MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | (out.armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0) // uint8_t base_mode; /*<  System mode bitmap.*/
    , veh.flightmode_ap_id() // uint32_t custom_mode; /*<  A bitfield for use for autopilot-specific flags*/
    , MAV_STATE_STANDBY // uint8_t system_status; /*<  System status flag.*/
  );
  return telem_send(&msg);
}

bool RcinMavlink::telem_attitude() {
  mavlink_message_t msg;
  //mavlink values are in rad & rad/s
  mavlink_msg_attitude_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg
    , millis()
    , ahrs.roll*0.0174533f
    , ahrs.pitch*0.0174533f
    , ahrs.yaw*0.0174533f
    , ahrs.gx*0.0174533f
    , ahrs.gy*0.0174533f
    , ahrs.gz*0.0174533f
  );
  return telem_send(&msg);
}

bool RcinMavlink::telem_global_position_int() { //33
  mavlink_message_t msg;
  mavlink_msg_global_position_int_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg
    , millis() //time_boot_ms	uint32_t	ms	Timestamp (time since system boot).
    , gps.lat  //lat	int32_t	degE7	Latitude, expressed
    , gps.lon  //lon	int32_t	degE7	Longitude, expressed
    , gps.alt  //alt	int32_t mm	Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
    , gps.alt  //relative_alt	int32_t	mm	Altitude above home
    , gps.veln/10 //vx	int16_t	cm/s	Ground X Speed (Latitude, positive north)
    , gps.vele/10 //vy	int16_t	cm/s	Ground Y Speed (Longitude, positive east)
    , (gps.have_veld ? gps.veld/10 : 0) //vz	int16_t	cm/s	Ground Z Speed (Altitude, positive down)
    , UINT16_MAX //hdg	uint16_t	cdeg	Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
  );
  return telem_send(&msg);
}

bool RcinMavlink::telem_gps_raw_int() {
  mavlink_message_t msg;
  mavlink_msg_gps_raw_int_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg
    , millis()   // uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
    , gps.fix    // uint8_t fix_type; /*<  GPS fix type.*/
    , gps.lat    // int32_t lat; /*< [degE7] Latitude (WGS84, EGM96 ellipsoid)*/
    , gps.lon    // int32_t lon; /*< [degE7] Longitude (WGS84, EGM96 ellipsoid)*/
    , gps.alt    // int32_t alt; /*< [mm] Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.*/
    , gps.hdop   // uint16_t eph; /*<  GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX*/
    , gps.vdop   // uint16_t epv; /*<  GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX*/
    , gps.sog/10 // uint16_t vel; /*< [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX*/
    , gps.cog/10 // uint16_t cog; /*< [cdeg] Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
    , gps.sat    // uint8_t satellites_visible; /*<  Number of satellites visible. If unknown, set to UINT8_MAX*/
    , gps.alt    // int32_t alt_ellipsoid; /*< [mm] Altitude (above WGS84, EGM96 ellipsoid). Positive for up.*/
    , gps.hacc   // uint32_t h_acc; /*< [mm] Position uncertainty.*/
    , gps.vacc   // uint32_t v_acc; /*< [mm] Altitude uncertainty.*/
    , (gps.have_vel_acc ? gps.vel_acc : 0) // uint32_t vel_acc; /*< [mm/s] Speed uncertainty.*/
    , 0          // uint32_t hdg_acc; /*< [degE5] Heading / track uncertainty*/
    , 0          // uint16_t yaw; /*< [cdeg] Yaw in earth frame from north. Use 0 if this GPS does not provide yaw. Use UINT16_MAX if this GPS is configured to provide yaw and is currently unable to provide it. Use 36000 for north.*/
  );
  return telem_send(&msg);
}

bool RcinMavlink::telem_battery_status() {
  mavlink_message_t msg;
  uint16_t voltages[10] = {};
  uint16_t voltages_ext[4] = {};
  voltages[0] =  bat.v*1000;
  mavlink_msg_battery_status_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg
    , 0 //uint8_t id; /*<  Battery ID*/  
    , 0 //uint8_t battery_function; /*<  Function of the battery*/
    , 0 //uint8_t type; /*<  Type (chemistry) of the battery*/
    , INT16_MAX //int16_t temperature; /*< [cdegC] Temperature of the battery. INT16_MAX for unknown temperature.*/
    , voltages //uint16_t voltages[10]; /*< [mV] Battery voltage of cells 1 to 10 (see voltages_ext for cells 11-14). Cells in this field above the valid cell count for this battery should have the UINT16_MAX value. If individual cell voltages are unknown or not measured for this battery, then the overall battery voltage should be filled in cell 0, with all others set to UINT16_MAX. If the voltage of the battery is greater than (UINT16_MAX - 1), then cell 0 should be set to (UINT16_MAX - 1), and cell 1 to the remaining voltage. This can be extended to multiple cells if the total voltage is greater than 2 * (UINT16_MAX - 1).*/
    , bat.i*100  //int16_t current_battery; /*< [cA] Battery current, -1: autopilot does not measure the current*/
    , bat.mah //int32_t current_consumed; /*< [mAh] Consumed charge, -1: autopilot does not provide consumption estimate*/
    , bat.wh*36 //int32_t energy_consumed; /*< [hJ] Consumed energy, -1: autopilot does not provide energy consumption estimate*/
    , -1 //int8_t battery_remaining; /*< [%] Remaining battery energy. Values: [0-100], -1: autopilot does not estimate the remaining battery.*/
    , 0 //int32_t time_remaining; /*< [s] Remaining battery time, 0: autopilot does not provide remaining battery time estimate*/
    , 0 //uint8_t charge_state; /*<  State for extent of discharge, provided by autopilot for warning or external reactions*/
    , voltages_ext //uint16_t voltages_ext[4]; /*< [mV] Battery voltages for cells 11 to 14. Cells above the valid cell count for this battery should have a value of 0, where zero indicates not supported (note, this is different than for the voltages field and allows empty byte truncation). If the measured value is 0 then 1 should be sent instead.*/
    , 0 //uint8_t mode; /*<  Battery mode. Default (0) is that battery mode reporting is not supported or battery is in normal-use mode.*/
    , 0 //uint32_t fault_bitmask; /*<  Fault/health indications. These should be set when charge_state is MAV_BATTERY_CHARGE_STATE_FAILED or MAV_BATTERY_CHARGE_STATE_UNHEALTHY (if not, fault reporting is not supported).*/
  );
  return telem_send(&msg);
}
  
  
//start sending param value list
void RcinMavlink::telem_param_value_enable() {
  telem_param_list_index = 0;
  telem_sched[0].interval_ms = 25; //enable scheduler for param_value - ELRS 333Hz Full data link is 1600 bytes/s, one PARAM_VALUE is 39 bytes -> 25 ms/PARAM_VALUE
}

bool RcinMavlink::telem_param_value(uint16_t param_index) {
  uint16_t param_count = cfg.valueCount();
  if(param_index >= param_count) return true; //return true (i.e. command executed), even if nothing was send...
  
  String param_id;
  float param_value = 0;
  cfg.getNameAndValue(param_index, &param_id, &param_value);

  mavlink_message_t msg;
  mavlink_msg_param_value_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg
    , param_id.c_str()
    , param_value
    , MAV_PARAM_TYPE_REAL32
    , param_count
    , param_index
  );
  return telem_send(&msg);
}


bool RcinMavlink::telem_param_list() {
  uint16_t param_count = cfg.valueCount();
  if(telem_param_list_index < param_count) {
    if(telem_param_value(telem_param_list_index)) {
      telem_param_list_index++;
      return true;
    }else{
      return false;
    }
  }else{
    telem_sched[0].interval_ms = 0; //disable scheduler for param_value
    return true; //return true (i.e. command executed), even if nothing was send...
  }
}

//send telemetry data
void RcinMavlink::telem_update() {
  const uint8_t telem_sched_cnt = sizeof(RcinMavlink::telem_sched) / sizeof(RcinMavlink::telem_sched_t);
  uint32_t now = millis();
  for(int i=0;i<telem_sched_cnt;i++) {
    telem_sched_idx = (telem_sched_idx+1) % telem_sched_cnt;
    telem_sched_t *sch = &telem_sched[telem_sched_idx];
    if(sch->interval_ms && now - sch->last_ms > sch->interval_ms) {
      sched_func_t f = sch->func;
      if( (this->*f)() ) {
        sch->last_ms = now;
      }
    }
  }
}

/*
Status Text

Value	Name	Description
0	MAV_SEVERITY_EMERGENCY	System is unusable. This is a "panic" condition.
1	MAV_SEVERITY_ALERT	Action should be taken immediately. Indicates error in non-critical systems.
2	MAV_SEVERITY_CRITICAL	Action must be taken immediately. Indicates failure in a primary system.
3	MAV_SEVERITY_ERROR	Indicates an error in secondary/redundant systems.
4	MAV_SEVERITY_WARNING	Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning.
5	MAV_SEVERITY_NOTICE	An unusual event has occurred, though not an error condition. This should be investigated for the root cause.
6	MAV_SEVERITY_INFO	Normal operational messages. Useful for logging. No action is required for these messages.
7	MAV_SEVERITY_DEBUG	Useful non-operational messages that can assist in debugging. These should not occur during normal operation.
*/
bool RcinMavlink::telem_statustext(uint8_t severity, char *text) {
  mavlink_message_t msg;
  mavlink_msg_statustext_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, severity, text, 0, 0);
  return telem_send(&msg, 5); //wait up to 5ms to send message
}