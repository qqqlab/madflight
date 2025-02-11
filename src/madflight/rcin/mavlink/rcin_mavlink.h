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


class RcinMavlink : public Rcin {
  public:
    void _setup();
    bool _update();
    bool telem_statustext(uint8_t severity, char *text);
    uint16_t pwm_instance[18];

  //TELEM
    SemaphoreHandle_t tx_mux; //UART TX mutex
    void telem_update();
    bool telem_send(mavlink_message_t *msg, uint16_t timeout_ms = 0);
    
    //scheduled messages
    bool telem_heartbeat();
    bool telem_attitude();
    bool telem_param_list();
    bool telem_global_position_int();
/*
TODO
MAVLINK_MSG_ID_HEARTBEAT (fightmode & armed )
MAVLINK_MSG_ID_BATTERY_STATUS
MAVLINK_MSG_ID_GPS_RAW_INT
MAVLINK_MSG_ID_VFR_HUD (climb, airspeed, groundspeed, heading)
MAVLINK_MSG_ID_HOME_POSITION
MAVLINK_MSG_ID_ALTITUDE // send the terrain message to Yaapu Telemetry Script
MAVLINK_MSG_ID_HIGH_LATENCY2 // send the waypoint message to Yaapu Telemetry Script
MAVLINK_MSG_ID_REQUEST_DATA_STREAM
parameter read/write
*/

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


void RcinMavlink::_setup() {
  Serial.println("RCIN: RCIN_USE_MAVLINK");
  pwm = pwm_instance;
  tx_mux = xSemaphoreCreateMutex();
  rcin_Serial->begin(460800);
}

//process received mavlink data
bool RcinMavlink::_update() {
  telem_update();

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
        case MAVLINK_MSG_ID_RADIO_STATUS: { //109
          //mavlink_radio_status_t m;
          //mavlink_msg_radio_status_decode(&msg, &m);
          //Serial.printf("Received RADIO_STATUS: rssi:%d\n",(int)m.rssi);
          break; 
        }
        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: { //66
          break;
        }
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: { //21
          telem_param_value_enable();
          break; }
        default: {
          //Serial.print("Received message with ID %d\n",msg.msgid);
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
  {&RcinMavlink::telem_param_list,            0}, //telem_param_list needs to be telem_sched[0]
  {&RcinMavlink::telem_heartbeat,          1000},
  {&RcinMavlink::telem_attitude,            250},
  {&RcinMavlink::telem_global_position_int, 500},
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
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg
    , MAV_TYPE_QUADROTOR
    , MAV_AUTOPILOT_GENERIC
    , MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
    , 0
    , MAV_STATE_STANDBY
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
return true;
/*
  mavlink_message_t msg;
  mavlink_global_position_int_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg
    , millis() //time_boot_ms	uint32_t	ms	Timestamp (time since system boot).
    , gps.lat  //lat	int32_t	degE7	Latitude, expressed
    , gps.lon  //lon	int32_t	degE7	Longitude, expressed
    , gps.alt  //alt	int32_t mm	Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
    , gps.alt  //relative_alt	int32_t	mm	Altitude above home
    , gps.sog/10 * cos(gps.cog/1000) //vx	int16_t	cm/s	Ground X Speed (Latitude, positive north)
    , gps.sog/10 * sin(gps.cog/1000) //vy	int16_t	cm/s	Ground Y Speed (Longitude, positive east)
    , gps.veld/10 //vz	int16_t	cm/s	Ground Z Speed (Altitude, positive down)
    , gps.cog/10 //hdg	uint16_t	cdeg	Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
  );
  return telem_send(&msg);
  */
}





//start sending param value list
void RcinMavlink::telem_param_value_enable() {
  telem_param_list_index = 0;
  telem_sched[0].interval_ms = 10; //enable scheduler for param_value
}

bool RcinMavlink::telem_param_value(uint16_t param_index) {
  uint16_t param_count = cfg.valueCount();
  if(param_index < param_count) return true; //return true (i.e. command executed), even if nothing was send...
  
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