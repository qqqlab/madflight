/*
  The MAVLink protocol code generator does its own alignment, so
  alignment cast warnings can be ignored
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"

#if defined(__GNUC__) && __GNUC__ >= 9
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#endif

#include "mavlink_c_library_v2/ardupilotmega/mavlink.h"


class RcinMavlink : public Rcin {
  public:
    void _setup() override;
    bool _update() override;
  private:
    uint16_t pwm_instance[18];
};


void RcinMavlink::_setup() {
  Serial.println("RCIN: RCIN_USE_MAVLINK");
  pwm = pwm_instance;
  rcin_Serial->begin(460800);
}

//process received mavlink data
bool RcinMavlink::_update() {
  int packetSize = rcin_Serial->available();
  if (!packetSize) return false;

  mavlink_message_t msg;
  mavlink_status_t status;
  for (int i = 0; i < packetSize; i++) {
    uint8_t c = rcin_Serial->read();
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
        default: {
          //Serial.print("Received message with ID %d\n",msg.msgid);
          break; 
        }
      }
    }
  }
  return false;
}