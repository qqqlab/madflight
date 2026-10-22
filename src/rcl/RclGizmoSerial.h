/*==========================================================================================
MIT License

Copyright (c) 2026 https://madflight.com

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
==========================================================================================*/

//==========================================================================================
// Serial RC control link
//
// Receives RC channel data over a UART from a ground station, a hand controller, or
// another microcontroller, and converts it into the standard RC channel PWM values used
// by the rest of madflight. Arming, flight mode selection and the radio failsafe all
// keep working exactly as they do with a normal radio receiver.
//
// Frame format (all multi byte fields little endian):
//
//   offset  size  field
//   0       1     0xA5        sync byte 1
//   1       1     0x5A        sync byte 2
//   2       1     command
//   3       2     payload length in bytes
//   5       N     payload
//   5+N     2     CRC16-CCITT (init 0xFFFF, poly 0x1021) over bytes [2 .. 4+N]
//
// Commands
//
//   0x01 SET_CHANNELS  payload: N * uint16 PWM in [us], channel 1 first, N <= 20
//                      The channels are used as-is, so the host decides which channel
//                      is the arm switch and which is the flight mode switch.
//
//   0x02 SET_STICKS    payload: 4 * float32 (roll, pitch, yaw, throttle)
//                              + uint8 arm + uint8 flight mode index
//                      roll  : -1.0 (left)  .. +1.0 (right)
//                      pitch : -1.0 (stick forward) .. +1.0 (stick back)
//                      yaw   : -1.0 (left)  .. +1.0 (right)
//                      throttle: 0.0 (idle) .. 1.0 (full)
//                      arm   : 0 = disarmed, 1 = armed
//                      flight mode index: 0..5
//                      The stick values are converted to PWM using the rcl_* parameters,
//                      so the arm switch and the flight mode switch keep working.
//
//   0x03 PING           payload: none. Use as a keepalive.
//
// The link is considered "connected" while frames keep arriving. RCL_TIMEOUT_US (3s)
// applies, so the host must send at least one frame every 3 seconds, 10Hz is typical.
//==========================================================================================

#pragma once

#include "rcl.h"
#include "../hal/MF_Serial.h"
#include "../tbx/tbx_crc.h"

#define RCL_SERIAL_SYNC1 0xA5
#define RCL_SERIAL_SYNC2 0x5A
#define RCL_SERIAL_MAX_PAYLOAD 64

//commands
#define RCL_SERIAL_CMD_SET_CHANNELS 0x01
#define RCL_SERIAL_CMD_SET_STICKS   0x02
#define RCL_SERIAL_CMD_PING          0x03

class RclGizmoSerial : public RclGizmo {
  private:
    MF_Serial *ser_bus = nullptr;
    uint16_t *pwm = nullptr;

    //parser state
    enum parse_state_enum {ST_SYNC1, ST_SYNC2, ST_CMD, ST_LEN_L, ST_LEN_H, ST_PAYLOAD, ST_CRC_L, ST_CRC_H};
    parse_state_enum state = ST_SYNC1;
    uint8_t frame[RCL_SERIAL_MAX_PAYLOAD + 8]; //cmd, len(2), payload, crc(2)
    uint16_t frame_len = 0;   //payload length
    uint16_t frame_pos = 0;   //bytes received in the payload
    uint8_t cmd = 0;

    //stick mode state
    float stick_roll = 0;
    float stick_pitch = 0;
    float stick_yaw = 0;
    float stick_throttle = 0;
    uint8_t stick_arm = 0;
    uint8_t stick_mode = 0;
    bool arm_prev = false;

    uint32_t frame_cnt = 0;
    uint32_t error_cnt = 0;

  public:
    RclGizmoSerial(MF_Serial *ser_bus, uint16_t *pwm) {
      this->ser_bus = ser_bus;
      this->pwm = pwm;

      //start with all channels centered
      for(int i = 0; i < RCL_MAX_CH; i++) pwm[i] = 1500;
      pwm[RCL_MAX_CH] = 1500; //unassigned channel
    }

    bool update() override {
      bool updated = false;
      while(ser_bus->available()) {
        int c = ser_bus->read();
        if(c < 0) break;
        if(_parse_byte((uint8_t)c)) updated = true;
      }
      return updated;
    }

    uint32_t frame_count() { return frame_cnt; }
    uint32_t error_count() { return error_cnt; }

  private:
    //returns true when a complete and valid frame was processed
    bool _parse_byte(uint8_t c) {
      switch(state) {
        case ST_SYNC1:
          if(c == RCL_SERIAL_SYNC1) state = ST_SYNC2;
          break;

        case ST_SYNC2:
          if(c == RCL_SERIAL_SYNC2) state = ST_CMD;
          else if(c != RCL_SERIAL_SYNC1) state = ST_SYNC1; //resync
          break;

        case ST_CMD:
          cmd = c;
          frame_len = 0;
          frame_pos = 0;
          state = ST_LEN_L;
          break;

        case ST_LEN_L:
          frame[1] = c;
          state = ST_LEN_H;
          break;

        case ST_LEN_H:
          frame[2] = c;
          frame_len = (uint16_t)frame[1] | ((uint16_t)frame[2] << 8);
          if(frame_len > RCL_SERIAL_MAX_PAYLOAD) {
            error_cnt++;
            state = ST_SYNC1;
            break;
          }
          frame_pos = 0;
          state = (frame_len == 0) ? ST_CRC_L : ST_PAYLOAD;
          break;

        case ST_PAYLOAD:
          frame[3 + frame_pos] = c;
          frame_pos++;
          if(frame_pos >= frame_len) state = ST_CRC_L;
          break;

        case ST_CRC_L:
          frame[3 + frame_len] = c;
          state = ST_CRC_H;
          break;

        case ST_CRC_H: {
          frame[4 + frame_len] = c;
          state = ST_SYNC1;
          //the CRC covers the command, the length and the payload
          uint16_t crc_calc = tbx_crc16(&frame[0], 3 + frame_len);
          uint16_t crc_recv = (uint16_t)frame[3 + frame_len] | ((uint16_t)frame[4 + frame_len] << 8);
          if(crc_calc != crc_recv) {
            error_cnt++;
            break;
          }
          frame[0] = cmd;
          frame_cnt++;
          _handle_frame();
          return true;
        }
      }
      return false;
    }

    void _handle_frame() {
      switch(cmd) {
        case RCL_SERIAL_CMD_SET_CHANNELS: {
          int n = frame_len / 2;
          if(n > RCL_MAX_CH) n = RCL_MAX_CH;
          for(int i = 0; i < n; i++) {
            pwm[i] = (uint16_t)frame[3 + i*2] | ((uint16_t)frame[4 + i*2] << 8);
          }
          break;
        }

        case RCL_SERIAL_CMD_SET_STICKS: {
          if(frame_len < 18) { error_cnt++; break; }
          stick_roll     = _getf32(3);
          stick_pitch    = _getf32(7);
          stick_yaw      = _getf32(11);
          stick_throttle = _getf32(15);
          stick_arm      = frame[19];
          stick_mode     = frame[20];
          _apply_sticks();
          break;
        }

        case RCL_SERIAL_CMD_PING:
          break;
      }
    }

    //convert the normalized stick values into PWM values and write them to the channels
    void _apply_sticks() {
      //roll
      _set_ch(cfg.rcl_rol_ch, _lerp_pwm(stick_roll, cfg.rcl_rol_left, cfg.rcl_rol_mid, cfg.rcl_rol_right));
      //pitch: +1.0 is stick back (pitch up), -1.0 is stick forward
      _set_ch(cfg.rcl_pit_ch, _lerp_pwm_rev(stick_pitch, cfg.rcl_pit_pull, cfg.rcl_pit_mid, cfg.rcl_pit_push));
      //yaw
      _set_ch(cfg.rcl_yaw_ch, _lerp_pwm(stick_yaw, cfg.rcl_yaw_left, cfg.rcl_yaw_mid, cfg.rcl_yaw_right));
      //throttle: inverse of the madflight throttle normalization
      {
        int lo = (cfg.rcl_thr_pull < cfg.rcl_thr_push) ? cfg.rcl_thr_pull : cfg.rcl_thr_push;
        int hi = (cfg.rcl_thr_pull < cfg.rcl_thr_push) ? cfg.rcl_thr_push : cfg.rcl_thr_pull;
        float t = constrain(stick_throttle, 0.0f, 1.0f);
        float tlow = lo + RCL_THROTTLE_DEADBAND;
        _set_ch(cfg.rcl_thr_ch, (uint16_t)(tlow + t * (hi - tlow)));
      }
      //flight mode switch
      {
        int spacing = (cfg.rcl_flt_max - cfg.rcl_flt_min) / 5;
        int idx = constrain((int)stick_mode, 0, 5);
        _set_ch(cfg.rcl_flt_ch, (uint16_t)(cfg.rcl_flt_min + idx * spacing));
      }
      //arm switch: madflight arms when the switch goes from low to high while the
      //throttle is at zero, so present the "disarmed" value for one frame first
      {
        int ch = cfg.rcl_arm_ch;
        if(stick_arm) {
          if(!arm_prev) {
            _set_ch(ch, (uint16_t)cfg.rcl_arm_max); //disarmed
          }else{
            _set_ch(ch, (uint16_t)cfg.rcl_arm_min); //armed
          }
        }else{
          _set_ch(ch, (uint16_t)cfg.rcl_arm_max); //disarmed
        }
        arm_prev = (stick_arm != 0);
      }
    }

    //write a PWM value to a 1-based channel number
    void _set_ch(int ch1based, uint16_t value) {
      if(ch1based < 1 || ch1based > RCL_MAX_CH) return;
      pwm[ch1based - 1] = value;
    }

    //v = -1.0 gives lo, v = 0.0 gives mid, v = +1.0 gives hi
    static uint16_t _lerp_pwm(float v, int lo, int mid, int hi) {
      if(v > 1.0f) v = 1.0f;
      if(v < -1.0f) v = -1.0f;
      if(v >= 0.0f) return (uint16_t)(mid + v * (hi - mid));
      return (uint16_t)(mid + v * (mid - lo));
    }

    //v = -1.0 gives hi, v = 0.0 gives mid, v = +1.0 gives lo
    static uint16_t _lerp_pwm_rev(float v, int lo, int mid, int hi) {
      if(v > 1.0f) v = 1.0f;
      if(v < -1.0f) v = -1.0f;
      if(v >= 0.0f) return (uint16_t)(mid + v * (lo - mid));
      return (uint16_t)(mid + v * (mid - hi));
    }

    float _getf32(uint16_t pos) {
      uint32_t v = (uint32_t)frame[pos] | ((uint32_t)frame[pos+1] << 8) |
                   ((uint32_t)frame[pos+2] << 16) | ((uint32_t)frame[pos+3] << 24);
      float f;
      memcpy(&f, &v, 4);
      return f;
    }
};
