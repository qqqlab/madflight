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
===========================================================================================*/

#pragma once

#include "../tbx/tbx.h" //MF_Schedule, RuntimeTrace, MsgBroker
#include "../cfg/cfg.h"

struct PidStatePID_s {
    float p = 0;
    float i = 0;
    float d = 0;
    float a = 0;
    float b = 0;
    float sum = 0;
    float setpoint = 0;
    float actual = 0;
};

struct __attribute__((aligned(4))) PidState {
  public:
    uint32_t ts = 0;  // Timestamp in [us]
 
    PidStatePID_s roll;
    PidStatePID_s pitch;
    PidStatePID_s yaw;
    PidStatePID_s throttle;
};

class PidGizmo {
  public:
    virtual ~PidGizmo() {}
    virtual const char* name() = 0;
    virtual void setup() = 0; // initial setup
    virtual void load_param() = 0; // (re-)load parameters
    virtual FlightMode set_flightmode(FlightMode fm) = 0; //set flightmode, returns actual fm set which could be differnt than requested
    virtual void controller() = 0; // execute controller

  protected:
    //load cfg parameter value, set default if value < 0
    float ifneg(float &value, float default_value) {
      if(value < 0) value = default_value; //set default cfg parameter value by reference
      return value;
    }
};

class Pid : public PidState {
  public:
    PidGizmo *gizmo = nullptr;
    MsgTopic<PidState> topic = MsgTopic<PidState>("pid", 10); //10-deep fifo

    void setup();

    void load_param() {
      if(!gizmo) return;
      gizmo->load_param();
    }

    FlightMode set_flightmode(FlightMode fm) {
      if(!gizmo) return FlightMode::mf_CUSTOM0;
      return gizmo->set_flightmode(fm);
    }

    void controller() {
      if(!gizmo) return;
      runtimeTrace.start();
      gizmo->controller();
      ts = micros();
      topic.publish(this);
      runtimeTrace.stop(true);
    }

  private:
    RuntimeTrace runtimeTrace = RuntimeTrace("PID");
};

extern Pid pid;
