#pragma once

#include "../common/MF_Serial.h"

class Rcin_interface {
  public:
    virtual void setup() = 0;
    virtual bool update() = 0; //returns true if channel pwm data was updated
    virtual bool connected() = 0;
    virtual void calibrate() = 0; //interactive calibration
    virtual bool telem_statustext(uint8_t severity, char *text) {(void)severity; (void)text; return true;} //send MAVLink status text

    uint16_t *pwm; //pwm channel data. values: 988-2012
    float throttle = 0; //throttle stick value 0.0 (zero throttle/stick back) to 1.0 (full throttle/stick forward)
    float roll = 0; //roll stick value -1.0 (left) to 1.0 (right)
    float pitch = 0; //pitch stick value -1.0 (pitch up/stick back) to 1.0 (pitch down/stick forward)
    float yaw = 0; //yaw stick value -1.0 (left) to 1.0 (right)
    float vspeed = 0; //vertical speed stick value -1.0 (descent/stick back) to 1.0 (ascent/stick forward)
    bool arm = false; //arm switch state
    uint8_t flightmode = 0; //flightmode 0 to 5
};

extern Rcin_interface &rcin;

extern MF_Serial *rcin_serial;
