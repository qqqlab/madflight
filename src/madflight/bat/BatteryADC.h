/*=================================================================================================
ADC Battery Monitor

Need to have at least HW_PIN_BAT_V or HW_PIN_BAT_I defined before including this header

Also needs cfg for:
cfg.bat_cal_v //BatteryADC voltage conversion factor, set this to 1 and enable print_bat(), then enter here: Actual Volt / bat_v ADC reading (for example: 8.04/13951 -> set BAT_CAL_V 0.0057630 )
cfg.bat_cal_i //BatteryADC current conversion factor, set this to 1 and enable print_bat(), then enter here: Actual Amperes / bat_i ADC reading (for example: 1.0/847 --> set BAT_CAL_I 0.0011806 )

=================================================================================================*/

#pragma once

#include "bat_interface.h"
#include "../cfg/cfg_interface.h"

class BatteryADC: public Battery {
    public:
        float factor_v;// = 8.04/13951; //voltage conversion factor, set this to 1 and enable print_bat(), then enter here: Actual Volt / bat_v ADC reading
        float factor_i;// = 1.0/847; //current conversion factor, set this to 1 and enable print_bat(), then enter here: Actual Amperes / bat_i ADC reading
        uint32_t interval_us = 10000; //update interval in us

    void begin(MF_I2C *i2c, int8_t i2c_adr) override {
       (void)i2c; (void)i2c_adr; //does not use i2c
        i = 0;
        v = 0;
        mah = 0;
        wh = 0;
        if(cfg.pin_bat_v >= 0) {
          pinMode((int)cfg.pin_bat_v, INPUT);
        }
        if(cfg.bat_cal_v >= 0) {
          pinMode((int)cfg.bat_cal_v, INPUT);
        }
        analogReadResolution(16);
    }

    //returns true if battery was updated
    bool update() override {
        static uint32_t ts = micros();
        uint32_t now = micros();
        if(now - ts >= interval_us) {
            uint32_t dt = now - ts;
            float dt_h = dt / 3600e6;
            ts = now;
            if(cfg.pin_bat_v >= 0) {
               v = cfg.bat_cal_v * analogRead((int)cfg.pin_bat_v);
            }
            if(cfg.bat_cal_v >= 0) {
               i = cfg.bat_cal_v * analogRead((int)cfg.pin_bat_i);
            }
            w = v * i;
            mah += i * dt_h * 1000;
            wh += w * dt_h;
            return true;
        }
        return false;
    }
};
