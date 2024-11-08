/*=================================================================================================
ADC Battery Monitor

Need to have at least HW_PIN_BAT_V or HW_PIN_BAT_I defined before including this header

Also needs cfg for:
cfg.bat_cal_v //BatteryADC voltage conversion factor, set this to 1 and enable print_bat(), then enter here: Actual Volt / bat_v ADC reading (for example: 8.04/13951 -> set bat_cal_v 0.0057630 )
cfg.bat_cal_i //BatteryADC current conversion factor, set this to 1 and enable print_bat(), then enter here: Actual Amperes / bat_i ADC reading (for example: 1.0/847 --> set bat_cal_i 0.0011806 )

=================================================================================================*/

#pragma once

#include "../interface.h"

class BatteryADC: public Battery {
    public:
        //float i;// = 0; //Battery current (A)
        //float v;// = 0; //battery voltage (V)
        //float mah;// = 0; //battery usage (Ah)
        //float wh;// = 0; //battery usage (Wh)
        float factor_v;// = 8.04/13951; //voltage conversion factor, set this to 1 and enable print_bat(), then enter here: Actual Volt / bat_v ADC reading
        float factor_i;// = 1.0/847; //current conversion factor, set this to 1 and enable print_bat(), then enter here: Actual Amperes / bat_i ADC reading
        uint32_t interval_us = 10000; //update interval in us

    void setup() {
        i = 0;
        v = 0;
        mah = 0;
        wh = 0;
        #ifdef HW_PIN_BAT_V
            pinMode(HW_PIN_BAT_V, INPUT);
        #endif
        #ifdef HW_PIN_BAT_I
            pinMode(HW_PIN_BAT_I, INPUT);
        #endif
        analogReadResolution(16);
    }

    //returns true if battery was updated
    bool update() {
        static uint32_t ts = micros();
        uint32_t now = micros();
        if(now - ts >= interval_us) {
            uint32_t dt = now - ts;
            float dt_h = dt / 3600e6;
            ts = now;
            #ifdef HW_PIN_BAT_V
                 v = cfg.bat_cal_v * analogRead(HW_PIN_BAT_V);
            #endif
            #ifdef HW_PIN_BAT_I
                 i = cfg.bat_cal_v * analogRead(HW_PIN_BAT_I);
            #endif
            w = v * i;
            mah += i * dt_h * 1000;
            wh += w * dt_h;
            return true;
        }
        return false;
    }
};
