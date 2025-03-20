#pragma once

#include "bat.h"

class BatGizmoADC: public BatGizmo {
    public:
        BatConfig *config;
        BatState *state;
        uint32_t interval_us = 10000; //update interval in us

    BatGizmoADC(Bat *bat) {
        this->config = &(bat->config);
        this->state = (BatState*)bat;
        interval_us = 1000000 / config->sampleRate;
        state->i = 0;
        state->v = 0;
        state->mah = 0;
        state->wh = 0;
        state->ts = micros();
        if(config->adc_pin_v >= 0) {
          pinMode(config->adc_pin_v, INPUT);
        }
        if(config->adc_pin_i >= 0) {
          pinMode(config->adc_pin_i, INPUT);
        }
        analogReadResolution(16);
    }

    //returns true if battery was updated
    bool update() override {
        uint32_t now = micros();
        if(now - state->ts >= interval_us) {
            uint32_t dt = now - state->ts;
            float dt_h = dt / 3600e6;
            if(config->adc_pin_v >= 0) {
               state->v = config->adc_cal_v * analogRead(config->adc_pin_v);
            }
            if(config->adc_pin_i >= 0) {
               state->i = config->adc_cal_v * analogRead(config->adc_pin_i);
            }
            state->w = state->v * state->i;
            state->mah += state->i * dt_h * 1000;
            state->wh += state->w * dt_h;
            state->ts = now;
            return true;
        }
        return false;
    }
};
