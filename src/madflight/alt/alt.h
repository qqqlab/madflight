#pragma once

//=================================================================================================
// ALT - Altitude Estimator
//=================================================================================================

class AltEst {
  public:
    virtual void setup(float alt) = 0; //setup with default parameters and initial altitude in [m]
    virtual void updateAccelUp(float a, uint32_t ts) = 0; //a: accel up in [m/s^2], ts: timestamp in [us]
    virtual void updateBarAlt(float alt, uint32_t ts) = 0; //alt: barometric altitude in [m], ts: timestamp in [us]
    virtual float getH() = 0; //altitude estimate in [m]
    virtual float getV() = 0; //vertical up speed (climb rate) estimate in [m/s]
    virtual void toString(char *s); //print state info to s (max 100 chars)
};

extern AltEst &alt;
