/*=================================================================================================
Each MAG_USE_xxx section in this file defines a specific Magnetometer class

The magnetometer sample rate is 100Hz

Body frame is NED: x-axis North(front), y-axis East(right), z-axis Down

Unit of Measure is uT (micro Tesla)
=================================================================================================*/

#define MAG_USE_NONE 1
#define MAG_USE_QMC5883L 2

#ifndef MAG_I2C_ADR
  #define MAG_I2C_ADR 0
#endif

#include "../interface.h"

/* INTERFACE
class Magnetometer {
  public:
    float x = 0; //"North" magnetic flux in uT
    float y = 0; //"East" magnetic flux in uT
    float z = 0; //"Down" magnetic flux in uT
    virtual bool installed() = 0; //returns true if a sensor is installed
    virtual int setup() = 0; //returns 0 on success
    bool update(); //returns true if values updated
  private:
    uint32_t mag_time = 0;
    virtual void _update() = 0;
};

extern Magnetometer &mag;
*/
bool Magnetometer::update() {
  if(micros() - mag_time < 10000) return false; //100 Hz sample rate
  mag_time = micros();
  _update();
  return true;
}


//=================================================================================================
// None or undefined
//=================================================================================================
#if MAG_USE == MAG_USE_NONE || !defined MAG_USE
class MagnetometerNone: public Magnetometer {
  public:
    bool installed() { return false;}
    int setup() { return 0; }
  private:
    void _update() {}
};

MagnetometerNone mag_instance;

//=================================================================================================
// QMC5883L
//=================================================================================================
#elif MAG_USE == MAG_USE_QMC5883L

#include "QMC5883L.h"
QMC5883L<HW_WIRETYPE> mag_QMC5883L(i2c);

class MagnetometerQMC5883L: public Magnetometer {
  public:
    bool installed() { return true;}
    int setup() {
      mag_QMC5883L.begin();
      Serial.printf("MAG_USE_QMC5883L detect=%d\n",mag_QMC5883L.detect());
      return 0;
    }
  private:
    void _update() {
      mag_QMC5883L.read_uT(&x, &y, &z);
    }
};

MagnetometerQMC5883L mag_instance;

//=================================================================================================
// Invalid value
//=================================================================================================
#else
  #error "invalid MAG_USE value"
#endif

Magnetometer &mag = mag_instance;