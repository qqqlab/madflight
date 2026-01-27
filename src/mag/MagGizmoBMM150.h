#pragma once

#define BMM150_I2C_CLOCK 400000

#include "mag.h"
#include "../hal/hal.h"
#include "BMM150/BMM150.h"

class MagGizmoBMM150: public MagGizmo {
  protected:
    BMM150_I2C* sensor;
    MagState* state;

  public:
    const char* name() override {return "BMM150";}

    static MagGizmoBMM150* create(MagConfig *config, MagState *state) {
      //get i2c bus
      MF_I2C* i2c = config->i2c_bus;
      if(!i2c) return nullptr;
      i2c->setClockMax(BMM150_I2C_CLOCK);

      //set default address
      if(config->i2c_adr <= 0) config->i2c_adr = 0x10;

      //configure sensor
      BMM150_I2C *sensor = new BMM150_I2C(i2c, config->i2c_adr);
      if(!sensor->begin()) {
        Serial.printf("RDR: BMM150 init failed.\n");
        delete sensor;
        return nullptr;
      }

      /**!
       * Set sensor operation mode
       * opMode:
       *   BMM150_POWERMODE_NORMAL  // normal mode  Get geomagnetic data normally
       *   BMM150_POWERMODE_FORCED  // forced mode  Single measurement, the sensor restores to sleep mode when the measurement is done.
       *   BMM150_POWERMODE_SLEEP   // sleep mode   Users can visit all the registers, but can't measure geomagnetic data
       *   BMM150_POWERMODE_SUSPEND // suspend mode At the time the sensor cpu doesn't work and can't implement any operation.
       *                                            Users can only visit the content of the control register BMM150_REG_POWER_CONTROL
       */
      sensor->setOperationMode(BMM150_POWERMODE_NORMAL);

      /**!
       * Set preset mode, make it easier for users to configure sensor to get geomagnetic data
       * presetMode:
       *   BMM150_PRESETMODE_LOWPOWER      // Low power mode, get a small number of data and take the mean value.
       *   BMM150_PRESETMODE_REGULAR       // Regular mode, get a number of data and take the mean value.
       *   BMM150_PRESETMODE_ENHANCED      // Enhanced mode, get a large number of data and take the mean value.
       *   BMM150_PRESETMODE_HIGHACCURACY  // High accuracy mode, get a huge number of data and take the mean value.
       */
      sensor->setPresetMode(BMM150_PRESETMODE_REGULAR);

      /**!
       * Set the rate of obtaining geomagnetic data, the higher, the faster (without delay function)
       * rate:
       *   BMM150_DATA_RATE_02HZ
       *   BMM150_DATA_RATE_06HZ
       *   BMM150_DATA_RATE_08HZ
       *   BMM150_DATA_RATE_10HZ   (default rate)
       *   BMM150_DATA_RATE_15HZ
       *   BMM150_DATA_RATE_20HZ
       *   BMM150_DATA_RATE_25HZ
       *   BMM150_DATA_RATE_30HZ
       */
      sensor->setRate(BMM150_DATA_RATE_30HZ);

      /**!
       * Enable the measurement at x-axis, y-axis and z-axis, default to be enabled, no config required, the geomagnetic data at x, y and z will be incorrect when disabled.
       * Refer to setMeasurementXYZ() function in the .h file if you want to configure more parameters.
       */
      sensor->setMeasurementXYZ();

      //setup gizmo
      auto gizmo = new MagGizmoBMM150();
      gizmo->state = state;
      gizmo->sensor = sensor;
      return gizmo;
    }

    bool update(float *x, float *y, float *z) override {
      sBmm150MagData_t magData = sensor->getGeomagneticData();
      *x = magData.xx;
      *y = magData.yy;
      *z = magData.zz;
      return true; 
    }
};
