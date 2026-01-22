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

/* Gizmo for VL53L3CX

Keep XSHUT pin pulled up

Max Range: 5 meter indoor, ca 1 m indoor
Min Range: 10 mm
Resolution: 1 mm
Rate: 30 measurements / second default (max 120)
I2C: max 400kHz

Sensor can track up to 4 objects, gizmo returns distance to closest object
Two pins are not used by this driver: interrupt pin and XSHUT pin.

Returns distance == -1 mm on bad measurement
*/

#pragma once

#define VL53L3CX_I2C_CLOCK 1000000 //datasheet max is 400k, but 1000k appears to work

#include "rdr.h"
#include "../hal/hal.h"
#include "VL53L3CX/vl53lx_class.h"

class RdrGizmoVL53L3CX: public RdrGizmo {
private:
  RdrState *state;
  VL53LX *sensor = nullptr;

  RdrGizmoVL53L3CX() {} //private constructor

public:
  static bool detect(RdrConfig *config) {
      //get i2c 
      MF_I2C* i2c = hal_get_i2c_bus(config->rdr_i2c_bus);
      if(!i2c) return false;

      //set clock speed to 100k
      uint32_t clock = i2c->getClock();
      i2c->setClock(100000);

      //attempt to read device
      uint8_t adr = (config->rdr_i2c_adr <= 0 ? 0x29 : config->rdr_i2c_adr); //default address
      uint8_t tx[] = {0x01, 0x0F};
      uint8_t rx[2] = {};
      i2c->transceive(adr, tx, 2, rx, 2, true);

      //restore clock
      i2c->setClock(clock);

      return ((rx[0] == 0xEA) && (rx[1] == 0xAA));
  }

  static RdrGizmoVL53L3CX* create(RdrConfig *config, RdrState *state) {
      //get i2c bus
      MF_I2C* i2c = hal_get_i2c_bus(config->rdr_i2c_bus);
      if(!i2c) return nullptr;
      i2c->setClockMax(VL53L3CX_I2C_CLOCK);

      //set default address
      if(config->rdr_i2c_adr <= 0) config->rdr_i2c_adr = 0x29;

      //detect
      if(!detect(config)) {
        Serial.println("RDR: WARNING - VL53L3CX not detected");
      }

      //configure sensor
      VL53LX_Error error = VL53LX_ERROR_NONE;
      VL53LX *sensor = new VL53LX(i2c, -1); //XSHUT pin not used
      //sensor->VL53LX_Off(); //only needed when changing i2c address from default 0x29, needs XSHUT pins to shut down other VL53L sensors
      error = sensor->InitSensor(config->rdr_i2c_adr << 1); //driver expects adr * 2
      if(error != VL53LX_ERROR_NONE) {
        Serial.printf("RDR: VL53L3CX init failed. error=%d\n", (int)error);
        delete sensor;
        return nullptr;
      }
      sensor->VL53LX_ClearInterruptAndStartMeasurement();

      //setup gizmo
      auto gizmo = new RdrGizmoVL53L3CX();
      gizmo->state = state;
      gizmo->sensor = sensor;

      return gizmo;
    }

  bool update() override {
    int status = 0;
    uint8_t NewDataReady = 0;
    status = sensor->VL53LX_GetMeasurementDataReady(&NewDataReady);
    if (status != 0 || NewDataReady == 0) {
      return false;
    }

    VL53LX_MultiRangingData_t measurement;
    status = sensor->VL53LX_GetMultiRangingData(&measurement);
    sensor->VL53LX_ClearInterruptAndStartMeasurement();
    if (status != 0) {
      return false;
    }

    if(measurement.NumberOfObjectsFound > 0) {
      state->dist = measurement.RangeData[0].RangeMilliMeter;
    }else{
      state->dist = -1;
    }
    return true;
  }
};