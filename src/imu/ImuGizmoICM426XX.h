#pragma once

#include "./imu.h"
#include "./MPUxxxx/MPU_interface.h"
#include "./ICM426XX/ICM426XX.h"

class ImuGizmoICM426XX : public ImuGizmo {
  private:
    ImuGizmoICM426XX() {} //private constructor
    ImuState *state = nullptr;
    ICM426XX *sensor = nullptr;

  public:
    static ImuGizmo* create(ImuConfig *config, ImuState *state) {
      if(!config || !state) return nullptr;

      //create sensor sensor
      MPU_Interface *dev = nullptr;
      if(config->spi_bus) {
        if(config->spi_cs >= 0) {
          dev = new MPU_InterfaceSPI(config->spi_bus, config->spi_cs);
        }
      }else if(config->i2c_bus) {
        dev = new MPU_InterfaceI2C(config->i2c_bus, config->i2c_adr);
      }
      ICM426XX *sensor = ICM426XX::detect(dev, config->pin_clkin);
      if(!sensor) {
        delete dev;
        return nullptr;
      }

      //create gizmo
      auto gizmo = new ImuGizmoICM426XX();
      gizmo->state = state;
      gizmo->sensor = sensor;
      //return config
      strncpy(config->name, sensor->type_name(), sizeof(config->name));
      config->sample_rate = sensor->sampling_rate_hz;

      Serial.printf("IMU: detected:%s sample_rate:%d\n", sensor->type_name(), sensor->sampling_rate_hz);

      return gizmo;
    }

/* Get sensor data in NED frame
   x=North (forward), y=East (right), z=Down 
   acc: gravitation force is positive in axis direction: 
        [ax,ay,az] nose-down:[1,0,0], right-down:[0,1,0], level:[0,0,1]
   gyr: direction of positive rotation by right hand rule: 
        [gx,gy,gz] roll-right:[positive,0,0], pitch-up:[0,positive,0], yaw-right:[0,0,positive]

ICM42688 has NWU orientation

  * +--+         Y
    |  | --> X   ^   Z-up
    +--+         |
    
*/
  
  void getMotion6NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) override {
    int16_t accgyr[7];
    sensor->read(accgyr);
    *ax = -accgyr[0] * sensor->acc_scale; //-N = -N
    *ay =  accgyr[1] * sensor->acc_scale; //-E =  W
    *az =  accgyr[2] * sensor->acc_scale; //-D =  U
    *gx =  accgyr[3] * sensor->gyr_scale; // N =  N
    *gy = -accgyr[4] * sensor->gyr_scale; // E = -W
    *gz = -accgyr[5] * sensor->gyr_scale; // D = -U
  }
};
