#pragma once

#include "./imu.h"
#include "./MPUxxxx/MPU_interface.h"
#include "./ICM426XX/ICM426XX.h"

class ImuGizmoICM426XX : public ImuGizmo {
  private:
    ImuGizmoICM426XX() {} //private constructor
    ImuState *state = nullptr;
    ICM426XX *icm = nullptr;

  public:
    static ImuGizmo* create(ImuConfig *config, ImuState *state) {
      if(!config || !state) return nullptr;

      //create icm sensor
      MPU_Interface *dev = nullptr;
      if(config->spi_bus) {
        if(config->spi_cs >= 0) {
          dev = new MPU_InterfaceSPI(config->spi_bus, config->spi_cs);
        }
      }else if(config->i2c_bus) {
        dev = new MPU_InterfaceI2C(config->i2c_bus, config->i2c_adr);
      }
      ICM426XX *icm = ICM426XX::detect(dev);
      if(!icm) {
        delete dev;
        return nullptr;
      }

      //create gizmo
      auto gizmo = new ImuGizmoICM426XX();
      gizmo->state = state;
      gizmo->icm = icm;
      gizmo->has_mag = false;
      gizmo->uses_i2c = (config->spi_bus != nullptr);

      Serial.printf("IMU: detected:%s sample_rate:%d\n", icm->type_name(), icm->sampling_rate_hz);

      return gizmo;
    }

  bool update() override {
    return false;
  }


  void getMotion6NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) override {
    int16_t accgyr[7];
    icm->read(accgyr);
    *ax = accgyr[0] * icm->acc_scale;
    *ay = accgyr[1] * icm->acc_scale;
    *az = accgyr[2] * icm->acc_scale;
    *gx = accgyr[3] * icm->gyr_scale;
    *gy = accgyr[4] * icm->gyr_scale;
    *gz = accgyr[5] * icm->gyr_scale;
  }

  int begin(int gyro_scale_dps, int acc_scale_g, int rate_hz) {
    return 0;
  }

  int get_rate() {
    return icm->sampling_rate_hz;
  }
};
