#pragma once

#include "imu.h"
#include "MPUxxxx/MPU_interface.h"
#include "ImuGizmoICM45686.h"
#include "ImuGizmoICM426XX.h"

class ImuGizmoAuto : public ImuGizmo {
  private:
    ImuGizmoAuto() {} //private constructor

  public:
    static ImuGizmo* create(ImuConfig *config, ImuState *state) {
        if(!config || !state) return nullptr;

        //AUTO is currently only for SPI sensors
        if(!config->spi_bus || config->spi_cs < 0) return nullptr;

        MPU_Interface *dev = new MPU_InterfaceSPI(config->spi_bus, config->spi_cs);
        dev->setFreq(1000000); //low speed

        //create gizmo
        ImuGizmo* gizmo = nullptr;
        if(dev->readReg(0x72) == 0xE9) {
          Serial.println("IMU: AUTO detected ICM45686");
          gizmo = ImuGizmoICM45686::create(config, state);
        }else if(dev->readReg(0x75) == 0x47) {
          Serial.println("IMU: AUTO detected ICM42688P");
          gizmo = ImuGizmoICM426XX::create(config, state);
        }else if(dev->readReg(0x0F) == 0x70) {
          Serial.println("IMU: AUTO detected LSM6DSV (TODO)");
          //todo 
        }else {
          Serial.println("IMU: AUTO nothing detected");
        }

        delete dev;

        return gizmo;
    }

  bool update() override {
    return false;
  }

  void getMotion6NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) override {
    (void)ax;
    (void)ay;
    (void)az;
    (void)gx;
    (void)gy;
    (void)gz;
  }

  int begin(int gyro_scale_dps, int acc_scale_g, int rate_hz) {
    (void) gyro_scale_dps;
    (void) acc_scale_g;
    (void) rate_hz;
    return 0;
  }

  int get_rate() {
    return 0;
  }
};
