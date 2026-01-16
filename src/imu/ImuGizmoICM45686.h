#pragma once

#include "./imu.h"
#include "./ICM45686/ICM45686.h"

class ImuGizmoICM45686 : public ImuGizmo {
  private:
    ImuGizmoICM45686() {} //private constructor
    ImuState *state = nullptr;
    ICM45686 *sensor = nullptr;
    PWM *pwm_clkin = nullptr;

  public:
    ~ImuGizmoICM45686() {
        delete sensor;
        delete pwm_clkin;
    }

    static ImuGizmo* create(ImuConfig *config, ImuState *state) {
        if(!config || !state) return nullptr;
        if(!config->spi_bus || config->spi_cs < 0) return nullptr;

        bool use_clkin = (config->pin_clkin >= 0);
        PWM *pwm_clkin = nullptr;
        if(use_clkin) {
            //32 kHz output
            int freq = 32000;
            pwm_clkin = new PWM();
            pwm_clkin->begin(config->pin_clkin, freq, 0, 1000000.0/freq);
            pwm_clkin->writeFactor(0.5);
        }

        auto *sensor = new ICM45686();
        int rv = sensor->begin(config->spi_bus, config->spi_cs, config->sample_rate_requested, use_clkin);

        if(rv < 0) {
            delete sensor;
            Serial.printf("IMU: ICM45686 init failed, rv=%d\n", rv);
            return nullptr;
        }

        //create gizmo
        auto gizmo = new ImuGizmoICM45686();
        gizmo->state = state;
        gizmo->sensor = sensor;
        gizmo->pwm_clkin = pwm_clkin;
        //return config
        strncpy(config->name, "ICM45686", sizeof(config->name));
        config->sample_rate = sensor->sampleRateActual;
        Serial.printf("IMU: ICM45686 started\n");

        return gizmo;
    }

/* Get sensor data in NED frame
   x=North (forward), y=East (right), z=Down 
   acc: gravitation force is positive in axis direction: 
        [ax,ay,az] nose-down:[1,0,0], right-down:[0,1,0], level:[0,0,1]
   gyr: direction of positive rotation by right hand rule: 
        [gx,gy,gz] roll-right:[positive,0,0], pitch-up:[0,positive,0], yaw-right:[0,0,positive]

ICM45686 has NWU orientation

  * +--+         Y
    |  | --> X   ^   Z-up
    +--+         |
    
*/
  
  void getMotion6NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) override {
    sensor->read();
    *ax = -sensor->acc[0]; //-N = -N
    *ay =  sensor->acc[1]; //-E =  W
    *az =  sensor->acc[2]; //-D =  U
    *gx =  sensor->gyr[0]; // N =  N
    *gy = -sensor->gyr[1]; // E = -W
    *gz = -sensor->gyr[2]; // D = -U
  }
};
