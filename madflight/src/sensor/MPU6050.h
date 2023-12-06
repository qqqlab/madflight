//2023-11-02 Invensense MPU-6050 I2C library
//sampling rate acc+gyro 1000Hz

#pragma once

#include "MPU_Interface.h"

class MPU6050 : public MPU_Interface {
public:
    MPU6050(uint8_t low_pass_filter = BITS_DLPF_CFG_188HZ, uint8_t low_pass_filter_acc = BITS_DLPF_CFG_188HZ);

    //MPU6050
    bool begin();
    int set_acc_scale(int scale);
    int set_acc_scale_g(int scale_in_g);
    int set_gyro_scale(int scale);
    int set_gyro_scale_dps(int scale_in_dps); 
    unsigned int whoami();
    void getMotion6NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);
    void read();     
    float accel[3];
    float gyro[3];
    float temperature;
    float acc_multiplier;
    float gyro_multiplier;    

private:
    uint8_t _i2c_adr;
    uint8_t _low_pass_filter;
    uint8_t _low_pass_filter_acc;
};
