//2023-11-03 Invensense MPU-9250 SPI/I2C library

#pragma once

#include "MPU_Interface.h"

#define AK8963FASTMODE
#define AK8963_uT_per_LSB ((float)0.15f)

/* ---- AK8963 Reg In MPU9250 ----------------------------------------------- */
#define AK8963_I2C_ADDR             0x0c//0x18
#define AK8963_Device_ID            0x48

// Read-only Reg
#define AK8963_WIA                  0x00
#define AK8963_INFO                 0x01
#define AK8963_ST1                  0x02
#define AK8963_HXL                  0x03
#define AK8963_HXH                  0x04
#define AK8963_HYL                  0x05
#define AK8963_HYH                  0x06
#define AK8963_HZL                  0x07
#define AK8963_HZH                  0x08
#define AK8963_ST2                  0x09
// Write/Read Reg
#define AK8963_CNTL1                0x0A
#define AK8963_CNTL2                0x0B
#define AK8963_ASTC                 0x0C
#define AK8963_TS1                  0x0D
#define AK8963_TS2                  0x0E
#define AK8963_I2CDIS               0x0F
// Read-only Reg ( ROM )
#define AK8963_ASAX                 0x10
#define AK8963_ASAY                 0x11
#define AK8963_ASAZ                 0x12

class MPU9250 {
public:
    MPU9250(MPU_Interface *iface, uint8_t low_pass_filter = BITS_DLPF_CFG_188HZ, uint8_t low_pass_filter_acc = BITS_DLPF_CFG_188HZ);
    
    //MPU9250
    int begin();
    int set_acc_scale(int scale);
    int set_acc_scale_g(int scale_in_g);
    int set_gyro_scale(int scale);
    int set_gyro_scale_dps(int scale_in_dps);
    unsigned int whoami();
    void getMotion9NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz);
    void read();
    float accel[3];
    float gyro[3];
    float temperature;
    float acc_multiplier;
    float gyro_multiplier;

    //AK8963 Magnetometer
    int AK8963_begin();
    uint8_t AK8963_whoami();
    void AK8963_getASA();
    int AK8963_ReadReg(uint8_t reg);
    bool AK8963_WriteReg(uint8_t reg, uint8_t data);
    float mag[3];    
    float mag_multiplier[3];

private:
    void _init(uint8_t low_pass_filter, uint8_t low_pass_filter_acc);

    MPU_Interface *_iface;
    uint8_t _low_pass_filter;
    uint8_t _low_pass_filter_acc;
};
