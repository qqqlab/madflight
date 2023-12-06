//2023-11-02 Invensense MPU-9150 I2C library
//sampling rate acc+gyro 1000Hz, mag 100Hz
//DON'T access AK8963 after begin() - the mag will stop sending data...

#pragma once

#include "MPU_Interface.h"

/* ---- AK8975 Reg In MPU9150 ----------------------------------------------- */
#define AK8975_I2C_ADDR             0x0C
#define AK8975_Device_ID            0x48

// Read-only Reg
#define AK8975_WIA                  0x00
#define AK8975_INFO                 0x01
#define AK8975_ST1                  0x02
#define AK8975_HXL                  0x03
#define AK8975_HXH                  0x04
#define AK8975_HYL                  0x05
#define AK8975_HYH                  0x06
#define AK8975_HZL                  0x07
#define AK8975_HZH                  0x08
#define AK8975_ST2                  0x09
// Write/Read Reg
#define AK8975_CNTL                 0x0A
#define AK8975_RSV                  0x0B
#define AK8975_ASTC                 0x0C
#define AK8975_TS1                  0x0D
#define AK8975_TS2                  0x0E
#define AK8975_I2CDIS               0x0F
// Read-only Reg ( ROM )
#define AK8975_ASAX                 0x10
#define AK8975_ASAY                 0x11
#define AK8975_ASAZ                 0x12

#define AK8975_uT_per_LSB ((float)0.3f)


class MPU9150 : public MPU_Interface {
public:
    MPU9150(uint8_t low_pass_filter = BITS_DLPF_CFG_188HZ, uint8_t low_pass_filter_acc = BITS_DLPF_CFG_188HZ);

    //MPU9150
    bool begin();
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
 
    //AK8975 Magnetometer
    int AK8975_begin();
    uint8_t AK8975_whoami();
    void AK8975_getASA();
    int AK8975_ReadReg(uint8_t reg);
    bool AK8975_WriteReg(uint8_t reg, uint8_t data);
    float mag[3];
    float mag_multiplier[3];

private:
    uint8_t _low_pass_filter;
    uint8_t _low_pass_filter_acc;
};
