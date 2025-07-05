#pragma once
#ifndef MF_ICM20948_H
#define MF_ICM20948_H

#include "./imu.h"

// Forward declaration to avoid including ICM_20948.h in header
class ICM_20948_SPI;

class ImuGizmoICM20948 : public ImuGizmo {
private:
    ICM_20948_SPI* _wrapped_imu;

    //raw measurements in NED frame
    int16_t rawa[3]; //accelerometer
    int16_t rawg[3]; //gyroscope
    int16_t rawm[3]; //magnetometer
    int16_t rawt; //temperature

    int _rate_hz = 5000000; // You can override the default SPI frequency

    int _interrupt_pin = 0;
    SPIClass * _spi;
    uint8_t _csPin;

    float acc_multiplier = 1.0;
    float gyro_multiplier = 1.0;

    static ImuGizmoICM20948* instance;

    uint16_t _convertAccelScale(uint16_t range);
    uint16_t _convertGyroScale(uint16_t range);

    // Converting raw values to physical values
    // Madflight:
    // Body frame is NED: x-axis North(front), y-axis East(right), z-axis Down
    // ---
    // float ax = 0; //"North" acceleration in G
    // float gx = 0; //"North" rotation speed in deg/s

    // in ICM-45686 driver https://github.com/tdk-invn-oss/motion.arduino.ICM45686/blob/main/examples/MicroROS_Publisher/MicroROS_Publisher.ino :
    // Converting raw values to physical values
    inline float_t _convert_accel(int16_t raw, uint16_t fs) {
        return (float)raw * fs / INT16_MAX;
    }

    // raw is in degrees per second (dps)
    inline float_t _convert_gyro(int16_t raw, uint16_t fs) {
        return ((float)raw * fs) / INT16_MAX;
    }

public:
    ImuGizmoICM20948(SPIClass *spi, const uint8_t csPin, const uint8_t intPin, const bool use_mag);
    ~ImuGizmoICM20948();

    int begin(int gyro_scale_dps, int acc_scale_g, int spi_rate_hz = 5000000);
    int get_rate() { return 225; /* Sparkfun driver sets 225 Hz sample rate */};

    void getMotion6NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);
    void getMotion9NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz);
    void get6DOF(float *q0, float *q1, float *q2, float *q3);
    void get9DOF(float *q0, float *q1, float *q2, float *q3);

    bool flipYaw() override { return true; }
};

#endif // MF_ICM20948_H
