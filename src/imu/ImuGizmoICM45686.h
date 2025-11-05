//#pragma once
#ifndef MF_ICM45686_H
#define MF_ICM45686_H

#include "./imu.h"
#include "./ICM45686/ICM45686.h"

class Invensensev3_Interface {
    public:
      int freqSlow;
      int freqFast;
      
      virtual void setFreq(int freq) = 0;
      
      inline void setFreqSlow() {
        setFreq(freqSlow);
      }
  
      inline void setFreqFast() {
        setFreq(freqFast);
      }
  };

#define INV3_SPI_FREQ_SLOW 1000000
#define INV3_SPI_FREQ_FAST 20000000

//================================================================
// SPI
//================================================================
class Invensensev3_InterfaceSPI : public Invensensev3_Interface {
    public:

    uint8_t _spi_cs;
    SPIClass * _spi;
    int _freq;

    Invensensev3_InterfaceSPI(SPIClass *spi, uint8_t cs) {
        _spi = spi; 
        _spi_cs = cs;
        freqSlow = INV3_SPI_FREQ_SLOW;
        freqFast = INV3_SPI_FREQ_FAST;
        setFreq(freqSlow);
    }
  
    void setFreq(int freq) {
        _freq = freq;
    }
};

class ImuGizmoICM45686 : public ImuGizmo {
private:
    ICM456xx _wrapped_imu;

    //raw measurements in NED frame
    int32_t rawa[3]; //accelerometer
    int32_t rawg[3]; //gyroscope
    int32_t rawm[3]; //magnetometer
    int16_t rawt; //temperature

    int _rate_hz = 100;

    float acc_multiplier = 1.0;
    float gyro_multiplier = 1.0;

    uint8_t fifo_watermark_threshold = 1; // Watermark threshold value



    uint16_t _convertSamplingRateHz(uint16_t freq) {
        uint16_t ret = 100;
        if (freq >= 6400) ret = 6400;
        else if (freq >= 3200) ret = 3200;
        else if (freq >= 1600) ret = 1600;
        else if (freq >= 800) ret = 800;
        else if (freq >= 400) ret = 400;
        else if (freq >= 200) ret = 200;
        else if (freq >= 100) ret = 100;
        else if (freq >= 50) ret = 50;
        else if (freq >= 25) ret = 25;
        else if (freq >= 12) ret = 12;
        else if (freq >= 6) ret = 6;
        else if (freq >= 3) ret = 3;
        else if (freq >= 1) ret = 1;
        return ret;
    }

public:
    ImuGizmoICM45686(Invensensev3_InterfaceSPI *iface)
        : _wrapped_imu( *(iface->_spi), iface->_spi_cs, iface->_freq) {}

    // Converting raw values to physical values
    // Madflight:
    // Body frame is NED: x-axis North(front), y-axis East(right), z-axis Down
    // ---
    // float ax = 0; //"North" acceleration in G
    // float gx = 0; //"North" rotation speed in deg/s


    int begin(int gyro_scale_dps, int acc_scale_g, int rate_hz) {
        (void)gyro_scale_dps; //20bit always uses 4000dps scale
        (void)acc_scale_g;    //20bit always uses 32G scale
        int status =  _wrapped_imu.begin();
        if (status != 0) {
            Serial.println("IMU: ICM45686 - initialization unsuccessful");
            Serial.println("Check IMU wiring or try cycling power");
            Serial.print("Status: ");
            Serial.println(status);
            while(1) { };
            return status;
        } else {
            Serial.println("IMU: ICM45686 - initialization successful");
        }

        uint16_t adjusted_sample_rate = _convertSamplingRateHz(rate_hz);
        set_rate(adjusted_sample_rate);

        _wrapped_imu.startAccel(adjusted_sample_rate, 32); //32G full scale (the only scale for 20 bit operation)
        acc_multiplier = 1.0 / 16384; //scale for 20 bit operation: 16384 LSB/g

        _wrapped_imu.startGyro(adjusted_sample_rate, 4000); //4000dps full scale (the only scale for 20 bit operation)
        gyro_multiplier = 1.0 / 131.1; //scale for 20 bit operation: 131.1 LSB/dps

        // Wait IMU to start
        delay(100);
        _wrapped_imu.enableFifoInterrupt(fifo_watermark_threshold);

        return status;
    }

    int get_rate() {
        return _rate_hz;
    }

    void set_rate(int rate) {
        _rate_hz = rate;
    }

    // Get sensor data in NED frame
    // x=North (forward), y=East (right), z=Down 
    // acc: gravitation force is positive in axis direction
    // gyro: direction of positive rotation by right hand rule, i.e. positive is: yaw right, roll right, pitch up
    void getMotion6NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
        read6();
        // FIXME: where can we use/report the temp data which was read?
        // sensor orientation for acc/gyro is NWU
        // acc result: gravitation force is positive in axis direction
        // ICM-45686 Datasheet:
        //   When the device is placed on a flat surface, it will measure 0g on the X- and Y- axes and +1g on the Z-axis.
        //   The accelerometers’ scale factor is calibrated at the factory and is nominally independent of supply voltage. The fullscale range of the digital output can be adjusted to ±2g, ±4g, ±8g, ±16g and ±32g.
        *ax = rawa[0] * -acc_multiplier; //-N = -N : FIXME: not sure why we have minus here, need to test (code was based on MPU driver)
        *ay = rawa[1] * acc_multiplier;  //-E =  W
        *az = rawa[2] * acc_multiplier;  //-D =  U
        // expected gyro result: direction of positive rotation by right hand rule, i.e. positive is: roll right (X), pitch up (Y), yaw right (Z)
        // gyro sensor positive:  X (roll right), Y (pitch down), Z (yaw left)
        // ICM-45686 Datasheet: "detects rotation about the X-, Y-, and Z- Axes"
        *gx = rawg[0] * gyro_multiplier; //N =  N
        *gy = rawg[1] * -gyro_multiplier; //E = -W
        *gz = rawg[2] * -gyro_multiplier; //D = -U
    }

    // FIXME: implement magnetometer support?
    // P.S. Orientation of axes seem same as for MPU-xxxx, and is converted in getMotion6NED() function
    void read6() {
        inv_imu_fifo_data_t imu_data;
        // FIXME: this might yield multiple samples if fifo_watermark_threshold>1 and there's imu_data.byte_16.timestamp !!!
        _wrapped_imu.getDataFromFifo(imu_data);
        rawa[0] = imu_data.byte_20.accel_data[0];
        rawa[1] = imu_data.byte_20.accel_data[1];
        rawa[2] = imu_data.byte_20.accel_data[2];

        rawg[0] = imu_data.byte_20.gyro_data[0];
        rawg[1] = imu_data.byte_20.gyro_data[1];
        rawg[2] = imu_data.byte_20.gyro_data[2];
        // Temperature in Degrees Centigrade = (TEMP_DATA / 128) + 25
        rawt = imu_data.byte_20.temp_data;
    }
};

#endif // MF_ICM45686_H
