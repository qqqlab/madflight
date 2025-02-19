#ifndef MF_ICM45686_H
#define MF_ICM45686_H
#include "../../interface.h"
#include "ICM45686.h"


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
  

// it is much slower without FIFO!!!
#define MF_ICM45686_USE_IMU_FIFO 1

class MF_ICM45686 {

private:
    ICM456xx _wrapped_imu;

    //raw measurements in NED frame
    int16_t rawa[3]; //accelerometer
    int16_t rawg[3]; //gyroscope
    int16_t rawm[3]; //magnetometer
    int16_t rawt; //temperature


    int _rate_hz = 100;
    int _interrupt_pin = 0;

    // FIXME !!!!
    float acc_multiplier = 1.0;
    float gyro_multiplier = 1.0;

    // Note: this is needed because original ICM-45686 driver interface .enableFifoInterrupt() requires to pass a interrupt handler function
    static void fake_interrupt_handler() {
    }


    uint8_t fifo_watermark_threshold = 1; // Watermark threshold value


    int _enableDataReadyInterrupt() {
        // route UI data ready interrupt to INT1
        // FIXME: this takes interrupt handler as argument, which is not standard interface in Madflight
        _wrapped_imu.enableFifoInterrupt(_interrupt_pin, fake_interrupt_handler, fifo_watermark_threshold);
        return 0;
    }

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
    MF_ICM45686(uint8_t intPin, Invensensev3_InterfaceSPI *iface)
        : _wrapped_imu(
            *((arduino::SPIClass *)(iface->_spi)), 
            iface->_spi_cs,
            iface->_freq) {
                _interrupt_pin = intPin;
    }

    int begin(int gyro_scale_dps, int acc_scale_g, int rate_hz) {
        Serial.println("MF_ICM45686 :: initializing wrapped IMU ...");
        Serial.flush();
        int status =  _wrapped_imu.begin();
        Serial.println("MF_ICM45686 :: begin done");
        Serial.flush();
        if (status != 0) {
            Serial.println("MF_ICM45686 :: wrapped IMU initialization unsuccessful");
            Serial.println("Check IMU wiring or try cycling power");
            Serial.print("Status: ");
            Serial.println(status);
            while(1) { };
            return status;
        } else {
            Serial.println("MF_ICM45686 :: wrapped IMU initialization successful");
        }
        Serial.flush();
        uint16_t adjusted_sample_rate = _convertSamplingRateHz(rate_hz);
        set_rate(adjusted_sample_rate);
        // FIXME: use gyro_scale_dps & acc_scale_g !!!
        // Accel - Full Scale Range = 16G
        // Gyro - Full Scale Range = 2000 dps
        _wrapped_imu.startAccel(adjusted_sample_rate, 16);
        _wrapped_imu.startGyro(adjusted_sample_rate, 2000);
        // Wait IMU to start
        delay(100);
        _enableDataReadyInterrupt();
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
        // FIXME: read temp
        // FIXME: fix multipliers?
        // FIXME: do we need to fix directions?
        *ax = rawa[0] * acc_multiplier;
        *ay = rawa[1] * acc_multiplier;
        *az = rawa[2] * acc_multiplier;
        *gx = rawg[0] * gyro_multiplier;
        *gy = rawg[1] * gyro_multiplier;
        *gz = rawg[2] * gyro_multiplier;
    }

    // FIXME: implement magnetometer support?

    void read6() {        
        #ifdef MF_ICM45686_USE_IMU_FIFO
        inv_imu_fifo_data_t imu_data;
        // FIXME: this might yield multiple samples if fifo_watermark_threshold>1 and there's imu_data.byte_16.timestamp !!!
        // FIXME there's also values represented as int20_t , if extra precission is needed...
        _wrapped_imu.getDataFromFifo(imu_data);
        rawa[0] = imu_data.byte_16.accel_data[0];
        rawa[1] = imu_data.byte_16.accel_data[1];
        rawa[2] = imu_data.byte_16.accel_data[2];

        rawg[0] = imu_data.byte_16.gyro_data[0];
        rawg[1] = imu_data.byte_16.gyro_data[1];
        rawg[2] = imu_data.byte_16.gyro_data[2];
        rawt = imu_data.byte_16.temp_data;
        #else
        inv_imu_sensor_data_t imu_data;
        _wrapped_imu.getDataFromRegisters(imu_data);

        rawa[0] = imu_data.accel_data[0];
        rawa[1] = imu_data.accel_data[1];
        rawa[2] = imu_data.accel_data[2];

        rawg[0] = imu_data.gyro_data[0];
        rawg[1] = imu_data.gyro_data[1];
        rawg[2] = imu_data.gyro_data[2];
        rawt = imu_data.temp_data;
        #endif
    }


};

#endif // MF_ICM45686_H
