// madflight https://github.com/qqqlab/madflight
// 2023-12-27 Invensense MPU6000/6050/6500/9150/9250 library
// sampling rate acc+gyro 1000Hz

#pragma once

#include "MPU_interface.h"
#include "AK8963.h"
#include "AK8975.h"

class MPUXXXX {

  public:

    enum MPU_Type {
        MPU6000,
        MPU6050,
        MPU6500,
        MPU9150,
        MPU9250
    };

    //raw measurements in NED frame
    int16_t rawa[3]; //accelerometer
    int16_t rawg[3]; //gyroscope
    int16_t rawm[3]; //magnetometer
    int16_t rawt; //temperature

    float acc_multiplier;
    float gyro_multiplier;
    float mag_multiplier[3]; //multipliers for magnetometer in NED frame

  private:
  
    MPU_Interface *_iface;
    MPU_Type _type;
    int _rate_hz;

  public:

    //some MPU6000/6050 revisions have half acc resolution
    uint8_t rev1 = 0;
    uint8_t rev2 = 0;
    int acc_resolution = 32786;

    AK8963 *mag9250; //magnetometer of MPU9250
    AK8975 *mag9150; //magnetometer of MPU9150

    MPUXXXX(MPU_Type type, MPU_Interface *iface) {
        _type = type;
        _iface = iface;
    }

    int get_rate() {
      return _rate_hz;
    }

    //return 0 on success, positive on error, negative on warning
    //Actual sample rate is set to the max possible rate smaller than/equal to the requested rate. 
    //I.e. 99999..1000->1000, 999..500->500, 499..333->333, 332..250->250, etc
    int begin(int gyro_scale_dps, int acc_scale_g, int rate_hz) {
      //start interface
      _iface->begin();
      _iface->setFreqSlow();

      //config
      _iface->WriteReg(MPUREG_PWR_MGMT_1, BIT_H_RESET);        // Reset
      delay(20);
      _iface->WriteReg(MPUREG_PWR_MGMT_1, 0x01);               // Clock Source XGyro
      _iface->WriteReg(MPUREG_PWR_MGMT_2, 0x00);               // Enable Acc & Gyro
      _iface->WriteReg(MPUREG_CONFIG, BITS_DLPF_CFG_188HZ);    // Use DLPF set Gyroscope bandwidth 184Hz, acc bandwidth 188Hz

      //sample rate: 
      if(rate_hz > 1000) {
        rate_hz = 1000;
      }else if(rate_hz < 1000) {
        int div = 1000 / (rate_hz + 1);
        if(div > 255) div = 255;
        rate_hz = 1000 / (div + 1);
        _iface->WriteReg(MPUREG_SMPLRT_DIV, div);
      }
      _rate_hz = rate_hz;

      //set scale
      set_acc_resolution(); //do this first, then scale
      set_gyro_scale_dps(gyro_scale_dps);
      set_acc_scale_g(acc_scale_g);

      //enable 50us data ready pulse on int pin
      _iface->WriteReg(MPUREG_INT_PIN_CFG, 0x00);
      _iface->WriteReg(MPUREG_INT_ENABLE, 0x01);

      //check whoami
      int wai = whoami();
      if(_type == MPU6000 && wai != 0x68) {
        Serial.printf("WARNING: MPU6000 whoami mismatch, got:0x%02X expected:0x68\n",wai);
      }  
      if(_type == MPU6050 && wai != 0x68) {
        Serial.printf("WARNING: MPU6050 whoami mismatch, got:0x%02X expected:0x68\n",wai);
      }
      if(_type == MPU6500 && wai != 0x70) {
        Serial.printf("WARNING: MPU6500 whoami mismatch, got:0x%02X expected:0x70\n",wai);
      }
      if(_type == MPU9150 && wai != 0x68) {
        Serial.printf("WARNING: MPU9150 whoami mismatch, got:0x%02X expected:0x68\n",wai);
      }  
      if(_type == MPU9250 && wai != 0x71) {
        if(wai == 0x70) {
            Serial.printf("WARNING: MPU9250 whoami mismatch, got:0x%02X expected:0x71 - this is probably a relabelled MPU6500\n",wai);
        }else{
            Serial.printf("WARNING: MPU9250 whoami mismatch, got:0x%02X expected:0x71\n",wai);
        }
      }

      //MPU9150: enable AK8975 magnetometer
      if(_type==MPU9150) {
        mag9150 = new AK8975(_iface);
        int rv = mag9150->begin();
        //multipliers (should be positive) - sensor orientation for mag is WND
        mag_multiplier[1] = mag9150->mag_multiplier[0]; //E = W (sign is set in read6()/read9())
        mag_multiplier[0] = mag9150->mag_multiplier[1]; //N = N
        mag_multiplier[2] = mag9150->mag_multiplier[2]; //D = D
        return rv;
      }

      //MPU9150: enable AK8963 magnetometer
      if(_type==MPU9250) {
        mag9250 = new AK8963(_iface);
        int rv = mag9250->begin();
        //multipliers (should be positive) - sensor orientation for mag is WND
        mag_multiplier[1] = mag9250->mag_multiplier[0]; //E = W (sign is set in read6()/read9())
        mag_multiplier[0] = mag9250->mag_multiplier[1]; //N = N
        mag_multiplier[2] = mag9250->mag_multiplier[2]; //D = D
        return rv;
      }

      return 0;
    }

    // MPU6000 should return 0x68
    // MPU6050 should return 0x68
    // MPU6500 should return 0x70
    // MPU9150 should return 0x68
    // MPU9250 should return 0x71
    unsigned int whoami()
    {
        _iface->setFreqSlow();
        return _iface->ReadReg(MPUREG_WHOAMI);
    }

    void set_acc_resolution() {
        if(_type == MPU6000 || _type == MPU6050) {
            uint8_t data[6] = {0};
            _iface->ReadRegs(MPUREG_XA_OFFS_H,data,6);
            rev1 = ((data[5]&1)<<2) | ((data[3]&1)<<1) | ((data[1]&1)<<0);
            rev2 = _iface->ReadReg(MPUREG_PRODUCT_ID) & 0x0F;
            //rev 0.4 and 1.x have half acc resolution
            acc_resolution = ( (rev1 == 0 && rev2 == 4) || (rev1 == 1) ? 16384 : 32786);

            Serial.printf("MPU60X0 revision:%d.%d\n",(int)rev1,(int)rev2);
        }else{
           acc_resolution = 32786;
        }
    }

    void set_acc_scale_g(int scale_in_g)
    {
        _iface->setFreqSlow();
        if(scale_in_g <= 2) {
          _iface->WriteReg(MPUREG_ACCEL_CONFIG, BITS_FS_2G);
          acc_multiplier = 2.0 / acc_resolution;
        }else if(scale_in_g <= 4) { 
          _iface->WriteReg(MPUREG_ACCEL_CONFIG, BITS_FS_4G);
          acc_multiplier = 4.0 / acc_resolution;
        }else if(scale_in_g <= 8) { 
          _iface->WriteReg(MPUREG_ACCEL_CONFIG, BITS_FS_8G);
          acc_multiplier = 8.0 / acc_resolution;
        }else{ 
          _iface->WriteReg(MPUREG_ACCEL_CONFIG, BITS_FS_16G);
          acc_multiplier = 16.0 / acc_resolution;
        }
    }

    void set_gyro_scale_dps(int scale_in_dps)
    {
        _iface->setFreqSlow();
        if(scale_in_dps <= 250) {
          _iface->WriteReg(MPUREG_GYRO_CONFIG, BITS_FS_250DPS);
          gyro_multiplier = 250.0 / 32768.0;
        }else if(scale_in_dps <= 500) { 
          _iface->WriteReg(MPUREG_GYRO_CONFIG, BITS_FS_500DPS);
          gyro_multiplier = 500.0 / 32768.0;
        }else if(scale_in_dps <= 1000) { 
          _iface->WriteReg(MPUREG_GYRO_CONFIG, BITS_FS_1000DPS);
          gyro_multiplier = 1000.0 / 32768.0;
        }else{ 
          _iface->WriteReg(MPUREG_GYRO_CONFIG, BITS_FS_2000DPS);
          gyro_multiplier = 2000.0 / 32768.0;
        }
    }

    float getTemperature() {
      return (((float)rawt-21)/333.87)+21;
    }

    //================================================================
    // Read 6 value acc/gyro sensor data
    //================================================================

    //Get sensor data in NED frame
    //x=North (forward), y=East (right), z=Down 
    //acc: gravitation force is positive in axis direction
    //gyro: direction of positive rotation by right hand rule, i.e. positive is: roll right, pitch up, yaw right
    void getMotion6NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
    {
      read6();
      *ax = rawa[0] * acc_multiplier;
      *ay = rawa[1] * acc_multiplier;
      *az = rawa[2] * acc_multiplier;
      *gx = rawg[0] * gyro_multiplier;
      *gy = rawg[1] * gyro_multiplier;
      *gz = rawg[2] * gyro_multiplier;
    }

    void read6()
    {
        uint8_t d[14]; //response is 14 bytes = 6 acc + 2 temp + 6 gyro
        _iface->setFreqFast();
        _iface->ReadRegs(MPUREG_ACCEL_XOUT_H, d, 14); 
        // Get accelerometer (6 bytes) - sensor orientation for acc/gyro is NWU
        rawa[0] = -(int16_t)((d[0]<<8) | d[1]); //-N = -N
        rawa[1] =  (int16_t)((d[2]<<8) | d[3]); //-E =  W
        rawa[2] =  (int16_t)((d[4]<<8) | d[5]); //-D =  U
        // Get temperature (2 bytes) 
        rawt = (int16_t)((d[6]<<8) | d[7]);
        // Get gyroscope (6 bytes) - sensor orientation for acc/gyro is NWU
        rawg[0] =  (int16_t)((d[ 8]<<8) | d[ 9]); //N =  N
        rawg[1] = -(int16_t)((d[10]<<8) | d[11]); //E = -W
        rawg[2] = -(int16_t)((d[12]<<8) | d[13]); //D = -U
    }


    //================================================================
    // Read 9 value acc/gyro/mag sensor data
    //================================================================

    //Get sensor data in NED frame
    //x=North (forward), y=East (right), z=Down 
    //acc: gravitation force is positive in axis direction (sensor reports negative)
    //gyro: direction of positive rotation by right hand rule, i.e. positive is: yaw right, roll right, pitch up
    void getMotion9NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz)
    {
      read9();
      *ax = rawa[0] * acc_multiplier;
      *ay = rawa[1] * acc_multiplier;
      *az = rawa[2] * acc_multiplier;
      *gx = rawg[0] * gyro_multiplier;
      *gy = rawg[1] * gyro_multiplier;
      *gz = rawg[2] * gyro_multiplier;
      *mx = rawm[0] * mag_multiplier[0];
      *my = rawm[1] * mag_multiplier[1];
      *mz = rawm[2] * mag_multiplier[2];
    }

    //read sensor data (axis as defined by sensor)
    void read9()
    {
        uint8_t d[20]; //response is 21 bytes = 6 acc + 2 temp + 6 gyro + 6 mag + 1 magstatus (last byte not retrieved)
        _iface->setFreqFast();
        _iface->ReadRegs(MPUREG_ACCEL_XOUT_H, d, 20); 
        // Get accelerometer (6 bytes) - sensor orientation for acc/gyro is NWU
        rawa[0] = -(int16_t)((d[0]<<8) | d[1]); //-N = -N
        rawa[1] =  (int16_t)((d[2]<<8) | d[3]); //-E =  W
        rawa[2] =  (int16_t)((d[4]<<8) | d[5]); //-D =  U
        // Get temperature (2 bytes) 
        rawt = (int16_t)((d[6]<<8) | d[7]);
        // Get gyroscope (6 bytes) - sensor orientation for acc/gyro is NWU
        rawg[0] =  (int16_t)((d[ 8]<<8) | d[ 9]); //N =  N
        rawg[1] = -(int16_t)((d[10]<<8) | d[11]); //E = -W
        rawg[2] = -(int16_t)((d[12]<<8) | d[13]); //D = -U
        // Get Magnetometer (6 bytes) - sensor orientation for mag is WND - NOTE: swapped byte order
        rawm[1] = -(int16_t)(d[14] | (d[15]<<8)); //E = -W
        rawm[0] =  (int16_t)(d[16] | (d[17]<<8)); //N = N
        rawm[2] =  (int16_t)(d[18] | (d[19]<<8)); //D = D

        //this hack appears to help to get more reasonable mag values from MPU9150
        if(_type == MPU9150) {
          _iface->WriteReg(MPUREG_ACCEL_XOUT_H+15,0xff);
        }

    }
};
