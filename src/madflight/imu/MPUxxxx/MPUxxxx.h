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

    float accel[3];
    float gyro[3];
    float mag[3];
    float temperature;
    float acc_multiplier;
    float gyro_multiplier;

  private:
  
    MPU_Interface *_iface;
    MPU_Type _type;

  public:

    //some MPU6000/6050 revisions have half acc resolution
    uint8_t rev1 = 0;
    uint8_t rev2 = 0;
    int acc_resolution = 32786;

    AK8963 *mag9250; //magnetometer of MPU9250
    AK8975 *mag9150; //magnetometer of MPU9150
    float* mag_multiplier; //pointer to multipliers in magnetometer object

    MPUXXXX(MPU_Type type, MPU_Interface *iface) {
        _type = type;
        _iface = iface;
    }

    int get_rate() {
      return 1000; //actual data rate is always 1000
    }

    //return 0 on success, positive on error, negative on warning
    int begin(int gyro_scale_dps, int acc_scale_g, int rate_hz) {
      //actual data rate is always 1000
      (void)(rate_hz); //suppress compiler warnings
      
      //start interface
      _iface->begin();
      _iface->setFreqSlow();

      //config
      _iface->WriteReg(MPUREG_PWR_MGMT_1, BIT_H_RESET);        // Reset
      delay(20);
      _iface->WriteReg(MPUREG_PWR_MGMT_1, 0x01);               // Clock Source XGyro
      _iface->WriteReg(MPUREG_PWR_MGMT_2, 0x00);               // Enable Acc & Gyro
      _iface->WriteReg(MPUREG_CONFIG, BITS_DLPF_CFG_188HZ);    // Use DLPF set Gyroscope bandwidth 184Hz, acc bandwidth 188Hz

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
      if(_type==MPU6500 && wai != 0x70) {
        Serial.printf("WARNING: MPU6500 whoami mismatch, got:0x%02X expected:0x70\n",wai);
      }
      if(_type == MPU9150 && wai != 0x68) {
        Serial.printf("WARNING: MPU9150 whoami mismatch, got:0x%02X expected:0x68\n",wai);
      }  
      if(_type==MPU9250 && wai != 0x71) {
        if(wai == 0x70) {
            Serial.printf("WARNING: MPU9250 whoami mismatch, got:0x%02X expected:0x71 - this is probably a relabelled MPU6500\n",wai);
        }else{
            Serial.printf("WARNING: MPU9250 whoami mismatch, got:0x%02X expected:0x71\n",wai);
        }
      }

      //MPU9150: enable AK8975 magnetometer
      if(_type==MPU9150) {
        mag9150 = new AK8975(_iface);
        mag_multiplier = mag9150->mag_multiplier;
        return mag9150->begin();
      }

      //MPU9150: enable AK8963 magnetometer
      if(_type==MPU9250) {
        mag9250 = new AK8963(_iface);
        mag_multiplier = mag9250->mag_multiplier;
        return mag9250->begin();
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


    //================================================================
    // Read 6 value acc/gyro sensor data
    //================================================================

    //Get sensor data in NED frame
    //x=North (forward), y=East (right), z=Down 
    //acc: gravitation force is positive in axis direction
    //gyro: direction of positive rotation by right hand rule, i.e. positive is: yaw right, roll right, pitch up
    void getMotion6NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
    {
      read6();
      //sensor orientation for acc/gyro is NWU
      *ax = -accel[0];
      *ay = accel[1];
      *az = accel[2];
      *gx = gyro[0];
      *gy = -gyro[1];
      *gz = -gyro[2];
    }

    void read6()
    {
        uint8_t response[14];
        int16_t bit_data;
        float data;
        int i,pos;

        _iface->setFreqFast();
        _iface->ReadRegs(MPUREG_ACCEL_XOUT_H,response,14);
        // Get accelerometer value (6 bytes)
        pos = 0;
        for(i = 0; i < 3; i++) {
            bit_data = ((int16_t)response[pos]<<8) | response[pos+1];
            data = (float)bit_data;
            accel[i] = data * acc_multiplier;
            pos += 2;;
        }
        // Get temperature (2 bytes)
        bit_data = ((int16_t)response[pos]<<8) | response[pos+1];
        data = (float)bit_data;
        temperature = ((data-21)/333.87)+21;
        pos += 2;
        // Get gyroscope value (6 bytes)
        for(i=0; i < 3; i++) {
            bit_data = ((int16_t)response[pos]<<8) | response[pos+1];
            data = (float)bit_data;
            gyro[i] = data * gyro_multiplier;
            pos += 2;
        }
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
      //sensor orientation for acc/gyro is NWU
      *ax = -accel[0]; //-N
      *ay = accel[1];  //-E
      *az = accel[2];  //-D
      *gx = gyro[0];   //N
      *gy = -gyro[1];  //E
      *gz = -gyro[2];  //D
      //sensor orientation for mag is WND
      *mx = mag[1];    //N
      *my = -mag[0];   //E
      *mz = mag[2];    //D
    }

    //read sensor data (axis as defined by sensor)
    void read9()
    {
        uint8_t response[20];
        int16_t bit_data;
        float data;
        int i,pos;

        _iface->setFreqFast();
        _iface->ReadRegs(MPUREG_ACCEL_XOUT_H,response,20);
        // Get accelerometer value (6 bytes)
        pos = 0;
        for(i = 0; i < 3; i++) {
            bit_data = ((int16_t)response[pos]<<8) | response[pos+1];
            data = (float)bit_data;
            accel[i] = data * acc_multiplier;
            pos += 2;;
        }
        // Get temperature (2 bytes)
        bit_data = ((int16_t)response[pos]<<8) | response[pos+1];
        data = (float)bit_data;
        temperature = ((data-21)/333.87)+21;
        pos += 2;
        // Get gyroscope value (6 bytes)
        for(i=0; i < 3; i++) {
            bit_data = ((int16_t)response[pos]<<8) | response[pos+1];
            data = (float)bit_data;
            gyro[i] = data * gyro_multiplier;
            pos += 2;
        }
        // Get Magnetometer value (6 bytes)
        for(i=0; i < 3; i++) {
            bit_data = ((int16_t)response[pos+1]<<8) | response[pos];
            data = (float)bit_data;
            mag[i] = data * mag_multiplier[i];
            pos += 2;
        }
    }

};
