/*==========================================================================================
MIT License

Copyright (c) 2026 https://madflight.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
===========================================================================================*/

// Driver for TDK Invensense MPU6000/6050/6500/9150/9250

//Datasheet: spi clock up to 1MHz for register operations, up to 20MHz allowed for reading data.
#define MPU_SPI_FREQ_SLOW 1000000
#define MPU_SPI_FREQ_FAST 20000000

//Datasheet: i2c clock up to 400kHz
#define MPU_I2C_FREQ_SLOW 400000
#define MPU_I2C_FREQ_FAST 1000000 //2.5 times overclocking

#include "MPUXXXX.h"
#include "MPU_regs.h"

//return 0 on success, positive on error, negative on warning
//Actual sample rate is set to the max possible rate smaller than/equal to the requested rate. 
//I.e. 99999..1000->1000, 999..500->500, 499..333->333, 332..250->250, etc
int MPUXXXX::begin(MPU_Type type, SensorDevice *dev, int gyro_scale_dps, int acc_scale_g, int rate_hz) {
    _type = type;
    _dev = dev;

    //set interface frequencies
    if(_dev->isSPI()) {
        freq_slow = MPU_SPI_FREQ_SLOW;
        freq_fast = MPU_SPI_FREQ_FAST;
    }else{
        freq_slow = MPU_I2C_FREQ_SLOW;
        freq_fast = MPU_I2C_FREQ_FAST;
    }
    
    //start interface slow
    _dev->setFreq(freq_slow);

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
    if(_type == MPU9250 && wai != 0x71 && wai != 0x73) {  // MPU9250 -> 0x71, MPU9255 -> 0x73
        if(wai == 0x70) {
            Serial.printf("WARNING: MPU9250 whoami mismatch, got:0x%02X expected:0x71 - this is probably a relabelled MPU6500\n",wai);
        }else{
            Serial.printf("WARNING: MPU9250 whoami mismatch, got:0x%02X expected:0x71\n",wai);
        }
    }

    //setup auto type
    switch(wai) {
        case 0x68: //MPU6000, MPU6050, MPU9150
            _type = MPUXXXX::MPU_Type::MPU6000; 
            break;
        case 0x70: //MPU6500
            _type = MPUXXXX::MPU_Type::MPU6500; 
            break; 
        case 0x71: //MPU9250
        case 0x73: //MPU9255
            _type = MPUXXXX::MPU_Type::MPU9250; 
            break;
        default:   
            return -99; //autodetect error
    }

    //config
    _dev->writeReg(MPUREG_PWR_MGMT_1, BIT_H_RESET);        // Reset
    delay(20);
    _dev->writeReg(MPUREG_PWR_MGMT_1, 0x01);               // Clock Source XGyro
    _dev->writeReg(MPUREG_PWR_MGMT_2, 0x00);               // Enable Acc & Gyro
    _dev->writeReg(MPUREG_CONFIG, BITS_DLPF_CFG_188HZ);    // Use DLPF set Gyroscope bandwidth 184Hz, acc bandwidth 188Hz

    //sample rate: 
    if(rate_hz > 1000) {
        rate_hz = 1000;
    }else if(rate_hz < 1000) {
        int div = 1000 / (rate_hz + 1);
    if(div > 255) div = 255;
        rate_hz = 1000 / (div + 1);
        _dev->writeReg(MPUREG_SMPLRT_DIV, div);
    }
    _rate_hz = rate_hz;

    //set scale
    set_acc_resolution(); //do this first, then scale
    set_gyro_scale_dps(gyro_scale_dps);
    set_acc_scale_g(acc_scale_g);

    //enable 50us data ready pulse on int pin
    _dev->writeReg(MPUREG_INT_PIN_CFG, 0x00);
    _dev->writeReg(MPUREG_INT_ENABLE, 0x01);

    //MPU9150: enable AK8975 magnetometer
    if(_type == MPU9150) {
        mag9150 = new AK8975(_dev);
        int rv = mag9150->begin();
        //multipliers (should be positive) - sensor orientation for mag is WND
        mag_multiplier[1] = mag9150->mag_multiplier[0]; //E = W (sign is set in read6()/read9())
        mag_multiplier[0] = mag9150->mag_multiplier[1]; //N = N
        mag_multiplier[2] = mag9150->mag_multiplier[2]; //D = D
        return rv;
    }

    //MPU9150: enable AK8963 magnetometer
    if(_type == MPU9250) {
        mag9250 = new AK8963(_dev);
        int rv = mag9250->begin();
        //multipliers (should be positive) - sensor orientation for mag is WND
        mag_multiplier[1] = mag9250->mag_multiplier[0]; //E = W (sign is set in read6()/read9())
        mag_multiplier[0] = mag9250->mag_multiplier[1]; //N = N
        mag_multiplier[2] = mag9250->mag_multiplier[2]; //D = D
        return rv;
    }

    return 0;
}

int MPUXXXX::get_rate() {
    return _rate_hz;
}


// MPU6000 should return 0x68
// MPU6050 should return 0x68
// MPU6500 should return 0x70
// MPU9150 should return 0x68
// MPU9250 should return 0x71
// MPU9255 should return 0x73
unsigned int MPUXXXX::whoami()
{
    _dev->setFreq(freq_slow);
    return _dev->readReg(MPUREG_WHOAMI);
}

const char* MPUXXXX::type_name() {
    switch (_type) {
    case MPU6000: return "MPU6000";
    case MPU6050: return "MPU6050";
    case MPU6500: return "MPU6500";
    case MPU9150: return "MPU9150";
    case MPU9250: return "MPU9250";
    }
    return "UNKNOWN";
}

void MPUXXXX::set_acc_resolution() {
    if(_type == MPU6000 || _type == MPU6050) {
        uint8_t data[6] = {0};
        _dev->readRegs(MPUREG_XA_OFFS_H,data,6);
        rev1 = ((data[5]&1)<<2) | ((data[3]&1)<<1) | ((data[1]&1)<<0);
        rev2 = _dev->readReg(MPUREG_PRODUCT_ID) & 0x0F;
        //rev 0.4 and 1.x have half acc resolution
        acc_resolution = ( (rev1 == 0 && rev2 == 4) || (rev1 == 1) ? 16384 : 32786);

        Serial.printf("MPU60X0 revision:%d.%d\n",(int)rev1,(int)rev2);
    }else{
        acc_resolution = 32786;
    }
}

void MPUXXXX::set_acc_scale_g(int scale_in_g)
{
    _dev->setFreq(freq_slow);
    if(scale_in_g <= 2) {
        _dev->writeReg(MPUREG_ACCEL_CONFIG, BITS_FS_2G);
        acc_multiplier = 2.0 / acc_resolution;
    }else if(scale_in_g <= 4) { 
        _dev->writeReg(MPUREG_ACCEL_CONFIG, BITS_FS_4G);
        acc_multiplier = 4.0 / acc_resolution;
    }else if(scale_in_g <= 8) { 
        _dev->writeReg(MPUREG_ACCEL_CONFIG, BITS_FS_8G);
        acc_multiplier = 8.0 / acc_resolution;
    }else{ 
        _dev->writeReg(MPUREG_ACCEL_CONFIG, BITS_FS_16G);
        acc_multiplier = 16.0 / acc_resolution;
    }
}

void MPUXXXX::set_gyro_scale_dps(int scale_in_dps)
{
    _dev->setFreq(freq_slow);
    if(scale_in_dps <= 250) {
        _dev->writeReg(MPUREG_GYRO_CONFIG, BITS_FS_250DPS);
        gyro_multiplier = 250.0 / 32768.0;
    }else if(scale_in_dps <= 500) { 
        _dev->writeReg(MPUREG_GYRO_CONFIG, BITS_FS_500DPS);
        gyro_multiplier = 500.0 / 32768.0;
    }else if(scale_in_dps <= 1000) { 
        _dev->writeReg(MPUREG_GYRO_CONFIG, BITS_FS_1000DPS);
        gyro_multiplier = 1000.0 / 32768.0;
    }else{ 
        _dev->writeReg(MPUREG_GYRO_CONFIG, BITS_FS_2000DPS);
        gyro_multiplier = 2000.0 / 32768.0;
    }
}

float MPUXXXX::getTemperature() {
    return (((float)rawt-21)/333.87)+21;
}

//================================================================
// Read 6 value acc/gyro sensor data
//================================================================

void MPUXXXX::read6()
{
    uint8_t d[14]; //response is 14 bytes = 6 acc + 2 temp + 6 gyro
    _dev->setFreq(freq_fast);
    _dev->readRegs(MPUREG_ACCEL_XOUT_H, d, 14); 
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

//read sensor data (axis as defined by sensor)
void MPUXXXX::read9()
{
    uint8_t d[20]; //response is 21 bytes = 6 acc + 2 temp + 6 gyro + 6 mag + 1 magstatus (last byte not retrieved)
    _dev->setFreq(freq_fast);
    _dev->readRegs(MPUREG_ACCEL_XOUT_H, d, 20); 
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
        _dev->writeReg(MPUREG_ACCEL_XOUT_H+15,0xff);
    }

}
