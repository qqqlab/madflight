//#pragma once
#ifndef MF_ICM20948_H
#define MF_ICM20948_H

#include "./imu.h"
#include <ICM_20948.h>

class ImuGizmoICM20948 : public ImuGizmo {
private:
    ICM_20948_SPI _wrapped_imu;

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

    // Note: this is needed because original ICM-20948 driver interface .enableFifoInterrupt() requires to pass a interrupt handler function
    static void fake_interrupt_handler() {
        if (instance) {
            instance->_wrapped_imu.getAGMT();
            Serial.print("Scaled. Acc (mg) [ ");
            Serial.print(instance->_wrapped_imu.accX());
            Serial.print(", ");
            Serial.print(instance->_wrapped_imu.accY());
            Serial.print(", ");
            Serial.print(instance->_wrapped_imu.accZ());
            Serial.print(" ], Gyr (DPS) [ ");
            Serial.print(instance->_wrapped_imu.gyrX());
            Serial.print(", ");
            Serial.print(instance->_wrapped_imu.gyrY());
            Serial.print(", ");
            Serial.print(instance->_wrapped_imu.gyrZ());
            Serial.print(" ], Mag (uT) [ ");
            Serial.print(instance->_wrapped_imu.magX());
            Serial.print(", ");
            Serial.print(instance->_wrapped_imu.magY());
            Serial.print(", ");
            Serial.print(instance->_wrapped_imu.magZ());
            Serial.print(" ], Tmp (C) [ ");
            Serial.print(instance->_wrapped_imu.temp());
            Serial.println(" ]");
            instance->_wrapped_imu.clearInterrupts();
        }
    }

    uint8_t fifo_watermark_threshold = 1; // Watermark threshold value

    int _enableDataReadyInterrupt() {
        // route UI data ready interrupt to INT1
        // FIXME: this takes interrupt handler as argument, which is not standard interface in Madflight
        instance = this;
        pinMode(_interrupt_pin, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(_interrupt_pin), fake_interrupt_handler, FALLING); // Set up a falling interrupt
        return 0;
    }

    uint16_t _convertAccelScale(uint16_t range) {
        // Accel: Supported full scale ranges are: 2, 4, 8, 16, 32 G (any other value defaults to 16 G).
        uint16_t ret = 16;
        if (range > 16) ret = 32;
        else if (range > 8) ret = 16;
        else if (range > 4) ret = 8;
        else if (range > 2) ret = 4;
        else if (range == 2) ret = 2;
        return ret;
    }

    uint16_t _convertGyroScale(uint16_t range) {
        // Gyro: Supported full scale ranges are: 16, 31, 62, 125, 250, 500, 1000, 2000, 4000 dps (any other value defaults to 2000 dps).
        uint16_t ret = 2000;
        if (range > 2000) ret = 4000;
        else if (range > 1000) ret = 2000;
        else if (range > 500) ret = 1000;
        else if (range > 250) ret = 500;
        else if (range > 125) ret = 250;
        else if (range > 62) ret = 125;
        else if (range > 31) ret = 62;
        else if (range > 16) ret = 31;
        else if (range == 16) ret = 16;
        return ret;
    }

public:
    ImuGizmoICM20948(SPIClass *spi, const uint8_t csPin, const uint8_t intPin)
        : _spi(spi), _csPin(csPin), _interrupt_pin(intPin) {
        has_mag = true;
        uses_i2c = false;
    }

    // Converting raw values to physical values
    // Madflight:
    // Body frame is NED: x-axis North(front), y-axis East(right), z-axis Down
    // ---
    // float ax = 0; //"North" acceleration in G
    // float gx = 0; //"North" rotation speed in deg/s

    // in ICM-45686 driver https://github.com/tdk-invn-oss/motion.arduino.ICM45686/blob/main/examples/MicroROS_Publisher/MicroROS_Publisher.ino :
    inline float_t _convert_accel(int16_t raw, uint16_t fs) {
        return (float)raw * fs / INT16_MAX;
    }

    // raw is in degrees per second (dps)
    inline float_t _convert_gyro(int16_t raw, uint16_t fs) {
        return ((float)raw * fs) / INT16_MAX;
    }
    

    int begin(int gyro_scale_dps, int acc_scale_g, int rate_hz) {
        Serial.println("IMU: ICM20948 - initializing wrapped IMU ...");
        Serial.flush();
        _rate_hz = rate_hz;
        //_enableDataReadyInterrupt();
        int status =  _wrapped_imu.begin(_csPin, *_spi, _rate_hz);
        Serial.println("MU: ICM20948 - begin done");
        Serial.flush();
        if (status != 0) {
            Serial.println("MU: ICM20948 - wrapped IMU initialization unsuccessful");
            Serial.println("Check IMU wiring or try cycling power");
            Serial.print("Status: ");
            Serial.println(status);
            while(1) { };
            return status;
        } else {
            Serial.println("IMU: ICM20948 - wrapped IMU initialization successful");
        }
        Serial.flush();

        // Here we are doing a SW reset to make sure the device starts in a known state
        _wrapped_imu.swReset();
        if (_wrapped_imu.status != ICM_20948_Stat_Ok)
        {
            Serial.print(F("Software Reset returned: "));
            Serial.println(_wrapped_imu.statusString());
        }
        delay(250);
        // Now wake the sensor up
        _wrapped_imu.sleep(false);
        _wrapped_imu.lowPower(false);

        // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

        // Set Gyro and Accelerometer to a particular sample mode
        // options: ICM_20948_Sample_Mode_Continuous
        //          ICM_20948_Sample_Mode_Cycled
        _wrapped_imu.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Cycled);
        if (_wrapped_imu.status != ICM_20948_Stat_Ok)
        {
            Serial.print(F("setSampleMode returned: "));
            Serial.println(_wrapped_imu.statusString());
        }
        
        // Set full scale ranges for both acc and gyr
        ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

        myFSS.a = gpm2; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                        // gpm2
                        // gpm4
                        // gpm8
                        // gpm16

        myFSS.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                            // dps250
                            // dps500
                            // dps1000
                            // dps2000

        _wrapped_imu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
        if (_wrapped_imu.status != ICM_20948_Stat_Ok)
        {
            Serial.print(F("setFullScale returned: "));
            Serial.println(_wrapped_imu.statusString());
        }

        // Set up Digital Low-Pass Filter configuration
        ICM_20948_dlpcfg_t myDLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors
        myDLPcfg.a = acc_d473bw_n499bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                        // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                        // acc_d111bw4_n136bw
                                        // acc_d50bw4_n68bw8
                                        // acc_d23bw9_n34bw4
                                        // acc_d11bw5_n17bw
                                        // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                        // acc_d473bw_n499bw

        myDLPcfg.g = gyr_d361bw4_n376bw5; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                            // gyr_d196bw6_n229bw8
                                            // gyr_d151bw8_n187bw6
                                            // gyr_d119bw5_n154bw3
                                            // gyr_d51bw2_n73bw3
                                            // gyr_d23bw9_n35bw9
                                            // gyr_d11bw6_n17bw8
                                            // gyr_d5bw7_n8bw9
                                            // gyr_d361bw4_n376bw5

        _wrapped_imu.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
        if (_wrapped_imu.status != ICM_20948_Stat_Ok)
        {
            Serial.print(F("setDLPcfg returned: "));
            Serial.println(_wrapped_imu.statusString());
        }

        // Choose whether or not to use DLPF
        // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
        ICM_20948_Status_e accDLPEnableStat = _wrapped_imu.enableDLPF(ICM_20948_Internal_Acc, true);
        ICM_20948_Status_e gyrDLPEnableStat = _wrapped_imu.enableDLPF(ICM_20948_Internal_Gyr, true);
        Serial.print(F("Enable DLPF for Accelerometer returned: "));
        Serial.println(_wrapped_imu.statusString(accDLPEnableStat));
        Serial.print(F("Enable DLPF for Gyroscope returned: "));
        Serial.println(_wrapped_imu.statusString(gyrDLPEnableStat));

        // Choose whether or not to start the magnetometer
        _wrapped_imu.startupMagnetometer();
        if (_wrapped_imu.status != ICM_20948_Stat_Ok)
        {
            Serial.print(F("startupMagnetometer returned: "));
            Serial.println(_wrapped_imu.statusString());
        }

        // Now we're going to set up interrupts. There are a lot of options, but for this test we're just configuring the interrupt pin and enabling interrupts to tell us when new data is ready
        /*
            ICM_20948_Status_e  cfgIntActiveLow         ( bool active_low );
            ICM_20948_Status_e  cfgIntOpenDrain         ( bool open_drain );
            ICM_20948_Status_e  cfgIntLatch             ( bool latching );                          // If not latching then the interrupt is a 50 us pulse

            ICM_20948_Status_e  cfgIntAnyReadToClear    ( bool enabled );                           // If enabled, *ANY* read will clear the INT_STATUS register. So if you have multiple interrupt sources enabled be sure to read INT_STATUS first

            ICM_20948_Status_e  cfgFsyncActiveLow       ( bool active_low );
            ICM_20948_Status_e  cfgFsyncIntMode         ( bool interrupt_mode );                    // Can ue FSYNC as an interrupt input that sets the I2C Master Status register's PASS_THROUGH bit

            ICM_20948_Status_e  intEnableI2C            ( bool enable );
            ICM_20948_Status_e  intEnableDMP            ( bool enable );
            ICM_20948_Status_e  intEnablePLL            ( bool enable );
            ICM_20948_Status_e  intEnableWOM            ( bool enable );
            ICM_20948_Status_e  intEnableWOF            ( bool enable );
            ICM_20948_Status_e  intEnableRawDataReady   ( bool enable );
            ICM_20948_Status_e  intEnableOverflowFIFO   ( uint8_t bm_enable );
            ICM_20948_Status_e  intEnableWatermarkFIFO  ( uint8_t bm_enable );
        */
        _wrapped_imu.cfgIntActiveLow(true);  // Active low to be compatible with the breakout board's pullup resistor
        _wrapped_imu.cfgIntOpenDrain(false); // Push-pull, though open-drain would also work thanks to the pull-up resistors on the breakout
        _wrapped_imu.cfgIntLatch(false);
        Serial.print(F("cfgIntLatch returned: "));
        Serial.println(_wrapped_imu.statusString());
        _wrapped_imu.cfgIntAnyReadToClear(true);

        _wrapped_imu.intEnableRawDataReady(true); // enable interrupts on raw data ready
        Serial.print(F("intEnableRawDataReady returned: "));
        Serial.println(_wrapped_imu.statusString());

        //  // Note: weirdness with the Wake on Motion interrupt being always enabled.....
        //  uint8_t zero_0 = 0xFF;
        //  ICM_20948_execute_r( &_wrapped_imu._device, AGB0_REG_INT_ENABLE, (uint8_t*)&zero_0, sizeof(uint8_t) );
        //  Serial.print("INT_EN was: 0x"); Serial.println(zero_0, HEX);
        //  zero_0 = 0x00;
        //  ICM_20948_execute_w( &_wrapped_imu._device, AGB0_REG_INT_ENABLE, (uint8_t*)&zero_0, sizeof(uint8_t) );

        //_wrapped_imu.clearInterrupts();
        return status;
    }

    int get_rate() {
        return _rate_hz;
    }
    void set_rate(int rate) {
        _rate_hz = rate;
    }

    //Get sensor data in NED frame
    //x=North (forward), y=East (right), z=Down 
    //acc: gravitation force is positive in axis direction (i.e. at rest az = 1)
    //gyro: direction of positive rotation by right hand rule, i.e. positive is: yaw right, roll right, pitch up
    void getMotion6NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
    {
        _wrapped_imu.getAGMT();
        if (_wrapped_imu.status != ICM_20948_Stat_Ok)
        {
            Serial.print(F("getAGMT returned: "));
            Serial.println(_wrapped_imu.statusString());
            return;
        }
        _wrapped_imu.clearInterrupts();

        *ax = _wrapped_imu.accX();
        *ay = _wrapped_imu.accY();
        *az = _wrapped_imu.accZ();

        *gx = _wrapped_imu.gyrX();
        *gy = _wrapped_imu.gyrY();
        *gz = _wrapped_imu.gyrZ();
    }

    // Get sensor data in NED frame
    // x=North (forward), y=East (right), z=Down 
    // acc: gravitation force is positive in axis direction
    // gyro: direction of positive rotation by right hand rule, i.e. positive is: yaw right, roll right, pitch up
    void getMotion9NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz) {
        _wrapped_imu.getAGMT();
        if (_wrapped_imu.status != ICM_20948_Stat_Ok)
        {
            Serial.print(F("getAGMT returned: "));
            Serial.println(_wrapped_imu.statusString());
            return;
        }
        //_wrapped_imu.clearInterrupts();

        *ax = _wrapped_imu.accX();
        *ay = _wrapped_imu.accY();
        *az = _wrapped_imu.accZ();

        *gx = _wrapped_imu.gyrX();
        *gy = _wrapped_imu.gyrY();
        *gz = _wrapped_imu.gyrZ();

        *mx = _wrapped_imu.magX();
        *my = _wrapped_imu.magY();
        *mz = _wrapped_imu.magZ();
    }
};

ImuGizmoICM20948* ImuGizmoICM20948::instance = nullptr;

#endif // MF_ICM20948_H
