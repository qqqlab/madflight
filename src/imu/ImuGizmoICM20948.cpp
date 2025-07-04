#include "ICM20948/ICM_20948.h"
#include "ImuGizmoICM20948.h"

// Static member initialization
ImuGizmoICM20948* ImuGizmoICM20948::instance = nullptr;

uint16_t ImuGizmoICM20948::_convertAccelScale(uint16_t range) {
    // Accel: Supported full scale ranges are: 2, 4, 8, 16, 32 G (any other value defaults to 16 G).
    uint16_t ret = 16;
    if (range > 16) ret = 32;
    else if (range > 8) ret = 16;
    else if (range > 4) ret = 8;
    else if (range > 2) ret = 4;
    else if (range == 2) ret = 2;
    return ret;
}

uint16_t ImuGizmoICM20948::_convertGyroScale(uint16_t range) {
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

ImuGizmoICM20948::ImuGizmoICM20948(SPIClass *spi, const uint8_t csPin, const uint8_t intPin, const bool has_mag)
    : _spi(spi), _csPin(csPin), _interrupt_pin(intPin) {
    _wrapped_imu = new ICM_20948_SPI();
    this->has_mag = has_mag;
    uses_i2c = false;
    has_sensor_fusion = true;
    interrupt_has_rising_edge = false;
    ImuGizmoICM20948::instance = this;
}

ImuGizmoICM20948::~ImuGizmoICM20948() {
    if (_wrapped_imu) {
        delete _wrapped_imu;
        _wrapped_imu = nullptr;
    }
    instance = nullptr;
}

int ImuGizmoICM20948::begin(int gyro_scale_dps, int acc_scale_g, int rate_hz) {
    Serial.println("IMU: ICM20948 - initializing wrapped IMU ...");
    Serial.flush();
    _rate_hz = rate_hz;
    int status =  _wrapped_imu->begin(_csPin, *_spi, _rate_hz);
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
    _wrapped_imu->swReset();
    if (_wrapped_imu->status != ICM_20948_Stat_Ok)
    {
        Serial.print(F("Software Reset returned: "));
        Serial.println(_wrapped_imu->statusString());
    }
    delay(250);
    // Now wake the sensor up
    _wrapped_imu->sleep(false);
    _wrapped_imu->lowPower(false);

    if (has_sensor_fusion) {
        // Manually start up the magnetometer
        status = _wrapped_imu->startupMagnetometer();
        if (status != ICM_20948_Stat_Ok)
        {
            Serial.print(F("startupMagnetometer returned: "));
            Serial.println(_wrapped_imu->statusString());
            return status;
        }
        Serial.print(F("startupMagnetometer returned: "));
        Serial.println(_wrapped_imu->statusString());

        // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
        status = _wrapped_imu->initializeDMP();
        if (status != ICM_20948_Stat_Ok)
        {
            Serial.print(F("initializeDMP returned: "));
            Serial.println(_wrapped_imu->statusString());
            return status;
        }

        // DMP sensor options are defined in ICM_20948_DMP.h
        //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
        //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
        //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
        //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
        //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
        //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
        //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
        //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
        //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
        //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
        //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
        //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
        //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
        //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
        //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

        if (has_mag) {
            // Enable the DMP orientation sensor
            Serial.println("Using Magnetometer");
            status = _wrapped_imu->enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION);
            if (status != ICM_20948_Stat_Ok)
            {
                Serial.print(F("enableDMPSensor returned: "));
                Serial.println(_wrapped_imu->statusString());
                return status;
            }

            // Enable raw sensor data for AHRS compatibility
            status = _wrapped_imu->enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE);
            if (status != ICM_20948_Stat_Ok)
            {
                Serial.print(F("enableDMPSensor RAW_GYRO returned: "));
                Serial.println(_wrapped_imu->statusString());
                return status;
            }

            status = _wrapped_imu->enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER);
            if (status != ICM_20948_Stat_Ok)
            {
                Serial.print(F("enableDMPSensor RAW_ACCEL returned: "));
                Serial.println(_wrapped_imu->statusString());
                return status;
            }

            status = _wrapped_imu->enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED);
            if (status != ICM_20948_Stat_Ok)
            {
                Serial.print(F("enableDMPSensor RAW_MAG returned: "));
                Serial.println(_wrapped_imu->statusString());
                return status;
            }

            // Configuring DMP to output data at multiple ODRs:
            // DMP is capable of outputting multiple sensor data at different rates to FIFO.
            // Setting value can be calculated as follows:
            // Value = (DMP running rate / ODR ) - 1
            // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
            status = _wrapped_imu->setDMPODRrate(DMP_ODR_Reg_Quat9, 0); // Set to the maximum
            if (status != ICM_20948_Stat_Ok)
            {
                Serial.print(F("setDMPODRrate returned: "));
                Serial.println(_wrapped_imu->statusString());
                return status;
            }

            // Set ODR rates for raw sensor data
            status = _wrapped_imu->setDMPODRrate(DMP_ODR_Reg_Accel, 0); // Set to the maximum
            if (status != ICM_20948_Stat_Ok)
            {
                Serial.print(F("setDMPODRrate Accel returned: "));
                Serial.println(_wrapped_imu->statusString());
                return status;
            }

            status = _wrapped_imu->setDMPODRrate(DMP_ODR_Reg_Gyro, 0); // Set to the maximum
            if (status != ICM_20948_Stat_Ok)
            {
                Serial.print(F("setDMPODRrate Gyro returned: "));
                Serial.println(_wrapped_imu->statusString());
                return status;
            }

            status = _wrapped_imu->setDMPODRrate(DMP_ODR_Reg_Cpass, 0); // Set to the maximum
            if (status != ICM_20948_Stat_Ok)
            {
                Serial.print(F("setDMPODRrate Cpass returned: "));
                Serial.println(_wrapped_imu->statusString());
                return status;
            }
        } else {
            Serial.println("Not using Magnetometer");
            status = _wrapped_imu->enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR);
            if (status != ICM_20948_Stat_Ok)
            {
                Serial.print(F("enableDMPSensor returned: "));
                Serial.println(_wrapped_imu->statusString());
                return status;
            }

            // Configuring DMP to output data at multiple ODRs:
            // DMP is capable of outputting multiple sensor data at different rates to FIFO.
            // Setting value can be calculated as follows:
            // Value = (DMP running rate / ODR ) - 1
            // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
            status = _wrapped_imu->setDMPODRrate(DMP_ODR_Reg_Quat6, 0); // Set to the maximum
            if (status != ICM_20948_Stat_Ok)
            {
                Serial.print(F("setDMPODRrate returned: "));
                Serial.println(_wrapped_imu->statusString());
                return status;
            }
        }
        //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
        //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
        //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
        //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
        //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum

        // Enable the FIFO
        status = _wrapped_imu->enableFIFO();
        if (status != ICM_20948_Stat_Ok)
        {
            Serial.print(F("enableFIFO returned: "));
            Serial.println(_wrapped_imu->statusString());
            return status;
        }

        // Enable the DMP
        status = _wrapped_imu->enableDMP();
        if (status != ICM_20948_Stat_Ok)
        {
            Serial.print(F("enableDMP returned: "));
            Serial.println(_wrapped_imu->statusString());
            return status;
        }

        // Reset DMP
        status = _wrapped_imu->resetDMP();
        if (status != ICM_20948_Stat_Ok)
        {
            Serial.print(F("resetDMP returned: "));
            Serial.println(_wrapped_imu->statusString());
            return status;
        }

        // Reset FIFO
        status = _wrapped_imu->resetFIFO();
        if (status != ICM_20948_Stat_Ok)
        {
            Serial.print(F("resetFIFO returned: "));
            Serial.println(_wrapped_imu->statusString());
            return status;
        }

        // Check success
        if (status == ICM_20948_Stat_Ok)
        {
            Serial.println(F("DMP enabled!"));
        }
        else
        {
            Serial.println(F("Enable DMP failed!"));
            Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
        }
    } else {
        // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

        // Set Gyro and Accelerometer to a particular sample mode
        // options: ICM_20948_Sample_Mode_Continuous
        //          ICM_20948_Sample_Mode_Cycled
        _wrapped_imu->setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Cycled);
        if (_wrapped_imu->status != ICM_20948_Stat_Ok)
        {
            Serial.print(F("setSampleMode returned: "));
            Serial.println(_wrapped_imu->statusString());
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

        _wrapped_imu->setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
        if (_wrapped_imu->status != ICM_20948_Stat_Ok)
        {
            Serial.print(F("setFullScale returned: "));
            Serial.println(_wrapped_imu->statusString());
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

        _wrapped_imu->setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
        if (_wrapped_imu->status != ICM_20948_Stat_Ok)
        {
            Serial.print(F("setDLPcfg returned: "));
            Serial.println(_wrapped_imu->statusString());
        }

        // Choose whether or not to use DLPF
        // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
        ICM_20948_Status_e accDLPEnableStat = _wrapped_imu->enableDLPF(ICM_20948_Internal_Acc, true);
        ICM_20948_Status_e gyrDLPEnableStat = _wrapped_imu->enableDLPF(ICM_20948_Internal_Gyr, true);
        Serial.print(F("Enable DLPF for Accelerometer returned: "));
        Serial.println(_wrapped_imu->statusString(accDLPEnableStat));
        Serial.print(F("Enable DLPF for Gyroscope returned: "));
        Serial.println(_wrapped_imu->statusString(gyrDLPEnableStat));

        // Choose whether or not to start the magnetometer
        _wrapped_imu->startupMagnetometer();
        if (_wrapped_imu->status != ICM_20948_Stat_Ok)
        {
            Serial.print(F("startupMagnetometer returned: "));
            Serial.println(_wrapped_imu->statusString());
        }
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
    _wrapped_imu->cfgIntActiveLow(true);  // Active low to be compatible with the breakout board's pullup resistor
    _wrapped_imu->cfgIntOpenDrain(false); // Push-pull, though open-drain would also work thanks to the pull-up resistors on the breakout
    _wrapped_imu->cfgIntLatch(false);
    Serial.print(F("cfgIntLatch returned: "));
    Serial.println(_wrapped_imu->statusString());
    _wrapped_imu->cfgIntAnyReadToClear(true);

    if (has_sensor_fusion) {
        _wrapped_imu->intEnableDMP(true);
        Serial.print(F("intEnableDMP returned: "));
        Serial.println(_wrapped_imu->statusString());
    } else {
        _wrapped_imu->intEnableRawDataReady(true); // enable interrupts on raw data ready
        Serial.print(F("intEnableRawDataReady returned: "));
        Serial.println(_wrapped_imu->statusString());
    }

    //  // Note: weirdness with the Wake on Motion interrupt being always enabled.....
    //  uint8_t zero_0 = 0xFF;
    //  ICM_20948_execute_r( &_wrapped_imu->_device, AGB0_REG_INT_ENABLE, (uint8_t*)&zero_0, sizeof(uint8_t) );
    //  Serial.print("INT_EN was: 0x"); Serial.println(zero_0, HEX);
    //  zero_0 = 0x00;
    //  ICM_20948_execute_w( &_wrapped_imu->_device, AGB0_REG_INT_ENABLE, (uint8_t*)&zero_0, sizeof(uint8_t) );

    //_wrapped_imu->clearInterrupts();

    return status;
}

int ImuGizmoICM20948::get_rate() {
    return _rate_hz;
}
void ImuGizmoICM20948::set_rate(int rate) {
    _rate_hz = rate;
}

//Get sensor data in NED frame
//x=North (forward), y=East (right), z=Down
//acc: gravitation force is positive in axis direction (i.e. at rest az = 1)
//gyro: direction of positive rotation by right hand rule, i.e. positive is: yaw right, roll right, pitch up
void ImuGizmoICM20948::getMotion6NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    _wrapped_imu->getAGMT();
    if (_wrapped_imu->status != ICM_20948_Stat_Ok)
    {
        Serial.print(F("getAGMT returned: "));
        Serial.println(_wrapped_imu->statusString());
        return;
    }
    _wrapped_imu->clearInterrupts();

    *ax = _wrapped_imu->accX();
    *ay = _wrapped_imu->accY();
    *az = _wrapped_imu->accZ();

    *gx = _wrapped_imu->gyrX();
    *gy = _wrapped_imu->gyrY();
    *gz = _wrapped_imu->gyrZ();
}

// Get sensor data in NED frame
// x=North (forward), y=East (right), z=Down
// acc: gravitation force is positive in axis direction
// gyro: direction of positive rotation by right hand rule, i.e. positive is: yaw right, roll right, pitch up
void ImuGizmoICM20948::getMotion9NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz) {
    if (has_sensor_fusion) {
        // Read raw sensor data from DMP FIFO
        icm_20948_DMP_data_t data;
        _wrapped_imu->readDMPdataFromFIFO(&data);

        if ((_wrapped_imu->status == ICM_20948_Stat_Ok) || (_wrapped_imu->status == ICM_20948_Stat_FIFOMoreDataAvail)) {
            // Extract raw accelerometer data
            if ((data.header & DMP_header_bitmap_Accel) > 0) {
                *ax = (float)data.Raw_Accel.Data.X;
                *ay = (float)data.Raw_Accel.Data.Y;
                *az = (float)data.Raw_Accel.Data.Z;
            } else {
                *ax = 0; *ay = 0; *az = 0;
            }

            // Extract raw gyroscope data
            if ((data.header & DMP_header_bitmap_Gyro) > 0) {
                *gx = (float)data.Raw_Gyro.Data.X;
                *gy = (float)data.Raw_Gyro.Data.Y;
                *gz = (float)data.Raw_Gyro.Data.Z;
            } else {
                *gx = 0; *gy = 0; *gz = 0;
            }

            // Extract raw magnetometer data
            if ((data.header & DMP_header_bitmap_Compass) > 0) {
                *mx = (float)data.Compass.Data.X;
                *my = (float)data.Compass.Data.Y;
                *mz = (float)data.Compass.Data.Z;
            } else {
                *mx = 0; *my = 0; *mz = 0;
            }
        } else {
            // If no DMP data available, fall back to getAGMT
            _wrapped_imu->getAGMT();
            if (_wrapped_imu->status != ICM_20948_Stat_Ok) {
                Serial.print(F("getAGMT returned: "));
                Serial.println(_wrapped_imu->statusString());
                return;
            }

            *ax = _wrapped_imu->accX();
            *ay = _wrapped_imu->accY();
            *az = _wrapped_imu->accZ();
            *gx = _wrapped_imu->gyrX();
            *gy = _wrapped_imu->gyrY();
            *gz = _wrapped_imu->gyrZ();
            *mx = _wrapped_imu->magX();
            *my = _wrapped_imu->magY();
            *mz = _wrapped_imu->magZ();
        }
    } else {
        // For non-sensor fusion mode, use the original getAGMT method
        _wrapped_imu->getAGMT();
        if (_wrapped_imu->status != ICM_20948_Stat_Ok) {
            Serial.print(F("getAGMT returned: "));
            Serial.println(_wrapped_imu->statusString());
            return;
        }

        *ax = _wrapped_imu->accX();
        *ay = _wrapped_imu->accY();
        *az = _wrapped_imu->accZ();
        *gx = _wrapped_imu->gyrX();
        *gy = _wrapped_imu->gyrY();
        *gz = _wrapped_imu->gyrZ();
        *mx = _wrapped_imu->magX();
        *my = _wrapped_imu->magY();
        *mz = _wrapped_imu->magZ();
    }
}

void ImuGizmoICM20948::get6DOF(float *q0, float *q1, float *q2, float *q3) {
    // Read any DMP data waiting in the FIFO
    // Note:
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
    //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
    //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
    icm_20948_DMP_data_t data;
    _wrapped_imu->readDMPdataFromFIFO(&data);

    if ((_wrapped_imu->status == ICM_20948_Stat_Ok) || (_wrapped_imu->status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
    {
        //Serial.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
        //if ( data.header < 0x1000) Serial.print( "0" ); // Pad the zeros
        //if ( data.header < 0x100) Serial.print( "0" );
        //if ( data.header < 0x10) Serial.print( "0" );
        //Serial.println( data.header, HEX );

        if ((data.header & DMP_header_bitmap_Quat6) > 0) // We have asked for GRV data so we should receive Quat6
        {
            // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
            // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
            // The quaternion data is scaled by 2^30.

            //Serial.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

            // Scale to +/- 1
            *q1 = (float)((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
            *q2 = (float)((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
            *q3 = (float)((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
            *q0 = (float)sqrt(1.0 - ((*q1 * *q1) + (*q2 * *q2) + (*q3 * *q3)));

            // Convert from Android LH ENU to RH NED
            float w = *q0;
            float x = -(*q1);
            float y = -(*q2);
            float z = -(*q3);

            *q0 = w;
            *q1 = -y;    // +X_NED = North
            *q2 = -x;    // +Y_NED = East
            *q3 = z;     // +Z_NED = Down
        }
    }
}

void ImuGizmoICM20948::get9DOF(float *q0, float *q1, float *q2, float *q3) {
    // Read any DMP data waiting in the FIFO
    // Note:
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
    //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
    //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
    icm_20948_DMP_data_t data;
    _wrapped_imu->readDMPdataFromFIFO(&data);

    if ((_wrapped_imu->status == ICM_20948_Stat_Ok) || (_wrapped_imu->status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
    {
        //Serial.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
        //if ( data.header < 0x1000) Serial.print( "0" ); // Pad the zeros
        //if ( data.header < 0x100) Serial.print( "0" );
        //if ( data.header < 0x10) Serial.print( "0" );
        //Serial.println( data.header, HEX );

        if ((data.header & DMP_header_bitmap_Quat9) > 0) // We have asked for orientation data so we should receive Quat9
        {
            // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
            // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
            // The quaternion data is scaled by 2^30.

            //Serial.printf("Quat9 data is: Q1:%ld Q2:%ld Q3:%ld Accuracy:%d\r\n", data.Quat9.Data.Q1, data.Quat9.Data.Q2, data.Quat9.Data.Q3, data.Quat9.Data.Accuracy);

            // Scale to +/- 1
            *q1 = (float)((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
            *q2 = (float)((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
            *q3 = (float)((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
            *q0 = (float)sqrt(1.0 - ((*q1 * *q1) + (*q2 * *q2) + (*q3 * *q3)));

            // Convert from Android LH ENU to RH NED
            float w = *q0;
            float x = -(*q1);
            float y = -(*q2);
            float z = -(*q3);

            *q0 = w;
            *q1 = -y;    // +X_NED = North
            *q2 = -x;    // +Y_NED = East
            *q3 = z;     // +Z_NED = Down
        }
    }
}
