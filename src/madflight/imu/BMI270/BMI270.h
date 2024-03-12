// madflight https://github.com/qqqlab/madflight
// Header only BMI270 SPI IMU sensor class

#pragma once

#include <SPI.h>

//Global array that stores the configuration file of BMI270
static const uint8_t bmi270_maximum_fifo_config_file[] = {
    0xc8, 0x2e, 0x00, 0x2e, 0x80, 0x2e, 0x1a, 0x00, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00,
    0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0x90, 0x32, 0x21, 0x2e, 0x59, 0xf5,
    0x10, 0x30, 0x21, 0x2e, 0x6a, 0xf5, 0x1a, 0x24, 0x22, 0x00, 0x80, 0x2e, 0x3b, 0x00, 0xc8, 0x2e, 0x44, 0x47, 0x22,
    0x00, 0x37, 0x00, 0xa4, 0x00, 0xff, 0x0f, 0xd1, 0x00, 0x07, 0xad, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
    0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00,
    0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x11, 0x24, 0xfc, 0xf5, 0x80, 0x30, 0x40, 0x42, 0x50, 0x50, 0x00, 0x30, 0x12, 0x24, 0xeb,
    0x00, 0x03, 0x30, 0x00, 0x2e, 0xc1, 0x86, 0x5a, 0x0e, 0xfb, 0x2f, 0x21, 0x2e, 0xfc, 0xf5, 0x13, 0x24, 0x63, 0xf5,
    0xe0, 0x3c, 0x48, 0x00, 0x22, 0x30, 0xf7, 0x80, 0xc2, 0x42, 0xe1, 0x7f, 0x3a, 0x25, 0xfc, 0x86, 0xf0, 0x7f, 0x41,
    0x33, 0x98, 0x2e, 0xc2, 0xc4, 0xd6, 0x6f, 0xf1, 0x30, 0xf1, 0x08, 0xc4, 0x6f, 0x11, 0x24, 0xff, 0x03, 0x12, 0x24,
    0x00, 0xfc, 0x61, 0x09, 0xa2, 0x08, 0x36, 0xbe, 0x2a, 0xb9, 0x13, 0x24, 0x38, 0x00, 0x64, 0xbb, 0xd1, 0xbe, 0x94,
    0x0a, 0x71, 0x08, 0xd5, 0x42, 0x21, 0xbd, 0x91, 0xbc, 0xd2, 0x42, 0xc1, 0x42, 0x00, 0xb2, 0xfe, 0x82, 0x05, 0x2f,
    0x50, 0x30, 0x21, 0x2e, 0x21, 0xf2, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0xf0, 0x6f, 0x02, 0x30, 0x02, 0x42, 0x20,
    0x26, 0xe0, 0x6f, 0x02, 0x31, 0x03, 0x40, 0x9a, 0x0a, 0x02, 0x42, 0xf0, 0x37, 0x05, 0x2e, 0x5e, 0xf7, 0x10, 0x08,
    0x12, 0x24, 0x1e, 0xf2, 0x80, 0x42, 0x83, 0x84, 0xf1, 0x7f, 0x0a, 0x25, 0x13, 0x30, 0x83, 0x42, 0x3b, 0x82, 0xf0,
    0x6f, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x00, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x3e, 0x84,
    0x00, 0x40, 0x40, 0x42, 0x7e, 0x82, 0xe1, 0x7f, 0xf2, 0x7f, 0x98, 0x2e, 0x6a, 0xd6, 0x21, 0x30, 0x23, 0x2e, 0x61,
    0xf5, 0xeb, 0x2c, 0xe1, 0x6f
};

class BMI270 {
  public:
    int spi_freq = 10000000; //BMI270: SPI 10MHz max, I2C 1MHz max
    int16_t a[3]; //raw acc data
    int16_t g[3]; //raw gyro data

    // BMI270 registers (not the complete list)
    typedef enum {
        BMI270_REG_CHIP_ID = 0x00,
        BMI270_REG_ERR_REG = 0x02,
        BMI270_REG_STATUS = 0x03,
        BMI270_REG_ACC_DATA_X_LSB = 0x0C,
        BMI270_REG_GYR_DATA_X_LSB = 0x12,
        BMI270_REG_SENSORTIME_0 = 0x18,
        BMI270_REG_SENSORTIME_1 = 0x19,
        BMI270_REG_SENSORTIME_2 = 0x1A,
        BMI270_REG_EVENT = 0x1B,
        BMI270_REG_INT_STATUS_0 = 0x1C,
        BMI270_REG_INT_STATUS_1 = 0x1D,
        BMI270_REG_INTERNAL_STATUS = 0x21,
        BMI270_REG_TEMPERATURE_LSB = 0x22,
        BMI270_REG_TEMPERATURE_MSB = 0x23,
        BMI270_REG_FIFO_LENGTH_LSB = 0x24,
        BMI270_REG_FIFO_LENGTH_MSB = 0x25,
        BMI270_REG_FIFO_DATA = 0x26,
        BMI270_REG_ACC_CONF = 0x40,
        BMI270_REG_ACC_RANGE = 0x41,
        BMI270_REG_GYRO_CONF = 0x42,
        BMI270_REG_GYRO_RANGE = 0x43,
        BMI270_REG_AUX_CONF = 0x44,
        BMI270_REG_FIFO_DOWNS = 0x45,
        BMI270_REG_FIFO_WTM_0 = 0x46,
        BMI270_REG_FIFO_WTM_1 = 0x47,
        BMI270_REG_FIFO_CONFIG_0 = 0x48,
        BMI270_REG_FIFO_CONFIG_1 = 0x49,
        BMI270_REG_SATURATION = 0x4A,
        BMI270_REG_INT1_IO_CTRL = 0x53,
        BMI270_REG_INT2_IO_CTRL = 0x54,
        BMI270_REG_INT_LATCH = 0x55,
        BMI270_REG_INT1_MAP_FEAT = 0x56,
        BMI270_REG_INT2_MAP_FEAT = 0x57,
        BMI270_REG_INT_MAP_DATA = 0x58,
        BMI270_REG_INIT_CTRL = 0x59,
        BMI270_REG_INIT_DATA = 0x5E,
        BMI270_REG_ACC_SELF_TEST = 0x6D,
        BMI270_REG_GYR_SELF_TEST_AXES = 0x6E,
        BMI270_REG_PWR_CONF = 0x7C,
        BMI270_REG_PWR_CTRL = 0x7D,
        BMI270_REG_CMD = 0x7E,
    } Register_e;

    // BMI270 register configuration values
    typedef enum {
        BMI270_VAL_CMD_SOFTRESET = 0xB6,
        BMI270_VAL_CMD_FIFOFLUSH = 0xB0,
        BMI270_VAL_PWR_CTRL = 0x0E,              // enable gyro, acc and temp sensors
        BMI270_VAL_PWR_CONF = 0x02,              // disable advanced power save, enable FIFO self-wake
        BMI270_VAL_ACC_CONF_BWP = 0x01,          // set acc filter in osr2 mode (only in high performance mode)
        BMI270_VAL_ACC_CONF_HP = 0x01,           // set acc in high performance mode
        BMI270_VAL_GYRO_CONF_BWP_OSR4 = 0x00,    // set gyro filter in OSR4 mode
        BMI270_VAL_GYRO_CONF_BWP_OSR2 = 0x01,    // set gyro filter in OSR2 mode
        BMI270_VAL_GYRO_CONF_BWP_NORM = 0x02,    // set gyro filter in normal mode
        BMI270_VAL_GYRO_CONF_NOISE_PERF = 0x01,  // set gyro in high performance noise mode
        BMI270_VAL_GYRO_CONF_FILTER_PERF = 0x01, // set gyro in high performance filter mode
        BMI270_VAL_INT_MAP_DATA_DRDY_INT1 = 0x04,// enable the data ready interrupt pin 1
        BMI270_VAL_INT_MAP_FIFO_WM_INT1 = 0x02,  // enable the FIFO watermark interrupt pin 1
        BMI270_VAL_INT1_IO_CTRL_PINMODE = 0x0A,  // active high, push-pull, output enabled, input disabled 
        BMI270_VAL_FIFO_CONFIG_0 = 0x00,         // don't stop when full, disable sensortime frame
        BMI270_VAL_FIFO_CONFIG_1 = 0x80,         // only gyro data in FIFO, use headerless mode
        BMI270_VAL_FIFO_DOWNS = 0x00,            // select unfiltered gyro data with no downsampling (6.4KHz samples)
        BMI270_VAL_FIFO_WTM_0 = 0x06,            // set the FIFO watermark level to 1 gyro sample (6 bytes)
        BMI270_VAL_FIFO_WTM_1 = 0x00,            // FIFO watermark MSB
    } ConfigValues_e;

    typedef enum {
        GYRO_ODR_25_HZ = 6,
        GYRO_ODR_50_HZ,
        GYRO_ODR_100_HZ,
        GYRO_ODR_200_HZ,
        GYRO_ODR_400_HZ,
        GYRO_ODR_800_HZ,
        GYRO_ODR_1600_HZ,
        GYRO_ODR_3200_HZ
    } gyroOdr_e;

    typedef enum {
        GYRO_RANGE_2000_DPS = 8, //for some reason you have to enable the ois_range bit (bit 3) for 2000dps as well, or else the gyro scale will be 250dps when in prefiltered FIFO mode (not documented in datasheet!)
        GYRO_RANGE_1000_DPS = 1,
        GYRO_RANGE_500_DPS = 2,
        GYRO_RANGE_250_DPS = 3,
        GYRO_RANGE_125_DPS = 4
    } gyroRange_e;

    typedef enum {
        ACCEL_ODR_0_78_HZ = 1,
        ACCEL_ODR_1_56_HZ,
        ACCEL_ODR_3_12_HZ,
        ACCEL_ODR_6_25_HZ,
        ACCEL_ODR_12_5_HZ,
        ACCEL_ODR_25_HZ,
        ACCEL_ODR_50_HZ,
        ACCEL_ODR_100_HZ,
        ACCEL_ODR_200_HZ,
        ACCEL_ODR_400_HZ,
        ACCEL_ODR_800_HZ,
        ACCEL_ODR_1600_HZ
    } accelOdr_e;

    typedef enum {
        ACCEL_RANGE_2_G = 0,
        ACCEL_RANGE_4_G,
        ACCEL_RANGE_8_G,
        ACCEL_RANGE_16_G
    } accelRange_e;

    float acc_multiplier;
    float gyro_multiplier;

    BMI270( SPIClass *spi, const uint8_t csPin ) {
        _spi = spi;
        _csPin = csPin;
    }

    int get_rate() {
      return _rate_hz;
    }

    //returns 0 on success
    int begin(int gyro_scale_dps, int acc_scale_g, int rate_hz) {
      set_gyro_scale_dps(gyro_scale_dps);
      set_acc_scale_g(acc_scale_g);
      set_rate(rate_hz);

      pinMode(_csPin, OUTPUT);
      digitalWrite(_csPin, HIGH);
      delay(10);

      // Toggle the CS to switch the device into SPI mode.
      // Device switches initializes as I2C and switches to SPI on a low to high CS transition
      digitalWrite(_csPin, LOW);
      delay(1);
      digitalWrite(_csPin, HIGH);
      delay(10);

      // Perform a soft reset to set all configuration to default
      // Delay 100ms before continuing configuration
      write_reg(BMI270_REG_CMD, BMI270_VAL_CMD_SOFTRESET);
      delay(100);

      // Toggle the CS to switch the device into SPI mode.
      // Device switches initializes as I2C and switches to SPI on a low to high CS transition
      digitalWrite(_csPin, LOW);
      delay(1);
      digitalWrite(_csPin, HIGH);
      delay(10);

      bmi270UploadConfig();

      // Configure the accelerometer
      write_reg(BMI270_REG_ACC_CONF, (BMI270_VAL_ACC_CONF_HP << 7) | (BMI270_VAL_ACC_CONF_BWP << 4) | accelOdr);

      // Configure the accelerometer full-scale range
      write_reg(BMI270_REG_ACC_RANGE, accelRange);

      // Configure the gyro
      uint8_t GyroOsrMode = BMI270_VAL_GYRO_CONF_BWP_OSR4;
      //uint8_t GyroOsrMode = BMI270_VAL_GYRO_CONF_BWP_OSR2;
      //uint8_t GyroOsrMode = BMI270_VAL_GYRO_CONF_BWP_NORM;        
      write_reg(BMI270_REG_GYRO_CONF, (BMI270_VAL_GYRO_CONF_FILTER_PERF << 7) | (BMI270_VAL_GYRO_CONF_NOISE_PERF << 6) | (GyroOsrMode << 4) | gyroOdr);

      // Configure the gyro full-range scale
      write_reg(BMI270_REG_GYRO_RANGE, gyroRange);

      // Interrupt driven by data ready
      write_reg(BMI270_REG_INT_MAP_DATA, BMI270_VAL_INT_MAP_DATA_DRDY_INT1);

      // Configure the behavior of the INT1 pin
      write_reg(BMI270_REG_INT1_IO_CTRL, BMI270_VAL_INT1_IO_CTRL_PINMODE);

      // Configure the device for  performance mode
      write_reg(BMI270_REG_PWR_CONF, BMI270_VAL_PWR_CONF);

      // Enable the gyro, accelerometer and temperature sensor - disable aux interface
      write_reg(BMI270_REG_PWR_CTRL, BMI270_VAL_PWR_CTRL);

      return 0;
  }

    int who_am_i() {
      return read_reg(0x00);
    }

    void readSensor(void) {
        uint8_t data[12];
        spi_read(0x0C, data, 12);
        a[0] = data[1]<<8 | data[0];
        a[1] = data[3]<<8 | data[2];
        a[2] = data[5]<<8 | data[4];
        g[0] = data[7]<<8 | data[6];
        g[1] = data[9]<<8 | data[8];
        g[2] = data[11]<<8 | data[10];
    }

    //Get sensor data in NED frame
    //x=North (forward), y=East (right), z=Down 
    //acc: gravitation force is positive in axis direction (i.e. at rest az = 1)
    //gyro: direction of positive rotation by right hand rule, i.e. positive is: yaw right, roll right, pitch up
    void getMotion6NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
    {
      readSensor();
      //BM270 sensor orientation for gyro is NWU
      //BM270 sensor orientation for acc  is SED (i.e. at rest az = 1)
      *ax = -a[0] * acc_multiplier;  //North = -South
      *ay = a[1] * acc_multiplier;   //East  = East
      *az = a[2] * acc_multiplier;   //Down  = Down
      *gx = g[0] * gyro_multiplier;  //North = North
      *gy = -g[1] * gyro_multiplier; //East  = -West
      *gz = -g[2] * gyro_multiplier; //Down  = -Up
    }

    //set maximum rate such that actual rate <= requested rate
    void set_rate(int rate_hz)
    {
      if(rate_hz >= 3200) {
        accelOdr = ACCEL_ODR_1600_HZ;
        gyroOdr = GYRO_ODR_3200_HZ;
        _rate_hz = 1600;
      }if(rate_hz >= 1600) {
        accelOdr = ACCEL_ODR_1600_HZ;
        gyroOdr = GYRO_ODR_1600_HZ;
        _rate_hz = 1600;
      }else if(rate_hz >= 800) {
        accelOdr = ACCEL_ODR_800_HZ;
        gyroOdr = GYRO_ODR_800_HZ;
        _rate_hz =  800;
      }else if(rate_hz >= 800) {
        accelOdr = ACCEL_ODR_400_HZ;
        gyroOdr = GYRO_ODR_400_HZ;
        _rate_hz =  400;
      }else if(rate_hz >= 200) {
        accelOdr = ACCEL_ODR_200_HZ;
        gyroOdr = GYRO_ODR_200_HZ;
        _rate_hz =  200;
      }else{
        accelOdr = ACCEL_ODR_100_HZ;
        gyroOdr = GYRO_ODR_100_HZ;
        _rate_hz =  100;       
      }
    }

    void set_acc_scale_g(int scale_in_g)
    {
      if(scale_in_g <= 2) {
        accelRange = ACCEL_RANGE_2_G;
        acc_multiplier = 2.0 / 32768.0;
      }else if(scale_in_g <= 4) { 
        accelRange = ACCEL_RANGE_4_G;
        acc_multiplier = 4.0 / 32768.0;
      }else if(scale_in_g <= 8) { 
        accelRange = ACCEL_RANGE_8_G;
        acc_multiplier = 8.0 / 32768.0;
      }else{ 
        accelRange = ACCEL_RANGE_16_G;
        acc_multiplier = 16.0 / 32768.0;
      }
    }

    void set_gyro_scale_dps(int scale_in_dps)
    {
      if(scale_in_dps <= 125) {
        gyroRange = GYRO_RANGE_125_DPS;
        gyro_multiplier = 125.0 / 32768.0;
      }else if(scale_in_dps <= 250) {
        gyroRange = GYRO_RANGE_250_DPS;
        gyro_multiplier = 250.0 / 32768.0;
      }else if(scale_in_dps <= 500) { 
        gyroRange = GYRO_RANGE_500_DPS;
        gyro_multiplier = 500.0 / 32768.0;
      }else if(scale_in_dps <= 1000) { 
        gyroRange = GYRO_RANGE_1000_DPS;
        gyro_multiplier = 1000.0 / 32768.0;
      }else{ 
        gyroRange = GYRO_RANGE_2000_DPS;
        gyro_multiplier = 2000.0 / 32768.0;
      }
    }

  private:
    SPIClass * _spi;
    uint8_t _csPin;
    int _rate_hz;
    accelRange_e accelRange = ACCEL_RANGE_16_G;
    accelOdr_e accelOdr = ACCEL_ODR_100_HZ;
    gyroRange_e gyroRange = GYRO_RANGE_2000_DPS;
    gyroOdr_e gyroOdr = GYRO_ODR_100_HZ;

    void bmi270UploadConfig()
    {
        write_reg(BMI270_REG_PWR_CONF, 0);
        write_reg(BMI270_REG_INIT_CTRL, 0);

        // Transfer the config file
        spi_write(BMI270_REG_INIT_DATA, (uint8_t *)bmi270_maximum_fifo_config_file, sizeof(bmi270_maximum_fifo_config_file));

        delay(10);
        write_reg(BMI270_REG_INIT_CTRL, 1);
    }

    int8_t spi_read(const uint8_t reg, uint8_t * data, const uint32_t count) {
        _spi->beginTransaction(SPISettings(spi_freq, MSBFIRST, SPI_MODE3));
        digitalWrite(_csPin, LOW);
        _spi->transfer(0x80 | reg);
        _spi->transfer(0); //dummy byte --- BMI270 specific ---
        for (uint32_t k=0; k<count; k++) {
            data[k] = _spi->transfer(0);
        }
        digitalWrite(_csPin, HIGH);
        _spi->endTransaction();
        return 0;
    }

    int8_t spi_write(uint8_t reg, const uint8_t *data, uint32_t count) {
        _spi->beginTransaction(SPISettings(spi_freq, MSBFIRST, SPI_MODE3));
        digitalWrite(_csPin, LOW);
        _spi->transfer(reg);
        for (uint32_t k=0; k<count; k++) {
            _spi->transfer(data[k]);
        }
        digitalWrite(_csPin, HIGH);
        _spi->endTransaction();
        return 0;
    }


    void write_reg(uint8_t reg, uint8_t data) {
      spi_write(reg, &data, 1);
      delay(1);
    }

    uint8_t read_reg(uint8_t reg) {
      uint8_t data;
      spi_read(reg, &data, 1);
      delay(1);
      return data;
    }

}; // class BMI270
