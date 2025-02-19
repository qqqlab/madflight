/*
modified Adafruit driver:

append .cpp file to .h
remove #include <Adafruit_xxx.h> from cpp
rename #include <Adafruit_I2CDevice.h> -> #include "I2CDevice.h" to prevent loading from installed libraries 
rename #include <Adafruit_SPIDevice.h> -> #include "SPIDevice.h"
remove Adafruit_Sensor
remove &Wire
remove #include <Wire.h>

Now this header can now be included in the main .ino which defines a custom TwoWire implementation
*/


/*!
 *  @file Adafruit_BMP280.h
 *
 *  This is a library for the Adafruit BMP280 Breakout.
 *
 *  Designed specifically to work with the Adafruit BMP280 Breakout.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/2651
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  K.Townsend (Adafruit Industries)
 *
 *  BSD license, all text above must be included in any redistribution
 */


// clang-format off
#include <Arduino.h>
#include "I2CDevice.h"
#include "SPIDevice.h"
// clang-format on

/*!
 *  I2C ADDRESS/BITS/SETTINGS
 */
#define BMP280_ADDRESS (0x77) /**< The default I2C address for the sensor. */
#define BMP280_ADDRESS_ALT (0x76) /**< Alternative I2C address for the sensor. */
#define BMP280_CHIPID (0x58) /**< Default chip ID. */

/*!
 * Registers available on the sensor.
 */
enum {
  BMP280_REGISTER_DIG_T1 = 0x88,
  BMP280_REGISTER_DIG_T2 = 0x8A,
  BMP280_REGISTER_DIG_T3 = 0x8C,
  BMP280_REGISTER_DIG_P1 = 0x8E,
  BMP280_REGISTER_DIG_P2 = 0x90,
  BMP280_REGISTER_DIG_P3 = 0x92,
  BMP280_REGISTER_DIG_P4 = 0x94,
  BMP280_REGISTER_DIG_P5 = 0x96,
  BMP280_REGISTER_DIG_P6 = 0x98,
  BMP280_REGISTER_DIG_P7 = 0x9A,
  BMP280_REGISTER_DIG_P8 = 0x9C,
  BMP280_REGISTER_DIG_P9 = 0x9E,
  BMP280_REGISTER_CHIPID = 0xD0,
  BMP280_REGISTER_VERSION = 0xD1,
  BMP280_REGISTER_SOFTRESET = 0xE0,
  BMP280_REGISTER_CAL26 = 0xE1, /**< R calibration = 0xE1-0xF0 */
  BMP280_REGISTER_STATUS = 0xF3,
  BMP280_REGISTER_CONTROL = 0xF4,
  BMP280_REGISTER_CONFIG = 0xF5,
  BMP280_REGISTER_PRESSUREDATA = 0xF7,
  BMP280_REGISTER_TEMPDATA = 0xFA,
};

/*!
 *  Struct to hold calibration data.
 */
typedef struct {
  uint16_t dig_T1; /**< dig_T1 cal register. */
  int16_t dig_T2;  /**<  dig_T2 cal register. */
  int16_t dig_T3;  /**< dig_T3 cal register. */

  uint16_t dig_P1; /**< dig_P1 cal register. */
  int16_t dig_P2;  /**< dig_P2 cal register. */
  int16_t dig_P3;  /**< dig_P3 cal register. */
  int16_t dig_P4;  /**< dig_P4 cal register. */
  int16_t dig_P5;  /**< dig_P5 cal register. */
  int16_t dig_P6;  /**< dig_P6 cal register. */
  int16_t dig_P7;  /**< dig_P7 cal register. */
  int16_t dig_P8;  /**< dig_P8 cal register. */
  int16_t dig_P9;  /**< dig_P9 cal register. */
} bmp280_calib_data;

class Adafruit_BMP280;

/**
 * Driver for the Adafruit BMP280 barometric pressure sensor.
 */
class Adafruit_BMP280 {
public:
  /** Oversampling rate for the sensor. */
  enum sensor_sampling {
    /** No over-sampling. */
    SAMPLING_NONE = 0x00,
    /** 1x over-sampling. */
    SAMPLING_X1 = 0x01,
    /** 2x over-sampling. */
    SAMPLING_X2 = 0x02,
    /** 4x over-sampling. */
    SAMPLING_X4 = 0x03,
    /** 8x over-sampling. */
    SAMPLING_X8 = 0x04,
    /** 16x over-sampling. */
    SAMPLING_X16 = 0x05
  };

  /** Operating mode for the sensor. */
  enum sensor_mode {
    /** Sleep mode. */
    MODE_SLEEP = 0x00,
    /** Forced mode. */
    MODE_FORCED = 0x01,
    /** Normal mode. */
    MODE_NORMAL = 0x03,
    /** Software reset. */
    MODE_SOFT_RESET_CODE = 0xB6
  };

  /** Filtering level for sensor data. */
  enum sensor_filter {
    /** No filtering. */
    FILTER_OFF = 0x00,
    /** 2x filtering. */
    FILTER_X2 = 0x01,
    /** 4x filtering. */
    FILTER_X4 = 0x02,
    /** 8x filtering. */
    FILTER_X8 = 0x03,
    /** 16x filtering. */
    FILTER_X16 = 0x04
  };

  /** Standby duration in ms */
  enum standby_duration {
    /** 1 ms standby. */
    STANDBY_MS_1 = 0x00,
    /** 62.5 ms standby. */
    STANDBY_MS_63 = 0x01,
    /** 125 ms standby. */
    STANDBY_MS_125 = 0x02,
    /** 250 ms standby. */
    STANDBY_MS_250 = 0x03,
    /** 500 ms standby. */
    STANDBY_MS_500 = 0x04,
    /** 1000 ms standby. */
    STANDBY_MS_1000 = 0x05,
    /** 2000 ms standby. */
    STANDBY_MS_2000 = 0x06,
    /** 4000 ms standby. */
    STANDBY_MS_4000 = 0x07
  };

  Adafruit_BMP280(TwoWire *theWire);
  Adafruit_BMP280(int8_t cspin, SPIClass *theSPI = &SPI);
  Adafruit_BMP280(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin);
  ~Adafruit_BMP280(void);

  bool begin(uint8_t addr, uint8_t chipid = BMP280_CHIPID);
  void reset(void);
  uint8_t getStatus(void);
  uint8_t sensorID(void);

  float readTemperature();
  float readPressure(void);
  float readAltitude(float seaLevelhPa = 1013.25);
  float seaLevelForAltitude(float altitude, float atmospheric);
  float waterBoilingPoint(float pressure);
  bool takeForcedMeasurement();

  void setSampling(sensor_mode mode = MODE_NORMAL,
                   sensor_sampling tempSampling = SAMPLING_X16,
                   sensor_sampling pressSampling = SAMPLING_X16,
                   sensor_filter filter = FILTER_OFF,
                   standby_duration duration = STANDBY_MS_1);

private:
  TwoWire *_wire;                     /**< Wire object */
  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
  Adafruit_SPIDevice *spi_dev = NULL; ///< Pointer to SPI bus interface

  /** Encapsulates the config register */
  struct config {
    /** Initialize to power-on-reset state */
    config() : t_sb(STANDBY_MS_1), filter(FILTER_OFF), none(0), spi3w_en(0) {}
    /** Inactive duration (standby time) in normal mode */
    unsigned int t_sb : 3;
    /** Filter settings */
    unsigned int filter : 3;
    /** Unused - don't set */
    unsigned int none : 1;
    /** Enables 3-wire SPI */
    unsigned int spi3w_en : 1;
    /** Used to retrieve the assembled config register's byte value. */
    unsigned int get() { return (t_sb << 5) | (filter << 2) | spi3w_en; }
  };

  /** Encapsulates trhe ctrl_meas register */
  struct ctrl_meas {
    /** Initialize to power-on-reset state */
    ctrl_meas()
        : osrs_t(SAMPLING_NONE), osrs_p(SAMPLING_NONE), mode(MODE_SLEEP) {}
    /** Temperature oversampling. */
    unsigned int osrs_t : 3;
    /** Pressure oversampling. */
    unsigned int osrs_p : 3;
    /** Device mode */
    unsigned int mode : 2;
    /** Used to retrieve the assembled ctrl_meas register's byte value. */
    unsigned int get() { return (osrs_t << 5) | (osrs_p << 2) | mode; }
  };

  void readCoefficients(void);
  uint8_t spixfer(uint8_t x);
  void write8(byte reg, byte value);
  uint8_t read8(byte reg);
  uint16_t read16(byte reg);
  uint32_t read24(byte reg);
  int16_t readS16(byte reg);
  uint16_t read16_LE(byte reg);
  int16_t readS16_LE(byte reg);


  int32_t _sensorID = 0;
  int32_t t_fine;
  // int8_t _cs, _mosi, _miso, _sck;
  bmp280_calib_data _bmp280_calib;
  config _configReg;
  ctrl_meas _measReg;
};

/*!
 *  @file Adafruit_BMP280.cpp
 *
 *  This is a library for the BMP280 orientation sensor
 *
 *  Designed specifically to work with the Adafruit BMP280 Sensor.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/2651
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  K.Townsend (Adafruit Industries)
 *
 *  BSD license, all text above must be included in any redistribution
 */


/*!
 * @brief  BMP280 constructor using i2c
 * @param  *theWire
 *         optional wire
 */
Adafruit_BMP280::Adafruit_BMP280(TwoWire *theWire) {
  _wire = theWire;
}

/*!
 * @brief  BMP280 constructor using hardware SPI
 * @param  cspin
 *         cs pin number
 * @param  theSPI
 *         optional SPI object
 */
Adafruit_BMP280::Adafruit_BMP280(int8_t cspin, SPIClass *theSPI) {
  spi_dev = new Adafruit_SPIDevice(cspin, 1000000, SPI_BITORDER_MSBFIRST, SPI_MODE0, theSPI);
}

/*!
 * @brief  BMP280 constructor using bitbang SPI
 * @param  cspin
 *         The pin to use for CS/SSEL.
 * @param  mosipin
 *         The pin to use for MOSI.
 * @param  misopin
 *         The pin to use for MISO.
 * @param  sckpin
 *         The pin to use for SCK.
 */
Adafruit_BMP280::Adafruit_BMP280(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin) {
  spi_dev = new Adafruit_SPIDevice(cspin, sckpin, misopin, mosipin);
}

Adafruit_BMP280::~Adafruit_BMP280(void) {
  if (spi_dev)
    delete spi_dev;
  if (i2c_dev)
    delete i2c_dev;
}

/*!
 *  Initialises the sensor.
 *  @param addr
 *         The I2C address to use (default = 0x77)
 *  @param chipid
 *         The expected chip ID (used to validate connection).
 *  @return True if the init was successful, otherwise false.
 */
bool Adafruit_BMP280::begin(uint8_t addr, uint8_t chipid) {
  if (spi_dev == NULL) {
    // I2C mode
    if (i2c_dev)
      delete i2c_dev;
    if (addr == 0x00) addr = BMP280_ADDRESS_ALT;
    i2c_dev = new Adafruit_I2CDevice(addr, _wire);
    if (!i2c_dev->begin())
      return false;
  } else {
    // SPI mode
    if (!spi_dev->begin())
      return false;
  }

  // check if sensor, i.e. the chip ID is correct
  _sensorID = read8(BMP280_REGISTER_CHIPID);
  if (_sensorID != chipid)
    return false;

  readCoefficients();
  // write8(BMP280_REGISTER_CONTROL, 0x3F); /* needed? */
  setSampling();
  delay(100);
  return true;
}

/*!
 * Sets the sampling config for the device.
 * @param mode
 *        The operating mode of the sensor.
 * @param tempSampling
 *        The sampling scheme for temp readings.
 * @param pressSampling
 *        The sampling scheme for pressure readings.
 * @param filter
 *        The filtering mode to apply (if any).
 * @param duration
 *        The sampling duration.
 */
void Adafruit_BMP280::setSampling(sensor_mode mode,
                                  sensor_sampling tempSampling,
                                  sensor_sampling pressSampling,
                                  sensor_filter filter,
                                  standby_duration duration) {
  if (!_sensorID)
    return; // begin() not called yet
  _measReg.mode = mode;
  _measReg.osrs_t = tempSampling;
  _measReg.osrs_p = pressSampling;

  _configReg.filter = filter;
  _configReg.t_sb = duration;

  write8(BMP280_REGISTER_CONFIG, _configReg.get());
  write8(BMP280_REGISTER_CONTROL, _measReg.get());
}

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C/SPI
*/
/**************************************************************************/
void Adafruit_BMP280::write8(byte reg, byte value) {
  byte buffer[2];
  buffer[1] = value;
  if (i2c_dev) {
    buffer[0] = reg;
    i2c_dev->write(buffer, 2);
  } else {
    buffer[0] = reg & ~0x80;
    spi_dev->write(buffer, 2);
  }
}

/*!
 *  @brief  Reads an 8 bit value over I2C/SPI
 *  @param  reg
 *          selected register
 *  @return value from selected register
 */
uint8_t Adafruit_BMP280::read8(byte reg) {
  uint8_t buffer[1];
  if (i2c_dev) {
    buffer[0] = uint8_t(reg);
    i2c_dev->write_then_read(buffer, 1, buffer, 1);
  } else {
    buffer[0] = uint8_t(reg | 0x80);
    spi_dev->write_then_read(buffer, 1, buffer, 1);
  }
  return buffer[0];
}

/*!
 *  @brief  Reads a 16 bit value over I2C/SPI
 */
uint16_t Adafruit_BMP280::read16(byte reg) {
  uint8_t buffer[2];

  if (i2c_dev) {
    buffer[0] = uint8_t(reg);
    i2c_dev->write_then_read(buffer, 1, buffer, 2);
  } else {
    buffer[0] = uint8_t(reg | 0x80);
    spi_dev->write_then_read(buffer, 1, buffer, 2);
  }
  return uint16_t(buffer[0]) << 8 | uint16_t(buffer[1]);
}

uint16_t Adafruit_BMP280::read16_LE(byte reg) {
  uint16_t temp = read16(reg);
  return (temp >> 8) | (temp << 8);
}

/*!
 *   @brief  Reads a signed 16 bit value over I2C/SPI
 */
int16_t Adafruit_BMP280::readS16(byte reg) { return (int16_t)read16(reg); }

int16_t Adafruit_BMP280::readS16_LE(byte reg) {
  return (int16_t)read16_LE(reg);
}

/*!
 *  @brief  Reads a 24 bit value over I2C/SPI
 */
uint32_t Adafruit_BMP280::read24(byte reg) {
  uint8_t buffer[3];

  if (i2c_dev) {
    buffer[0] = uint8_t(reg);
    i2c_dev->write_then_read(buffer, 1, buffer, 3);
  } else {
    buffer[0] = uint8_t(reg | 0x80);
    spi_dev->write_then_read(buffer, 1, buffer, 3);
  }
  return uint32_t(buffer[0]) << 16 | uint32_t(buffer[1]) << 8 |
         uint32_t(buffer[2]);
}

/*!
 *  @brief  Reads the factory-set coefficients
 */
void Adafruit_BMP280::readCoefficients() {
  _bmp280_calib.dig_T1 = read16_LE(BMP280_REGISTER_DIG_T1);
  _bmp280_calib.dig_T2 = readS16_LE(BMP280_REGISTER_DIG_T2);
  _bmp280_calib.dig_T3 = readS16_LE(BMP280_REGISTER_DIG_T3);

  _bmp280_calib.dig_P1 = read16_LE(BMP280_REGISTER_DIG_P1);
  _bmp280_calib.dig_P2 = readS16_LE(BMP280_REGISTER_DIG_P2);
  _bmp280_calib.dig_P3 = readS16_LE(BMP280_REGISTER_DIG_P3);
  _bmp280_calib.dig_P4 = readS16_LE(BMP280_REGISTER_DIG_P4);
  _bmp280_calib.dig_P5 = readS16_LE(BMP280_REGISTER_DIG_P5);
  _bmp280_calib.dig_P6 = readS16_LE(BMP280_REGISTER_DIG_P6);
  _bmp280_calib.dig_P7 = readS16_LE(BMP280_REGISTER_DIG_P7);
  _bmp280_calib.dig_P8 = readS16_LE(BMP280_REGISTER_DIG_P8);
  _bmp280_calib.dig_P9 = readS16_LE(BMP280_REGISTER_DIG_P9);
}

/*!
 * Reads the temperature from the device.
 * @return The temperature in degrees celsius.
 */
float Adafruit_BMP280::readTemperature() {
  int32_t var1, var2;
  if (!_sensorID)
    return NAN; // begin() not called yet

  int32_t adc_T = read24(BMP280_REGISTER_TEMPDATA);
  adc_T >>= 4;

  var1 = ((((adc_T >> 3) - ((int32_t)_bmp280_calib.dig_T1 << 1))) *
          ((int32_t)_bmp280_calib.dig_T2)) >> 11;

  var2 = (((((adc_T >> 4) - ((int32_t)_bmp280_calib.dig_T1)) *
            ((adc_T >> 4) - ((int32_t)_bmp280_calib.dig_T1))) >> 12) *
          ((int32_t)_bmp280_calib.dig_T3)) >> 14;

  t_fine = var1 + var2;

  float T = (t_fine * 5 + 128) >> 8;
  return T / 100;
}

/*!
 * Reads the barometric pressure from the device.
 * @return Barometric pressure in Pa.
 */
float Adafruit_BMP280::readPressure() {
  int64_t var1, var2, p;
  if (!_sensorID)
    return NAN; // begin() not called yet

  // Must be done first to get the t_fine variable set up
  readTemperature();

  int32_t adc_P = read24(BMP280_REGISTER_PRESSUREDATA);
  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)_bmp280_calib.dig_P5) << 17);
  var2 = var2 + (((int64_t)_bmp280_calib.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3) >> 8) +
         ((var1 * (int64_t)_bmp280_calib.dig_P2) << 12);
  var1 =
      (((((int64_t)1) << 47) + var1)) * ((int64_t)_bmp280_calib.dig_P1) >> 33;

  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)_bmp280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7) << 4);
  return (float)p / 256;
}

/*!
 * @brief Calculates the approximate altitude using barometric pressure and the
 * supplied sea level hPa as a reference.
 * @param seaLevelhPa
 *        The current hPa at sea level.
 * @return The approximate altitude above sea level in meters.
 */
float Adafruit_BMP280::readAltitude(float seaLevelhPa) {
  float altitude;

  float pressure = readPressure(); // in Si units for Pascal
  pressure /= 100;

  altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

  return altitude;
}

/*!
 * Calculates the pressure at sea level (QNH) from the specified altitude,
 * and atmospheric pressure (QFE).
 * @param  altitude      Altitude in m
 * @param  atmospheric   Atmospheric pressure in hPa
 * @return The approximate pressure in hPa
 */
float Adafruit_BMP280::seaLevelForAltitude(float altitude, float atmospheric) {
  // Equation taken from BMP180 datasheet (page 17):
  // http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude.  See this thread for more information:
  // http://forums.adafruit.com/viewtopic.php?f=22&t=58064
  return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
}

/*!
    @brief  calculates the boiling point  of water by a given pressure
    @param pressure pressure in hPa
    @return temperature in °C
*/

float Adafruit_BMP280::waterBoilingPoint(float pressure) {
  // Magnusformular for calculation of the boiling point of water at a given
  // pressure
  return (234.175 * log(pressure / 6.1078)) /
         (17.08085 - log(pressure / 6.1078));
}

/*!
    @brief  Take a new measurement (only possible in forced mode)
    @return true if successful, otherwise false
 */
bool Adafruit_BMP280::takeForcedMeasurement() {
  // If we are in forced mode, the BME sensor goes back to sleep after each
  // measurement and we need to set it to forced mode once at this point, so
  // it will take the next measurement and then return to sleep again.
  // In normal mode simply does new measurements periodically.
  if (_measReg.mode == MODE_FORCED) {
    // set to forced mode, i.e. "take next measurement"
    write8(BMP280_REGISTER_CONTROL, _measReg.get());
    // wait until measurement has been completed, otherwise we would read
    // the values from the last measurement
    while (read8(BMP280_REGISTER_STATUS) & 0x08)
      delay(1);
    return true;
  }
  return false;
}

/*!
 *  @brief  Resets the chip via soft reset
 */
void Adafruit_BMP280::reset(void) {
  write8(BMP280_REGISTER_SOFTRESET, MODE_SOFT_RESET_CODE);
}

/*!
 *   Returns Sensor ID for diagnostics
 *   @returns 0x61 for BME680, 0x60 for BME280, 0x56, 0x57, 0x58 for BMP280
 */
uint8_t Adafruit_BMP280::sensorID(void) { return _sensorID; };

/*!
    @brief  Gets the most recent sensor event from the hardware status register.
    @return Sensor status as a byte.
 */
uint8_t Adafruit_BMP280::getStatus(void) {
  return read8(BMP280_REGISTER_STATUS);
}
