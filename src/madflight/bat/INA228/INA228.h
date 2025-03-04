//modified for madflight - see CHANGELOG.md

#pragma once
//    FILE: INA228.h
//  AUTHOR: Rob Tillaart
// VERSION: 0.1.4
//    DATE: 2024-05-09
// PURPOSE: Arduino library for INA228 voltage, current and power sensor.
//     URL: https://github.com/RobTillaart/INA228
//          https://www.adafruit.com/product/5832           ( 10 A version)
//          https://www.mateksys.com/?portfolio=i2c-ina-bm  (200 A version))
//
//
//  Read the datasheet for the details

#include "../../common/MF_I2C.h"

#define INA228_LIB_VERSION          (F("0.1.4"))

#define INA228_ADDRESS              (0x40) //default address

//  for setMode() and getMode()
enum ina228_mode_enum {
  INA228_MODE_SHUTDOWN            = 0x00,
  INA228_MODE_TRIG_BUS            = 0x01,
  INA228_MODE_TRIG_SHUNT          = 0x02,
  INA228_MODE_TRIG_BUS_SHUNT      = 0x03,
  INA228_MODE_TRIG_TEMP           = 0x04,
  INA228_MODE_TRIG_TEMP_BUS       = 0x05,
  INA228_MODE_TRIG_TEMP_SHUNT     = 0x06,
  INA228_MODE_TRIG_TEMP_BUS_SHUNT = 0x07,

  INA228_MODE_SHUTDOWN2           = 0x08,
  INA228_MODE_CONT_BUS            = 0x09,
  INA228_MODE_CONT_SHUNT          = 0x0A,
  INA228_MODE_CONT_BUS_SHUNT      = 0x0B,
  INA228_MODE_CONT_TEMP           = 0x0C,
  INA228_MODE_CONT_TEMP_BUS       = 0x0D,
  INA228_MODE_CONT_TEMP_SHUNT     = 0x0E,
  INA228_MODE_CONT_TEMP_BUS_SHUNT = 0x0F
};

//  for setAverage() and getAverage()
enum ina228_average_enum {
    INA228_1_SAMPLE     = 0,
    INA228_4_SAMPLES    = 1,
    INA228_16_SAMPLES   = 2,
    INA228_64_SAMPLES   = 3,
    INA228_128_SAMPLES  = 4,
    INA228_256_SAMPLES  = 5,
    INA228_512_SAMPLES  = 6,
    INA228_1024_SAMPLES = 7
};

//  for Bus, shunt and temperature conversion timing.
enum ina228_timing_enum {
    INA228_50_us   = 0,
    INA228_84_us   = 1,
    INA228_150_us  = 2,
    INA228_280_us  = 3,
    INA228_540_us  = 4,
    INA228_1052_us = 5,
    INA228_2074_us = 6,
    INA228_4120_us = 7
};

//  for diagnose/alert() bit fields.
//  TODO bit masks?
enum ina228_diag_enum {
  INA228_DIAG_MEMORY_STATUS      = 0,
  INA228_DIAG_CONVERT_COMPLETE   = 1,
  INA228_DIAG_POWER_OVER_LIMIT   = 2,
  INA228_DIAG_BUS_UNDER_LIMIT    = 3,
  INA228_DIAG_BUS_OVER_LIMIT     = 4,
  INA228_DIAG_SHUNT_UNDER_LIMIT  = 5,
  INA228_DIAG_SHUNT_OVER_LIMIT   = 6,
  INA228_DIAG_TEMP_OVER_LIMIT    = 7,
  INA228_DIAG_RESERVED           = 8,
  INA228_DIAG_MATH_OVERFLOW      = 9,
  INA228_DIAG_CHARGE_OVERFLOW    = 10,
  INA228_DIAG_ENERGY_OVERFLOW    = 11,
  INA228_DIAG_ALERT_POLARITY     = 12,
  INA228_DIAG_SLOW_ALERT         = 13,
  INA228_DIAG_CONVERT_READY      = 14,
  INA228_DIAG_ALERT_LATCH        = 15
};


class INA228
{
public:
  bool     begin(MF_I2C *wire, uint8_t address = INA228_ADDRESS);
  bool     isConnected();
  uint8_t  getAddress();

  bool isConversionReady();

  //
  //  CORE FUNCTIONS + scale wrappers.
  //
  //       BUS VOLTAGE
  float    getBusVoltage();     //  Volt
  float    getBusVolt()         { return getBusVoltage(); };
  float    getBusMilliVolt()    { return getBusVoltage()   * 1e3; };
  float    getBusMicroVolt()    { return getBusVoltage()   * 1e6; };

  //       SHUNT VOLTAGE
  float    getShuntVoltage();   //  Volt
  float    getShuntVolt()       { return getShuntVoltage(); };
  float    getShuntMilliVolt()  { return getShuntVoltage() * 1e3; };
  float    getShuntMicroVolt()  { return getShuntVoltage() * 1e6; };

  //       SHUNT CURRENT
  float    getCurrent();        //  Ampere
  float    getAmpere()          { return getCurrent(); };
  float    getMilliAmpere()     { return getCurrent()      * 1e3; };
  float    getMicroAmpere()     { return getCurrent()      * 1e6; };

  //       POWER
  float    getPower();          //  Watt
  float    getWatt()            { return getPower(); };
  float    getMilliWatt()       { return getPower()        * 1e3; };
  float    getMicroWatt()       { return getPower()        * 1e6; };
  float    getKiloWatt()        { return getPower()        * 1e-3; };

  //       TEMPERATURE
  float    getTemperature();    //  Celsius

  //  the Energy and Charge functions are returning double as they have higher accuracy.
  //       ENERGY
  double   getEnergy();         //  Joule or watt second
  double   getJoule()           { return getEnergy(); };
  double   getMegaJoule()       { return getEnergy()       * 1e-6; };
  double   getKiloJoule()       { return getEnergy()       * 1e-3; };
  double   getMilliJoule()      { return getEnergy()       * 1e3; };
  double   getMicroJoule()      { return getEnergy()       * 1e6; };
  double   getWattHour()        { return getEnergy()       * (1.0 / 3600.0); };
  double   getKiloWattHour()    { return getEnergy()       * (1.0 / 3.6); };

  //  CHARGE
  double   getCharge();         //  Coulombs
  double   getCoulomb()         { return getCharge(); };
  double   getMilliCoulomb()    { return getCharge()       * 1e3; };
  double   getMicroCoulomb()    { return getCharge()       * 1e6; };

  //
  //  CONFIG REGISTER 0
  //  read datasheet for details, section 7.6.1.1, page 22
  //
  void     reset();
  //  value: 0 == normal operation,  1 = clear registers
  bool     setAccumulation(uint8_t value);
  bool     getAccumulation();
  //  Conversion delay in 0..255 steps of 2 ms
  void     setConversionDelay(uint8_t steps);
  uint8_t  getConversionDelay();
  void     setTemperatureCompensation(bool on);
  bool     getTemperatureCompensation();
  //  flag = false => 164 mV, true => 41 mV
  void     setADCRange(bool flag);
  bool     getADCRange();

  //
  //  CONFIG ADC REGISTER 1
  //  read datasheet for details, section 7.6.1.2, page 22++
  //
  bool     setMode(uint8_t mode = INA228_MODE_CONT_TEMP_BUS_SHUNT);
  uint8_t  getMode();
  //  default value = ~1 milliseconds for all.
  bool     setBusVoltageConversionTime(uint8_t bvct = INA228_1052_us);
  uint8_t  getBusVoltageConversionTime();
  bool     setShuntVoltageConversionTime(uint8_t svct = INA228_1052_us);
  uint8_t  getShuntVoltageConversionTime();
  bool     setTemperatureConversionTime(uint8_t tct = INA228_1052_us);
  uint8_t  getTemperatureConversionTime();
  bool     setAverage(uint8_t avg = INA228_1_SAMPLE);
  uint8_t  getAverage();

  //
  //  SHUNT CALIBRATION REGISTER 2
  //  read datasheet for details. use with care.
  //  maxCurrent <= 204, (in fact no limit)
  //  shunt >= 0.0001.
  //  returns _current_LSB;
  void     calibrate(float shunt, float maxCurrent = 0);
  bool     isCalibrated()    { return _current_LSB != 0.0; };
  float    getMaxCurrent();
  float    getShunt();
  float    getCurrentLSB();

  //
  //  SHUNT TEMPERATURE COEFFICIENT REGISTER 3
  //  read datasheet for details, page 16.
  //  ppm = 0..16383.
  bool     setShuntTemperatureCoefficent(uint16_t ppm = 0);
  uint16_t getShuntTemperatureCoefficent();

  //
  //  DIAGNOSE ALERT REGISTER 11  (0x0B)
  //  read datasheet for details, section 7.6.1.12, page 26++.
  //
  void     setDiagnoseAlert(uint16_t flags);
  uint16_t getDiagnoseAlert();
  //  INA228.h has an enum for the bit fields.
  //  See ina228_diag_enum above
  void     setDiagnoseAlertBit(uint8_t bit);
  void     clearDiagnoseAlertBit(uint8_t bit);
  uint16_t getDiagnoseAlertBit(uint8_t bit);

  //
  //  THRESHOLD AND LIMIT REGISTERS 12-17
  //  read datasheet for details, section 7.3.7, page 16++
  //
  //  TODO - design and implement better API?
  //
  void     setShuntOvervoltageTH(uint16_t threshold);
  uint16_t getShuntOvervoltageTH();
  void     setShuntUndervoltageTH(uint16_t threshold);
  uint16_t getShuntUndervoltageTH();
  void     setBusOvervoltageTH(uint16_t threshold);
  uint16_t getBusOvervoltageTH();
  void     setBusUndervoltageTH(uint16_t threshold);
  uint16_t getBusUndervoltageTH();
  void     setTemperatureOverLimitTH(uint16_t threshold);
  uint16_t getTemperatureOverLimitTH();
  void     setPowerOverLimitTH(uint16_t threshold);
  uint16_t getPowerOverLimitTH();

  //
  //  MANUFACTURER and ID REGISTER 3E and 3F
  //
  //                               typical value
  uint16_t getManufacturer();  //  0x5449
  uint16_t getDieID();         //  0x0228
  uint16_t getRevision();      //  0x0001

private:
  //  max 4 bytes
  uint32_t _readRegister(uint8_t reg, uint8_t bytes);
  //  5 bytes or more
  double   _readRegisterF(uint8_t reg, uint8_t bytes);
  uint16_t _writeRegister(uint8_t reg, uint16_t value);

  float    _current_LSB = 0;
  float    _shunt = 0;
  float    _maxCurrent = 0;

  uint8_t   _address = 0;
  MF_I2C * _wire;
  
  bool _ADCRange = false;
};

//  -- END OF FILE --


//modified for madflight - see CHANGELOG.md

//    FILE: INA228.cpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.1.4
//    DATE: 2024-05-09
// PURPOSE: Arduino library for INA228 voltage, current and power sensor.
//     URL: https://github.com/RobTillaart/INA228
//          https://www.adafruit.com/product/5832           ( 10 A version)
//          https://www.mateksys.com/?portfolio=i2c-ina-bm  (200 A version))

//
//  Read the datasheet for the details

#include "INA228.h"

//      REGISTERS                   ADDRESS    BITS  RW
#define INA228_CONFIG               0x00    //  16   RW
#define INA228_ADC_CONFIG           0x01    //  16   RW
#define INA228_SHUNT_CAL            0x02    //  16   RW
#define INA228_SHUNT_TEMP_CO        0x03    //  16   RW
#define INA228_SHUNT_VOLTAGE        0x04    //  24   R-
#define INA228_BUS_VOLTAGE          0x05    //  24   R-
#define INA228_TEMPERATURE          0x06    //  16   R-
#define INA228_CURRENT              0x07    //  24   R-
#define INA228_POWER                0x08    //  24   R-
#define INA228_ENERGY               0x09    //  40   R-
#define INA228_CHARGE               0x0A    //  40   R-
#define INA228_DIAG_ALERT           0x0B    //  16   RW
#define INA228_SOVL                 0x0C    //  16   RW
#define INA228_SUVL                 0x0D    //  16   RW
#define INA228_BOVL                 0x0E    //  16   RW
#define INA228_BUVL                 0x0F    //  16   RW
#define INA228_TEMP_LIMIT           0x10    //  16   RW
#define INA228_POWER_LIMIT          0x11    //  16   RW
#define INA228_MANUFACTURER         0x3E    //  16   R-
#define INA228_DEVICE_ID            0x3F    //  16   R-

//  CONFIG MASKS (register 0)
#define INA228_CFG_RST              0x8000
#define INA228_CFG_RSTACC           0x4000
#define INA228_CFG_CONVDLY          0x3FC0
#define INA228_CFG_TEMPCOMP         0x0020
#define INA228_CFG_ADCRANGE         0x0010
#define INA228_CFG_RESERVED         0x000F

//  ADC MASKS (register 1)
#define INA228_ADC_MODE             0xF000
#define INA228_ADC_VBUSCT           0x0E00
#define INA228_ADC_VSHCT            0x01C0
#define INA228_ADC_VTCT             0x0038
#define INA228_ADC_AVG              0x0007

// DIAG_ALERT register
#define INA228_ALATCH               0x8000
#define INA228_CNVRF                0x0002

////////////////////////////////////////////////////////
//
//  CONSTRUCTOR
//
bool INA228::begin(MF_I2C *wire, uint8_t address)
{
  _address     = address;
  _wire        = wire;

  if (! isConnected()) return false;

  //set ALATCH
  uint16_t value = _readRegister(INA228_DIAG_ALERT, 2);
  value |= INA228_ALATCH;
  _writeRegister(INA228_DIAG_ALERT, value);

  return true;
}


bool INA228::isConnected()
{
  _wire->beginTransmission(_address);
  return ( _wire->endTransmission() == 0);
}


uint8_t INA228::getAddress()
{
  return _address;
}


bool INA228::isConversionReady() {
  uint16_t value = _readRegister(INA228_DIAG_ALERT, 2);
  return (value && INA228_CNVRF != 0); //INA228_CNVRF is cleared on reading DIAG_ALERT when ALATCH=1
}

////////////////////////////////////////////////////////
//
//  CORE FUNCTIONS
//
//  PAGE 25
float INA228::getBusVoltage()
{
  //  always positive, remove reserved bits.
  int32_t value = _readRegister(INA228_BUS_VOLTAGE, 3) >> 4;
  float bus_LSB = 195.3125e-6;  //  195.3125 uV
  float voltage = value * bus_LSB;
  return voltage;
}

//  PAGE 25
float INA228::getShuntVoltage()
{
  //  shunt_LSB depends on ADCRANGE in INA228_CONFIG register.
  float shunt_LSB = 312.5e-9;  //  312.5 nV
  if (_ADCRange)
  {
    shunt_LSB = 78.125e-9;     //  78.125 nV
  }

  //  remove reserved bits.
  int32_t value = _readRegister(INA228_SHUNT_VOLTAGE, 3) >> 4;
  //  handle negative values (20 bit)
  if (value & 0x00080000)
  {
    value |= 0xFFF00000;
  }
  float voltage = value * shunt_LSB;
  return voltage;
}

//  PAGE 25 + 8.1.2
float INA228::getCurrent()
{
  //  remove reserved bits.
  int32_t value = _readRegister(INA228_CURRENT, 3) >> 4;
  //  handle negative values (20 bit)
  if (value & 0x00080000)
  {
    value |= 0xFFF00000;
  }
  float current = value * _current_LSB;
  return current;
}

//  PAGE 26 + 8.1.2
float INA228::getPower()
{
  uint32_t value = _readRegister(INA228_POWER, 3) ;
  //  PAGE 31 (8.1.2)
  return value * 3.2 * _current_LSB;
}

//  PAGE 25
float INA228::getTemperature()
{
  uint32_t value = _readRegister(INA228_TEMPERATURE, 2);
  float LSB = 7.8125e-3;  //   milli degree Celsius
  return value * LSB;
}

//  PAGE 26 + 8.1.2
double INA228::getEnergy()
{
  //  read 40 bit unsigned as a double to prevent 64 bit ints
  //  double might be 8 or 4 byte, depends on platform
  //  40 bit ==> O(10^12)
  double value = _readRegisterF(INA228_ENERGY, 5);
  //  PAGE 31 (8.1.2)
  return value * (16 * 3.2) * _current_LSB;
}


//  PAGE 26 + 8.1.2
double INA228::getCharge()
{
  //  read 40 bit unsigned as a float to prevent 64 bit ints
  //  double might be 8 or 4 byte, depends on platform
  //  40 bit ==> O(10^12)
  double value = _readRegisterF(INA228_CHARGE, 5);
  //  PAGE 32 (8.1.2)
  return value * _current_LSB;
}


////////////////////////////////////////////////////////
//
//  CONFIG REGISTER 0
//
void INA228::reset()
{
  uint16_t value = _readRegister(INA228_CONFIG, 2);
  value |= INA228_CFG_RST;
  _writeRegister(INA228_CONFIG, value);
}

bool INA228::setAccumulation(uint8_t value)
{
  if (value > 1) return false;
  uint16_t reg = _readRegister(INA228_CONFIG, 2);
  reg &= ~INA228_CFG_RSTACC;
  if (value == 1) reg |= INA228_CFG_RSTACC;
  _writeRegister(INA228_CONFIG, reg);
  return true;
}

bool INA228::getAccumulation()
{
  uint16_t value = _readRegister(INA228_CONFIG, 2);
  return (value & INA228_CFG_RSTACC) > 0;
}

void INA228::setConversionDelay(uint8_t steps)
{
  uint16_t value = _readRegister(INA228_CONFIG, 2);
  value &= ~INA228_CFG_CONVDLY;
  value |= (steps << 6);
  _writeRegister(INA228_CONFIG, value);
}

uint8_t INA228::getConversionDelay()
{
  uint16_t value = _readRegister(INA228_CONFIG, 2);
  return (value >> 6) & 0xFF;
}

void INA228::setTemperatureCompensation(bool on)
{
  uint16_t value = _readRegister(INA228_CONFIG, 2);
  value &= ~INA228_CFG_TEMPCOMP;
  if (on) value |= INA228_CFG_TEMPCOMP;
  _writeRegister(INA228_CONFIG, value);
}

bool INA228::getTemperatureCompensation()
{
  uint16_t value = _readRegister(INA228_CONFIG, 2);
  return (value & INA228_CFG_TEMPCOMP) > 0;
}

void INA228::setADCRange(bool flag)
{
  uint16_t value = _readRegister(INA228_CONFIG, 2);
  value &= ~INA228_CFG_ADCRANGE;
  if (flag) value |= INA228_CFG_ADCRANGE;
  _writeRegister(INA228_CONFIG, value);
  _ADCRange = flag;
}

bool INA228::getADCRange()
{
  uint16_t value = _readRegister(INA228_CONFIG, 2);
  _ADCRange = ((value & INA228_CFG_ADCRANGE) != 0);
  return _ADCRange;
}


////////////////////////////////////////////////////////
//
//  CONFIG ADC REGISTER 1
//
bool INA228::setMode(uint8_t mode)
{
  if (mode > 0x0F) return false;
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  value &= ~INA228_ADC_MODE;
  value |= (mode << 12);
  _writeRegister(INA228_ADC_CONFIG, value);
  return true;
}

uint8_t INA228::getMode()
{
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  return (value & INA228_ADC_MODE) >> 12;
}

bool INA228::setBusVoltageConversionTime(uint8_t bvct)
{
  if (bvct > 7) return false;
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  value &= ~INA228_ADC_VBUSCT;
  value |= (bvct << 9);
  _writeRegister(INA228_ADC_CONFIG, value);
  return true;
}

uint8_t INA228::getBusVoltageConversionTime()
{
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  return (value & INA228_ADC_VBUSCT) >> 9;
}

bool INA228::setShuntVoltageConversionTime(uint8_t svct)
{
  if (svct > 7) return false;
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  value &= ~INA228_ADC_VSHCT;
  value |= (svct << 6);
  _writeRegister(INA228_ADC_CONFIG, value);
  return true;
}

uint8_t INA228::getShuntVoltageConversionTime()
{
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  return (value & INA228_ADC_VSHCT) >> 6;
}

bool INA228::setTemperatureConversionTime(uint8_t tct)
{
  if (tct > 7) return false;
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  value &= ~INA228_ADC_VTCT;
  value |= (tct << 3);
  _writeRegister(INA228_ADC_CONFIG, value);
  return true;
}

uint8_t INA228::getTemperatureConversionTime()
{
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  return (value & INA228_ADC_VTCT) >> 3;
}

bool INA228::setAverage(uint8_t avg)
{
  if (avg > 7) return false;
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  value &= ~INA228_ADC_AVG;
  value |= avg;
  _writeRegister(INA228_ADC_CONFIG, value);
  return true;
}

uint8_t INA228::getAverage()
{
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  return (value & INA228_ADC_AVG);
}


////////////////////////////////////////////////////////
//
//  SHUNT CALIBRATION REGISTER 2
//
void INA228::calibrate(float rShuntValue, float iMaxExpected)
{
  _shunt = rShuntValue;
  _maxCurrent = iMaxExpected;
  _current_LSB = _maxCurrent * 1.9073486328125e-6;  //  pow(2, -19);

  //  PAGE 31 (8.1.2)
  float shunt_cal = 13107.2e6 * _current_LSB * _shunt;
  //  depends on ADCRANGE in INA228_CONFIG register.
  if (_ADCRange)
  {
    shunt_cal *= 4;
  }
  //rounding
  shunt_cal += 0.5;
  //  shunt_cal must be written to REGISTER.
  //  work in progress PR #7
  _writeRegister(INA228_SHUNT_CAL, shunt_cal);
}


float INA228::getMaxCurrent()
{
  return _maxCurrent;
}

float INA228::getShunt()
{
  return _shunt;
}

float INA228::getCurrentLSB()
{
  return _current_LSB;
}

////////////////////////////////////////////////////////
//
//  SHUNT TEMPERATURE COEFFICIENT REGISTER 3
//
bool INA228::setShuntTemperatureCoefficent(uint16_t ppm)
{
  if (ppm > 16383) return false;
  _writeRegister(INA228_SHUNT_TEMP_CO, ppm);
  return true;
}

uint16_t INA228::getShuntTemperatureCoefficent()
{
  uint16_t value = _readRegister(INA228_SHUNT_TEMP_CO, 2);
  return value;
}



////////////////////////////////////////////////////////
//
//  DIAGNOSE ALERT REGISTER 11
//
void INA228::setDiagnoseAlert(uint16_t flags)
{
  _writeRegister(INA228_DIAG_ALERT, flags);
}

uint16_t INA228::getDiagnoseAlert()
{
  return _readRegister(INA228_DIAG_ALERT, 2);
}

//  INA228.h has an enum for the bit fields.
void INA228::setDiagnoseAlertBit(uint8_t bit)
{
  uint16_t value = _readRegister(INA228_DIAG_ALERT, 2);
  uint16_t mask = (1 << bit);
  //  only write new value if needed.
  if ((value & mask) == 0)
  {
    value |= mask;
    _writeRegister(INA228_DIAG_ALERT, value);
  }
}

void INA228::clearDiagnoseAlertBit(uint8_t bit)
{
  uint16_t value = _readRegister(INA228_DIAG_ALERT, 2);
  uint16_t mask = (1 << bit);
  //  only write new value if needed.
  if ((value & mask ) != 0)
  {
    value &= ~mask;
    _writeRegister(INA228_DIAG_ALERT, value);
  }
}

uint16_t INA228::getDiagnoseAlertBit(uint8_t bit)
{
  uint16_t value = _readRegister(INA228_DIAG_ALERT, 2);
  return (value >> bit) & 0x01;
}


////////////////////////////////////////////////////////
//
//  THRESHOLD AND LIMIT REGISTERS 12-17
//
//  TODO - API ?

void INA228::setShuntOvervoltageTH(uint16_t threshold)
{
  //  TODO ADCRANGE DEPENDENT
  _writeRegister(INA228_SOVL, threshold);
}

uint16_t INA228::getShuntOvervoltageTH()
{
  //  TODO ADCRANGE DEPENDENT
  return _readRegister(INA228_SOVL, 2);
}

void INA228::setShuntUndervoltageTH(uint16_t threshold)
{
  //  TODO ADCRANGE DEPENDENT
  _writeRegister(INA228_SUVL, threshold);
}

uint16_t INA228::getShuntUndervoltageTH()
{
  //  TODO ADCRANGE DEPENDENT
  return _readRegister(INA228_SUVL, 2);
}

void INA228::setBusOvervoltageTH(uint16_t threshold)
{
  if (threshold > 0x7FFF) return;
  //float LSB = 3.125e-3;
  _writeRegister(INA228_BOVL, threshold);
}

uint16_t INA228::getBusOvervoltageTH()
{
  //float LSB = 3.125e-3;
  return _readRegister(INA228_BOVL, 2);
}

void INA228::setBusUndervoltageTH(uint16_t threshold)
{
  if (threshold > 0x7FFF) return;
  //float LSB = 3.125e-3;
  _writeRegister(INA228_BUVL, threshold);
}

uint16_t INA228::getBusUndervoltageTH()
{
  //float LSB = 3.125e-3;
  return _readRegister(INA228_BUVL, 2);
}

void INA228::setTemperatureOverLimitTH(uint16_t threshold)
{
  //float LSB = 7.8125e-3;  //  milliCelsius
  _writeRegister(INA228_TEMP_LIMIT, threshold);
}

uint16_t INA228::getTemperatureOverLimitTH()
{
  //float LSB = 7.8125e-3;  //  milliCelsius
  return _readRegister(INA228_TEMP_LIMIT, 2);
}

void INA228::setPowerOverLimitTH(uint16_t threshold)
{
  //  P29
  //  Conversion factor: 256 × Power LSB.
  _writeRegister(INA228_POWER_LIMIT, threshold);
}

uint16_t INA228::getPowerOverLimitTH()
{
  //  P29
  //  Conversion factor: 256 × Power LSB.
  return _readRegister(INA228_POWER_LIMIT, 2);
}


////////////////////////////////////////////////////////
//
//  MANUFACTURER and ID REGISTER 3E/3F
//
uint16_t INA228::getManufacturer()
{
  uint16_t value = _readRegister(INA228_MANUFACTURER, 2);
  return value;
}

uint16_t INA228::getDieID()
{
  uint16_t value = _readRegister(INA228_DEVICE_ID, 2);
  return (value >> 4) & 0x0FFF;
}

uint16_t INA228::getRevision()
{
  uint16_t value = _readRegister(INA228_DEVICE_ID, 2);
  return value & 0x000F;
}


////////////////////////////////////////////////////////
//
//  SHOULD BE PRIVATE
//
uint32_t INA228::_readRegister(uint8_t reg, uint8_t bytes)
{
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->endTransmission();

  _wire->requestFrom(_address, (uint8_t)bytes);
  uint32_t value = 0;
  for (int i = 0; i < bytes; i++)
  {
    value <<= 8;
    value |= _wire->read();
  }
  return value;
}


double INA228::_readRegisterF(uint8_t reg, uint8_t bytes)
{
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->endTransmission();

  _wire->requestFrom(_address, (uint8_t)bytes);
  double value = 0;
  for (int i = 0; i < bytes; i++)
  {
    value *= 256.0;
    value += _wire->read();
  }
  return value;
}


uint16_t INA228::_writeRegister(uint8_t reg, uint16_t value)
{
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->write(value >> 8);
  _wire->write(value & 0xFF);
  return _wire->endTransmission();
}


//  -- END OF FILE --

