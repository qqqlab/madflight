/**
 * @file  BMM150.h
 * @brief Defines the infrastructure of the BMM150 class
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      [ZhixinLiu](zhixin.liu@dfrobot.com)
 * @version     V1.0.0
 * @date        2021-04-21
 * @url         https://github.com/DFRobot/BMM150
 */
#ifndef __BMM150_H__
#define __BMM150_H__
#include "BMM150_defs.h"
#include "Arduino.h"
#include <stdlib.h>
#include <SPI.h>
#include "../../hal/MF_I2C.h"
#include <math.h>


//#define ENABLE_DBG                //< Open this macro to see the program running in detail

#ifdef ENABLE_DBG
#define DBG(...) {Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
#define DBG(...)
#endif

class BMM150{
public:
  BMM150();
  virtual ~BMM150();

  /**
   * @fn softReset
   * @brief Soft reset, restore to suspended mode after soft reset and then enter sleep mode, soft reset can't be implemented under suspend mode
   */
  void softReset(void);

  /**
   * @fn setOperationMode
   * @brief Set sensor operation mode
   * @param opMode mode
   * @n BMM150_POWERMODE_NORMAL      normal mode  Get geomagnetic data normally
   * @n BMM150_POWERMODE_FORCED      forced mode  Single measurement, the sensor restores to sleep mode when the measurement is done.
   * @n BMM150_POWERMODE_SLEEP       sleep mode   Users can visit all the registers, but can’t measure geomagnetic data
   * @n BMM150_POWERMODE_SUSPEND     suspend mode At the time the sensor cpu doesn’t work and can’t implement any operation. Users can only visit the content of the control register BMM150_REG_POWER_CONTROL
   */
  void setOperationMode(uint8_t opMode);

  /**
   * @fn getOperationMode
   * @brief Get sensor operation mode
   * @return result Return sensor operation mode as a character string
   */
  String getOperationMode(void);

  /**
   * @fn setPresetMode
   * @brief Set preset mode, make it easier for users to configure sensor to get geomagnetic data 
   * @param presetMode
   * @n BMM150_PRESETMODE_LOWPOWER       Low power mode, get a fraction of data and take the mean value.
   * @n BMM150_PRESETMODE_REGULAR        Regular mode, get a number of data and take the mean value.
   * @n BMM150_PRESETMODE_ENHANCED       Enhanced mode, get a plenty of data and take the mean value.
   * @n BMM150_PRESETMODE_HIGHACCURACY   High accuracy mode, get a huge number of data and take the mean value.
   */
  void setPresetMode(uint8_t presetMode);

  /**
   * @fn setRate
   * @brief Set the rate of obtaining geomagnetic data, the higher, the faster (without delay function)
   * @param rate
   * @n BMM150_DATA_RATE_02HZ
   * @n BMM150_DATA_RATE_06HZ
   * @n BMM150_DATA_RATE_08HZ
   * @n BMM150_DATA_RATE_10HZ  (default rate)
   * @n BMM150_DATA_RATE_15HZ
   * @n BMM150_DATA_RATE_20HZ
   * @n BMM150_DATA_RATE_25HZ
   * @n BMM150_DATA_RATE_30HZ
   */
  void setRate(uint8_t rate);

  /**
   * @fn getRate
   * @brief Get the config data rate, unit: HZ
   * @return rate
   */
  uint8_t getRate(void);

  /**
   * @fn getGeomagneticData
   * @brief Get the geomagnetic data of 3 axis (x, y, z)
   * @return Geomagnetic data structure, unit: (uT)
   */
  sBmm150MagData_t getGeomagneticData(void);

  /**
   * @fn getCompassDegree
   * @brief Get compass degree
   * @return Compass degree (0° - 360°)
   * @n      0° = North, 90° = East, 180° = South, 270° = West.
   */
  float getCompassDegree(void);

  /**
   * @fn setDataReadyPin
   * @brief Enable or disable data ready interrupt pin
   * @n After enabling, the DRDY pin jump when there's data coming.
   * @n After disabling, the DRDY pin will not jump when there's data coming.
   * @n High polarity: active on high, the default is low level, which turns to high level when the interrupt is triggered.
   * @n Low polarity: active on low, default is high level, which turns to low level when the interrupt is triggered.
   * @param modes
   * @n     DRDY_ENABLE        Enable DRDY
   * @n     DRDY_DISABLE       Disable DRDY
   * @param polarity
   * @n     POLARITY_HIGH      High polarity
   * @n     POLARITY_LOW       Low polarity
   */
  void setDataReadyPin(uint8_t modes, uint8_t polarity=POLARITY_HIGH);

  /**
   * @fn getDataReadyState
   * @brief Get the data ready status, determine whether the data is ready
   * @return status
   * @n true  Data ready
   * @n false Data is not ready
   */
  bool getDataReadyState(void);

  /**
   * @fn setMeasurementXYZ
   * @brief Enable the measurement at x-axis, y-axis and z-axis, default to be enabled. After disabling, the geomagnetic data at x, y, and z axis are wrong.
   * @param channelX
   * @n   MEASUREMENT_X_ENABLE        Enable the measurement at x-axis
   * @n   MEASUREMENT_X_DISABLE       Disable the measurement at x-axis
   * @param channelY
   * @n   MEASUREMENT_Y_ENABLE        Enable the measurement at y-axis
   * @n   MEASUREMENT_Y_DISABLE       Disable the measurement at y-axis
   * @param channelZ
   * @n   MEASUREMENT_Z_ENABLE        Enable the measurement at z-axis
   * @n   MEASUREMENT_Z_DISABLE       Disable the measurement at z-axis
   */
  void setMeasurementXYZ(uint8_t channelX = MEASUREMENT_X_ENABLE, uint8_t channelY = MEASUREMENT_Y_ENABLE, uint8_t channelZ = MEASUREMENT_Z_ENABLE);

  /**
   * @fn getMeasurementStateXYZ
   * @brief Get the enabling status at x-axis, y-axis and z-axis
   * @return result Return enabling status as a character string
   */
  String getMeasurementStateXYZ(void);

  /**
   * @fn setThresholdInterrupt(uint8_t modes, int8_t threshold, uint8_t polarity)
   * @brief Set threshold interrupt, an interrupt is triggered when the geomagnetic value of a channel is beyond/below the threshold
   * @n      High polarity: active on high level, the default is low level, which turns to high level when the interrupt is triggered.
   * @n      Low polarity: active on low level, the default is high level, which turns to low level when the interrupt is triggered.
   * @param modes
   * @n     LOW_THRESHOLD_INTERRUPT       Low threshold interrupt mode
   * @n     HIGH_THRESHOLD_INTERRUPT      High threshold interrupt mode
   * @param  threshold
   * @n     Threshold, default to expand 16 times, for example: under low threshold mode, if the threshold is set to be 1, actually the geomagnetic data below 16 will trigger an interrupt
   * @param polarity
   * @n     POLARITY_HIGH      High polarity
   * @n     POLARITY_LOW       Low polarity
   */
  void setThresholdInterrupt(uint8_t modes, int8_t threshold, uint8_t polarity);

  /**
   * @fn setThresholdInterrupt(uint8_t modes, uint8_t channelX, uint8_t channelY, uint8_t channelZ, int8_t threshold, uint8_t polarity)
   * @brief Set threshold interrupt, an interrupt is triggered when the geomagnetic value of a channel is beyond/below the threshold
   * @n   When an interrupt occurs, INT pin level will jump
   * @n   High polarity: active on high level, the default is low level, which turns to high level when the interrupt is triggered.
   * @n   Low polarity: active on low level, the default is high level, which turns to low level when the interrupt is triggered.
   * @param modes
   * @n   LOW_THRESHOLD_INTERRUPT   Low threshold interrupt mode
   * @n   HIGH_THRESHOLD_INTERRUPT  High threshold interrupt mode
   * @param channelX
   * @n   INTERRUPT_X_ENABLE        Enable high threshold interrupt at x-axis
   * @n   INTERRUPT_X_DISABLE       Disable high threshold interrupt at x-axis
   * @param channelY
   * @n   INTERRUPT_Y_ENABLE         Enable high threshold interrupt at y-axis
   * @n   INTERRUPT_Y_DISABLE        Disable high threshold interrupt at y-axis
   * @param channelZ
   * @n   INTERRUPT_Z_ENABLE         Enable high threshold interrupt at z-axis
   * @n   INTERRUPT_Z_DISABLE        Disable high threshold interrupt at z-axis
   * @param  threshold
   * @n   Threshold, default to expand 16 times, for example: if the threshold is set to be 1, actually the geomagnetic data below 16 will trigger an interrupt
   * @param polarity
   * @n     POLARITY_HIGH      High polarity
   * @n     POLARITY_LOW       Low polarity
   */
  void setThresholdInterrupt(uint8_t modes, uint8_t channelX, uint8_t channelY, uint8_t channelZ, int8_t threshold, uint8_t polarity);

  /**
   * @fn getThresholdData
   * @brief Get the data with threshold interrupt occurred
   * @return Returns the structure for storing geomagnetic data, the structure stores the data of 3 axis and interrupt status,
   * @n The interrupt is not triggered when the data at x-axis, y-axis and z-axis are NO_DATA
   * @n String state The storage state is binary data string
   * @n uint8_t value The storage state is binary raw value, the data format are as follows:
   * @n bit0 is 1 Indicate the interrupt occur at x-axis
   * @n bit1 is 1 Indicate the interrupt occur at y-axis
   * @n bit2 is 1 Indicate the interrupt occur at z-axis
   * @n ------------------------------------
   * @n | bit7 ~ bit3 | bit2 | bit1 | bit0 |
   * @n ------------------------------------
   * @n |  reserved   |  0   |  0   |  0   |
   * @n ------------------------------------
   */
  sBmm150ThresholdData_t getThresholdData(void);

  /**
   * @fn selfTest
   * @brief The sensor self test, the returned value indicate the self test result.
   * @param testMode:
   * @n    BMM150_SELF_TEST_NORMAL               Normal self test, test whether x-axis, y-axis and z-axis are connected or short-circuited
   * @n    BMM150_SELF_TEST_ADVANCED             Advanced self test, test the data accuracy at z-axis
   * @return result The returned character string is the self test result
   */
  String selfTest(uint8_t testMode);
  sBmm150Trim_t _trimData;
protected:

  /**
   * @fn sensorInit
   * @brief Init bmm150 check whether the chip id is right
   * @return state
   * @n     true is   Chip id is right init succeeds
   * @n     false is  Chip id is wrong init failed
   */
  bool sensorInit(void);

  /**
   * @fn binChangeString
   * @brief Convert binary value to character string
   * @param value 
   * @return The character string with the same expression as the binary
   */
  String binChangeString(uint8_t value);

  /**
   * @fn setLowThresholdInterrupt
   * @brief Set low threshold interrupt, an interrupt is triggered when the geomagnetic value of a channel is below the low threshold, the threshold is default to expand 16 times
   * @n   When an interrupt occurs, INT pin level will jump
   * @n   High polarity: active on high level, the default is low level, which will jump when the threshold is triggered.
   * @n   Low polarity: active on low level, the default is high level, which will jump when the threshold is triggered.
   * @param channelX
   * @n    INTERRUPT_X_ENABLE        Enable low threshold interrupt at x-axis
   * @n    INTERRUPT_X_DISABLE       Disable low threshold interrupt at x-axis
   * @param channelY
   * @n    INTERRUPT_Y_ENABLE        Enable low threshold interrupt at y-axis
   * @n    INTERRUPT_Y_DISABLE       Disable low threshold interrupt at y-axis
   * @param channelZ
   * @n    INTERRUPT_Z_ENABLE        Enable low threshold interrupt at z-axis
   * @n    INTERRUPT_Z_DISABLE       Disable low threshold interrupt at z-axis
   * @param  lowThreshold
   * @n    Low threshold, default to expand 16 times, for example: if the threshold is set to be 1, actually the geomagnetic data below 16 will trigger an interrupt
   * @param polarity
   * @n    POLARITY_HIGH      High polarity
   * @n    POLARITY_LOW       Low polarity
   */
  void setLowThresholdInterrupt(uint8_t channelX, uint8_t channelY, uint8_t channelZ, int8_t lowThreshold, uint8_t polarity);

  /**
   * @fn getLowThresholdInterrputState
   * @brief Get the status of low threshold interrupt, which axis triggered the low threshold interrupt
   * @return status
   * @n bit0 is 1 Indicate the interrupt occur at x-axis
   * @n bit1 is 1 Indicate the interrupt occur at y-axis
   * @n bit2 is 1 Indicate the interrupt occur at z-axis
   * @n ------------------------------------
   * @n | bit7 ~ bit3 | bit2 | bit1 | bit0 |
   * @n ------------------------------------
   * @n |  reserved   |  0   |  0   |  0   |
   * @n ------------------------------------
   */
  uint8_t getLowThresholdInterrputState(void);

  /**
   * @fn setHighThresholdInterrupt
   * @brief Set high threshold interrupt, an interrupt is triggered when the geomagnetic value of a channel is beyond the threshold, the threshold is default to expand 16 times
   * @n  When an interrupt occurs, INT pin level will jump
   * @n  High pin polarity: active on high level, the default is low level, which will jump when the threshold is triggered.
   * @n  Low pin polarity: active on low level, the default is high level, which will jump when the threshold is triggered.
   * @param channelX
   * @n     INTERRUPT_X_ENABLE        Enable high threshold interrupt at x-axis
   * @n     INTERRUPT_X_DISABLE       Disable high threshold interrupt at x-axis
   * @param channelY
   * @n     INTERRUPT_Y_ENABLE        Enable high threshold interrupt at y-axis
   * @n     INTERRUPT_Y_DISABLE       Disable high threshold interrupt at y-axis
   * @param channelZ
   * @n     INTERRUPT_Z_ENABLE        Enable high threshold interrupt at z-axis
   * @n     INTERRUPT_Z_DISABLE       Disable high threshold interrupt at z-axis
   * @param  highThreshold
   * @n     High threshold, default to expand 16 times, for example: if the threshold is set to be 1, actually the geomagnetic data beyond 16 will trigger an interrupt
   * @param polarity
   * @n     POLARITY_HIGH      High polarity
   * @n     POLARITY_LOW       Low polarity
   */
  void setHighThresholdInterrupt(uint8_t channelX, uint8_t channelY, uint8_t channelZ, int8_t highThreshold, uint8_t polarity);

  /**
   * @fn getHighThresholdInterrputState
   * @brief Get the status of high threshold interrupt, which axis triggered the high threshold interrupt
   * @return status
   * @n bit0 is 1 Indicate the interrupt occur at x-axis
   * @n bit1 is 1 Indicate the interrupt occur at y-axis
   * @n bit2 is 1 Indicate the interrupt occur at z-axis
   * @n ------------------------------------
   * @n | bit7 ~ bit3 | bit2 | bit1 | bit0 |
   * @n ------------------------------------
   * @n |  reserved   |  0   |  0   |  0   |
   * @n ------------------------------------
   */
  uint8_t getHighThresholdInterrputState(void);

  /**
   * @fn setInterrputPin
   * @brief Enable or disable INT interrupt pin, enabling pin will trigger interrupt pin INT level jump
   * @n  After disabling pin, INT interrupt pin will not have level jump
   * @n  High polarity: active on high level, the default is low level, which turns to high level when the interrupt is triggered.
   * @n  Low polarity: active on low level, the default is high level, which turns to low level when the interrupt is triggered.
   * @param modes
   * @n  ENABLE_INTERRUPT_PIN        Enable interrupt pin
   * @n  DISABLE_INTERRUPT_PIN       Disable interrupt pin
   * @param polarity
   * @n  POLARITY_HIGH               High polarity
   * @n  POLARITY_LOW                Low polarity
   */
  void setInterrputPin(uint8_t modes, uint8_t polarity = POLARITY_HIGH);

  /**
   * @brief Set interrupt latch mode, after enabling interrupt latch, the data can be refreshed only when the BMM150_REG_INTERRUPT_STATUS interrupt status register is read.
   * @n Disable interrupt latch, data update in real-time
   * @param modes
   * @n INTERRUPUT_LATCH_ENABLE      Enable interrupt latch
   * @n INTERRUPUT_LATCH_DISABLE     Disable interrupt latch
   */
  void setInterruputLatch(uint8_t modes);

  /**
   * @fn getChipID
   * @brief get bmm150 chip id
   * @return chip id
   */
  uint8_t getChipID(void);

  /**
   * @fn getTrimData
   * @brief Get bmm150 reserved data information, which is used for data compensation
   */
  void getTrimData(void);

  /**
   * @fn setReg
   * @brief Write the data with specified length to the specified register
   * @param regAddr register address
   * @param regData register data
   * @param len register data length
   */
  void setReg(uint8_t regAddr, uint8_t *regData, uint8_t len);

  /**
   * @fn getReg
   * @brief Get the data with specified length from the register
   * @param regAddr register address
   * @param regData register data
   * @param len register data length
   */
  void getReg(uint8_t regAddr, uint8_t *regData, uint8_t len);

  /**
   * @fn setXYRep
   * @brief the number of repetitions for x/y-axis
   * @param repXY    xy axis repetition
   */
  void setXYRep(uint8_t repXY);

  /**
   * @fn setZRep
   * @brief the number of repetitions for z-axis
   * @param repZ    z axis repetition
   */
  void setZRep(uint8_t repZ);

  /**
   * @fn setPowerControlBit
   * @brief Enable or disable power control bit, for switching between suspended and sleep mode
   * @param powerBit:
   * @n   BMM150_POWER_CNTRL_ENABLE        Enable power
   * @n   BMM150_POWER_CNTRL_DISABLE       Disable power
   */
  void setPowerControlBit(uint8_t powerBit);

  /**
   * @fn compensateX
   * @brief Compensate the geomagnetic data at x-axis
   * @param magDataX     The raw geomagnetic data
   * @param dataRhall    The compensated data
   * @return data        The calibrated geomagnetic data
   */
  int16_t compensateX(int16_t magDataX ,uint16_t dataRhall);

  /**
   * @fn compensateY
   * @brief Compensate the geomagnetic data at y-axis
   * @param magDataY     The raw geomagnetic data
   * @param dataRhall    The compensated data
   * @return data        The calibrated geomagnetic data
   */
  int16_t compensateY(int16_t magDataY, uint16_t dataRhall);

  /**
   * @fn compensateZ
   * @brief Compensate the geomagnetic data at z-axis
   * @param magDataZ     The raw geomagnetic data
   * @param dataRhall    The compensated data
   * @return data        The calibrated geomagnetic data
   */
  int16_t compensateZ(int16_t magDataZ ,uint16_t dataRhall);



float fcompensateX(int16_t magDataX, uint16_t dataRhall);
float fcompensateY(int16_t magDataY, uint16_t dataRhall);
float fcompensateZ(int16_t magDataZ, uint16_t dataRhall);
  /**
   * @fn normalSelfTest
   * @brief Normal self test mode, get the test results
   * @return results:
   * @n      0       SELF_TEST_XYZ_SUCCESS                   xyz Self test success
   * @n      1       BMM150_W_NORMAL_SELF_TEST_YZ_FAIL       x  success,yz fail
   * @n      2       BMM150_W_NORMAL_SELF_TEST_XZ_FAIL       y  success,xz fail
   * @n      3       BMM150_W_NORMAL_SELF_TEST_Z_FAIL        xy success,z  fail
   * @n      4       BMM150_W_NORMAL_SELF_TEST_XY_FAIL       z  success,xy fail
   * @n      5       BMM150_W_NORMAL_SELF_TEST_Y_FAIL        xz success,y  fail
   * @n      6       BMM150_W_NORMAL_SELF_TEST_X_FAIL        yz success,x  fail
   * @n      7       SELF_TEST_XYZ_FAIL                      xyz fail
   */
  int8_t normalSelfTest(void);

  /**
   * @fn validatNormalSelfTest
   * @brief Get the results of normal self test
   * @n  This internal API is used to validate the results of normal self test
   * @n  by using the self test status available in the bit0 of registers 0x42,0x44,0x46.
   * @return results
   */
  int8_t  validatNormalSelfTest(void);

  /**
   * @fn advSelfTest
   * @brief Advanced self test mode, compare the measured data
   * @return results:
   * @n  0       SELF_TEST_XYZ_SUCCESS                   xyz Self test success
   * @n  1       BMM150_W_NORMAL_SELF_TEST_YZ_FAIL       x  success,yz fail
   * @n  2       BMM150_W_NORMAL_SELF_TEST_XZ_FAIL       y  success,xz fail
   * @n  3       BMM150_W_NORMAL_SELF_TEST_Z_FAIL        xy success,z  fail
   * @n  4       BMM150_W_NORMAL_SELF_TEST_XY_FAIL       z  success,xy fail
   * @n  5       BMM150_W_NORMAL_SELF_TEST_Y_FAIL        xz success,y  fail
   * @n  6       BMM150_W_NORMAL_SELF_TEST_X_FAIL        yz success,x  fail
   * @n  7       SELF_TEST_XYZ_FAIL                      xyz fail
   */
  int8_t  advSelfTest(void);
  
  /**
   * @fn advSelfTestMeasurement
   * @brief Self test of data at z-axis
   * @n This internal API is used to set the positive or negative value of
   * @n self-test current and obtain the corresponding magnetometer z axis data
   * @param selfTestCurrent    Self test current mode
   * @n BMM150_DISABLE_SELF_TEST_CURRENT         Disable test current mode
   * @n BMM150_ENABLE_NEGATIVE_CURRENT           Negative current mode
   * @n BMM150_ENABLE_POSITIVE_CURRENT           Positive current mode
   */
  int16_t advSelfTestMeasurement(uint8_t selfTestCurrent);
  
  /**
   * @fn validateAdvSelfTest
   * @brief Get the results of advanced self test, compare the test data of positive and negative current
   * @param postiveDataZ      The test data of positive current
   * @param negativeDataZ     The test data of negative current
   * @return results
   * @n     SELF_TEST_XYZ_SUCCESS                xyz Self test success
   * @n     SELF_TEST_XYZ_FAIL                   Test fail
   */
  int8_t validateAdvSelfTest(int16_t postiveDataZ ,int16_t negativeDataZ);

  /**
   * @fn setAdvSelfTestCurrent
   * @brief Set advanced self test current mode, for self test
   * @n  This internal API is used to set the self test current value in
   * @n  the Adv. ST bits (bit6 and bit7) of 0x4C register
   * @param selfTestCurrent
   * @n  BMM150_DISABLE_SELF_TEST_CURRENT         Disable test current mode
   * @n  BMM150_ENABLE_NEGATIVE_CURRENT           Negative current mode
   * @n  BMM150_ENABLE_POSITIVE_CURRENT           Positive current mode
   */
  void setAdvSelfTestCurrent(uint8_t selfTestCurrent);

  /**
   * @fn setDataOverrun
   * @brief Set data overrun interrupt state
   * @param modes
   * @n DATA_OVERRUN_ENABLE    enable overrun
   * @n DATA_OVERRUN_DISABLE   disable overrun (default disable overrun)
   */
  void setDataOverrun(uint8_t modes);

  /**
   * @fn getDataOverrunState
   * @brief get data overrun interrupt state
   * @return status  data overrun status
   * @n true  is data overrun
   * @n false is not data overrun
   */
  bool getDataOverrunState(void);

  /**
   * @fn setOverflowPin
   * @brief Set data overflow interrupt pin
   * @param modes  enable or disable overflow pin
   * @n OVERFLOW_INT_ENABLE    enable overflow
   * @n OVERFLOW_INT_DISABLE   disable overflow (default disable overflow)
   */
  void setOverflowPin(uint8_t modes);

  /**
   * @fn getOverflowState
   * @brief get data overflow interrupt state
   * @return status  data overflow status
   * @n  true  is  overflow
   * @n  false is  not overflow
   */
  bool getOverflowState(void);

  virtual void writeData(uint8_t Reg, uint8_t *Data, uint8_t len) = 0;
  virtual int16_t readData(uint8_t Reg, uint8_t *Data, uint8_t len) = 0;
private:
  uint8_t __thresholdMode = 3;
};

class BMM150_I2C:public BMM150{
public:
  BMM150_I2C(MF_I2C *pWire, uint8_t addr = 0x75);
  uint8_t begin(void);
protected:
  virtual void     writeData(uint8_t Reg, uint8_t *Data, uint8_t len);
  virtual int16_t  readData(uint8_t Reg, uint8_t *Data, uint8_t len);
private:
  MF_I2C *_i2c;
  uint8_t _I2C_addr;
};

class BMM150_SPI:public BMM150{
public:
  uint8_t begin(void);
  BMM150_SPI(uint8_t csPin=10,SPIClass *spi=&SPI);
protected:
  virtual void     writeData(uint8_t Reg, uint8_t *Data, uint8_t len);
  virtual int16_t  readData(uint8_t Reg, uint8_t *Data, uint8_t len);

private:
  SPIClass *_pSpi;
  uint8_t _csPin;
};
#endif
