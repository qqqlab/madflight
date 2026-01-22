/*!
 * @file BMM150.cpp
 * @brief Define the infrastructure of the BMM150 class and the implementation of the underlying methods
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      [ZhixinLiu](zhixin.liu@dfrobot.com)
 * @version     V1.0.0
 * @date        2020-04-21
 * @url         https://github.com/DFRobot/BMM150
 */
#include "BMM150.h"

BMM150::BMM150()
{
}
BMM150::~BMM150()
{
}

bool BMM150::sensorInit()
{
  uint8_t chipID;
  /* Power up the sensor from suspend to sleep mode */
  setPowerControlBit(BMM150_POWER_CNTRL_DISABLE);
  setPowerControlBit(BMM150_POWER_CNTRL_ENABLE);
  getReg(BMM150_REG_CHIP_ID, &chipID, 1);

  if(chipID == BMM150_CHIP_ID){
    getTrimData();
    return true;
  }else{
    return false;
  }
}

uint8_t BMM150::getChipID(void)
{
  uint8_t chipID = 0;
  getReg(BMM150_REG_CHIP_ID, &chipID, 1);
  return chipID;
}

void BMM150::softReset(void)
{
  uint8_t regData = 0;
  getReg(BMM150_REG_POWER_CONTROL, &regData, 1);
  regData = regData | BMM150_SET_SOFT_RESET;
  setReg(BMM150_REG_POWER_CONTROL, &regData, 1);
  delay(1000);
}

void BMM150::setOperationMode(uint8_t opMode)
{
  uint8_t regData = 0;
  switch (opMode)
  {
    case BMM150_POWERMODE_NORMAL:
    case BMM150_POWERMODE_FORCED:
    case BMM150_POWERMODE_SLEEP:
      setPowerControlBit(BMM150_POWER_CNTRL_ENABLE);
      getReg(BMM150_REG_OP_MODE, &regData, 1);
      regData = BMM150_SET_BITS(regData, BMM150_OP_MODE, opMode);
      setReg(BMM150_REG_OP_MODE, &regData, 1);
      break;
    case BMM150_POWERMODE_SUSPEND:
      setPowerControlBit(BMM150_POWER_CNTRL_DISABLE);
      break;
    default:
      break;
  }
}

String BMM150::getOperationMode(void)
{
  uint8_t regData = 0;
  uint8_t powerData = 0;
  uint8_t number = 0;
  String result;
  getReg(BMM150_REG_POWER_CONTROL, &powerData, 1);
  if(powerData == 0){
    number = BMM150_POWERMODE_SUSPEND;
  }else{
    getReg(BMM150_REG_OP_MODE, &regData, 1);
    number = (regData&0x06)>>1;
  }
  switch(number)
  {
    case BMM150_POWERMODE_NORMAL:
      result = "bmm150 is normal mode!";
      break;
    case BMM150_POWERMODE_FORCED:
      result = "bmm150 is forced mode!";
      break;
    case BMM150_POWERMODE_SLEEP:
      result = "bmm150 is sleep mode!";
      break;
    case BMM150_POWERMODE_SUSPEND:
      result = "bmm150 is suspend mode!";
      break;
    default:
      result = "error mode!";
      break;
  }
  return result;
}

void BMM150::setRate(uint8_t rate)
{
  uint8_t regData;
  getReg(BMM150_REG_OP_MODE, &regData, 1);
  switch(rate){
    case BMM150_DATA_RATE_10HZ:
    case BMM150_DATA_RATE_02HZ:
    case BMM150_DATA_RATE_06HZ:
    case BMM150_DATA_RATE_08HZ:
    case BMM150_DATA_RATE_15HZ:
    case BMM150_DATA_RATE_20HZ:
    case BMM150_DATA_RATE_25HZ:
    case BMM150_DATA_RATE_30HZ:
      regData = BMM150_SET_BITS(regData, BMM150_ODR, rate);
      setReg(BMM150_REG_OP_MODE, &regData, 1);
    break;
    default:
      break;
  }
}

uint8_t BMM150::getRate(void)
{
  uint8_t regData;
  uint8_t result;
  getReg(BMM150_REG_OP_MODE, &regData, 1);
  regData = (regData&0x38)>>3;
  switch(regData){
    case BMM150_DATA_RATE_02HZ:
      result = 2;
      break;
    case BMM150_DATA_RATE_06HZ:
      result = 6;
      break;
    case BMM150_DATA_RATE_08HZ:
      result = 8;
      break;
    case BMM150_DATA_RATE_10HZ:
      result = 10;
      break;
    case BMM150_DATA_RATE_15HZ:
      result = 15;
      break;
    case BMM150_DATA_RATE_20HZ:
      result = 20;
      break;
    case BMM150_DATA_RATE_25HZ:
      result = 25;
      break;
    case BMM150_DATA_RATE_30HZ:
      result = 30;
      break;
  }
  return result;
}

void BMM150::setPresetMode(uint8_t presetMode)
{
  switch (presetMode){
    case BMM150_PRESETMODE_LOWPOWER:
      setXYRep(BMM150_REPXY_LOWPOWER);
      setZRep(BMM150_REPZ_LOWPOWER);
      break;
    case BMM150_PRESETMODE_REGULAR:
      setXYRep(BMM150_REPXY_REGULAR);
      setZRep(BMM150_REPXY_REGULAR);
      break;
    case BMM150_PRESETMODE_HIGHACCURACY:
      setXYRep(BMM150_REPXY_HIGHACCURACY);
      setZRep(BMM150_REPZ_HIGHACCURACY);
      break;
    case BMM150_PRESETMODE_ENHANCED:
      setXYRep(BMM150_REPXY_ENHANCED);
      setZRep(BMM150_REPZ_ENHANCED);
      break;
    default:
      break;
  }
}

sBmm150MagData_t BMM150::getGeomagneticData(void)
{
  int16_t msbData;
  sBmm150MagData_t magData;
  uint8_t regData[BMM150_LEN_XYZR_DATA] = {0};
  sBmm150RawMagData_t rawMagData;

  getReg(BMM150_REG_DATA_X_LSB, regData, BMM150_LEN_XYZR_DATA);

  regData[0] = BMM150_GET_BITS(regData[0], BMM150_DATA_X);
  msbData = ((int16_t)((int8_t)regData[1])) * 32;
  rawMagData.rawDataX = (int16_t)(msbData | regData[0]);

  regData[2] = BMM150_GET_BITS(regData[2], BMM150_DATA_Y);
  msbData = ((int16_t)((int8_t)regData[3])) * 32;
  rawMagData.rawDataY = (int16_t)(msbData | regData[2]);

  regData[4] = BMM150_GET_BITS(regData[4], BMM150_DATA_Z);
  msbData = ((int16_t)((int8_t)regData[5])) * 128;
  rawMagData.rawDataZ = (int16_t)(msbData | regData[4]);

  regData[6] = BMM150_GET_BITS(regData[6], BMM150_DATA_RHALL);
  rawMagData.rawDataR = (uint16_t)(((uint16_t)regData[7] << 6) | regData[6]);

  magData.x = compensateX(rawMagData.rawDataX, rawMagData.rawDataR);
  magData.y = compensateY(rawMagData.rawDataY, rawMagData.rawDataR);
  magData.z = compensateZ(rawMagData.rawDataZ, rawMagData.rawDataR);

  magData.xx = fcompensateX(rawMagData.rawDataX, rawMagData.rawDataR);
  magData.yy = fcompensateY(rawMagData.rawDataY, rawMagData.rawDataR);
  magData.zz = fcompensateZ(rawMagData.rawDataZ, rawMagData.rawDataR);
  return magData;
}

float BMM150::getCompassDegree(void)
{
  float compass = 0.0;
  sBmm150MagData_t magData = getGeomagneticData();
  compass = atan2(magData.x, magData.y);
  if (compass < 0) {
    compass += 2 * PI;
  }
  if (compass > 2 * PI) {
     compass -= 2 * PI;
  }
  return compass * 180 / M_PI;
}

void BMM150::setDataReadyPin(uint8_t modes, uint8_t polarity)
{
  uint8_t regData = 0;
  getReg(BMM150_REG_AXES_ENABLE, &regData, 1);
  if(modes == DRDY_DISABLE){
    regData = (regData&0x7F);
  }else{
    regData = (regData|0x80);
  }
  if(polarity == POLARITY_LOW){
    regData = (regData&0xFB);
  }else{
    regData = (regData|0x04);
  }
  setReg(BMM150_REG_AXES_ENABLE, &regData, 1);
}

bool BMM150::getDataReadyState(void)
{
  uint8_t regData = 0;
  getReg(BMM150_REG_DATA_READY_STATUS, &regData, 1);
  if(regData & 0x01){
    return true;
  }else{
    return false;
  }
}

void BMM150::setMeasurementXYZ(uint8_t channelX, uint8_t channelY ,uint8_t channelZ)
{
  uint8_t regData = 0;
  getReg(BMM150_REG_AXES_ENABLE, &regData, 1);
  if(channelX == MEASUREMENT_X_DISABLE){
    regData |= 0x08;
  }else{
    regData &= 0xF7;
  }
  if(channelY == MEASUREMENT_Y_DISABLE){
    regData |= 0x10;
  }else{
    regData &= 0xEF;
  }
  if(channelZ == MEASUREMENT_Z_DISABLE){
    regData |= 0x20; 
  }else{
    regData &= 0xDF;
  }
  setReg(BMM150_REG_AXES_ENABLE, &regData, 1);
}

String BMM150::getMeasurementStateXYZ(void)
{
  uint8_t regData = 0;
  String result;
  getReg(BMM150_REG_AXES_ENABLE, &regData, 1);
  regData = (regData&0x38)>>3;
  switch(regData)
  {
    case 0:
      result = "The xyz axis is enable!";
      break;
    case 1:
      result = "The yz axis is enable!";
      break;
    case 2:
      result = "The xz axis is enable!";
      break;
    case 3:
      result = "The z axis is enable!";
      break;
    case 4:
      result = "The xy axis is enable!";
      break;
    case 5:
      result = "The y axis is enable!";
      break;
    case 6:
      result = "The x axis is enable!";
      break;
    case 7:
      result = "The xyz axis is not enabled !";
      break;
  }
  return result;
}

void BMM150::setLowThresholdInterrupt(uint8_t channelX, uint8_t channelY, uint8_t channelZ, int8_t lowThreshold, uint8_t polarity)
{
  uint8_t regData = 0;
  uint8_t temp     = 0;
  setInterrputPin(ENABLE_INTERRUPT_PIN, polarity);
  delay(100);
  if(lowThreshold < 0){
    temp = (uint8_t)(lowThreshold*-1) | 0x80;
  }else{
    temp = (uint8_t)lowThreshold;
  }
  setReg(BMM150_REG_LOW_THRESHOLD, &temp, 1);
  getReg(BMM150_REG_INT_CONFIG, &regData, 1);
  if(channelX == INTERRUPT_X_DISABLE){
    regData |= 0x01;
  }else{
    regData &= 0xFE;
  }
  if(channelY == INTERRUPT_Y_DISABLE){
    regData |= 0x02;
  }else{
    regData &= 0xFC;
  }
  if(channelZ == INTERRUPT_Z_DISABLE){
    regData |= 0x04; 
  }else{
    regData &= 0xFB;
  }
  setReg(BMM150_REG_INT_CONFIG, &regData, 1);
  
}

uint8_t BMM150::getLowThresholdInterrputState(void)
{
  uint8_t regData = 0;
  uint8_t state = getDataReadyState();
  if(state == 1){
    getReg(BMM150_REG_INTERRUPT_STATUS, &regData, 1);
    return regData&0x07;
  }else{
    return 0;
  }
}

void BMM150::setThresholdInterrupt(uint8_t modes, uint8_t channelX, uint8_t channelY, uint8_t channelZ, int8_t threshold, uint8_t polarity)
{
  if(modes == LOW_THRESHOLD_INTERRUPT){
    __thresholdMode = LOW_THRESHOLD_INTERRUPT;
    setLowThresholdInterrupt(channelX, channelY, channelZ, threshold, polarity);
  }else{
    setHighThresholdInterrupt(channelX, channelY, channelZ, threshold, polarity);
    __thresholdMode = HIGH_THRESHOLD_INTERRUPT;
  }
}

void BMM150::setThresholdInterrupt(uint8_t modes, int8_t threshold, uint8_t polarity)
{
  if(modes == LOW_THRESHOLD_INTERRUPT){
    __thresholdMode = LOW_THRESHOLD_INTERRUPT;
    setLowThresholdInterrupt(INTERRUPT_X_ENABLE, INTERRUPT_Y_ENABLE, INTERRUPT_Z_ENABLE, threshold, polarity);
  }else{
    setHighThresholdInterrupt(INTERRUPT_X_ENABLE, INTERRUPT_Y_ENABLE, INTERRUPT_Z_ENABLE, threshold, polarity);
    __thresholdMode = HIGH_THRESHOLD_INTERRUPT;
  }
}

sBmm150ThresholdData_t BMM150::getThresholdData(void)
{
  uint8_t value = 0;
  sBmm150MagData_t magData;
  sBmm150ThresholdData_t threshold;
  if(__thresholdMode == LOW_THRESHOLD_INTERRUPT){
    value = getLowThresholdInterrputState();
  }else if(__thresholdMode == HIGH_THRESHOLD_INTERRUPT){
    value = getHighThresholdInterrputState();
  }else{
    threshold.value = 0;
  }
  magData = getGeomagneticData();
  if((value>>0)&0x01){ threshold.x = magData.x;}
  else{ threshold.x = NO_DATA;}
  if((value>>1)&0x01){ threshold.y = magData.y;}
  else{ threshold.y = NO_DATA;}
  if((value>>2)&0x01){ threshold.z = magData.z;}
  else{ threshold.z = NO_DATA;}
  threshold.value = value;
  threshold.state = binChangeString(value);
  return threshold;
}

String BMM150::binChangeString(uint8_t value)
{
  String state ="";
  for(uint8_t i = 8; i > 0; i--){
    if((value>>(i-1))&0x01){
      state += "1";
    }else{
      state += "0";
    }
  }
  return state;
}

void BMM150::setHighThresholdInterrupt(uint8_t channelX, uint8_t channelY, uint8_t channelZ, int8_t highThreshold, uint8_t polarity)
{
  uint8_t regData = 0;
  uint8_t temp    = 0;
  setInterrputPin(ENABLE_INTERRUPT_PIN, polarity);
  delay(100);
  if(highThreshold<0){
    temp = (uint8_t)(highThreshold*-1) | 0x80;
  }else{
    temp = (uint8_t)highThreshold;
  }
  setReg(BMM150_REG_HIGH_THRESHOLD, &temp, 1);
  getReg(BMM150_REG_HIGH_THRESHOLD, &regData, 1);
  getReg(BMM150_REG_INT_CONFIG, &regData, 1);
  if(channelX == INTERRUPT_X_DISABLE){
    regData |= 0x08;
  }else{
    regData &= 0xF7;
  }
  if(channelY == INTERRUPT_Y_DISABLE){
    regData |= 0x10;
  }else{
    regData &= 0xEF;
  }
  if(channelZ == INTERRUPT_Z_DISABLE){
    regData |= 0x20; 
  }else{
    regData &= 0xDF;
  }
  setReg(BMM150_REG_INT_CONFIG, &regData, 1);
  getReg(BMM150_REG_INT_CONFIG, &regData, 1);
  
}

uint8_t BMM150::getHighThresholdInterrputState(void)
{
  uint8_t regData = 0;
  uint8_t state = getDataReadyState();
  if(state == 1){
    getReg(BMM150_REG_INTERRUPT_STATUS, &regData, 1);
    return (regData&0x38)>>3;
  }else{
    return 0;
  }
}

void BMM150::setInterrputPin(uint8_t modes, uint8_t polarity)
{
  uint8_t regData = 0;
  getReg(BMM150_REG_AXES_ENABLE, &regData, 1);
  if(modes == DISABLE_INTERRUPT_PIN){
    regData = (regData&0xBF);
  }else{
    regData = (regData|0x40);
  }

  if(polarity == POLARITY_LOW){
    regData = (regData&0xFE);
  }else{
    regData = (regData|0x01);
  }
  setReg(BMM150_REG_AXES_ENABLE, &regData, 1);
}

void BMM150::setInterruputLatch(uint8_t modes)
{
  uint8_t regData = 0;
  getReg(BMM150_REG_AXES_ENABLE, &regData, 1);
  if(modes == INTERRUPUT_LATCH_DISABLE){
    regData &= 0xFD;
  }else{
    regData |= 0x02;
  }
  setReg(BMM150_REG_AXES_ENABLE, &regData, 1);
}

String BMM150::selfTest(uint8_t testMode)
{
  int8_t rslt;
  String result;
  switch (testMode)
  {
    case BMM150_SELF_TEST_NORMAL:
      setOperationMode(BMM150_POWERMODE_SLEEP);
      rslt = normalSelfTest();  
      break;
    case BMM150_SELF_TEST_ADVANCED:
      rslt = advSelfTest();
      softReset();
      break;
    default:
      rslt = -1;
      break;
  }
  if(rslt == SELF_TEST_XYZ_SUCCESS){
    result = "xyz aixs self test success!";
  }else if(rslt == SELF_TEST_XYZ_FAIL){
    result = "xyz aixs self test failed!";
  }else{
    if((rslt>>0)&0x01) result += "x";
    if((rslt>>1)&0x01) result += "y";
    if((rslt>>2)&0x01) result += "z";
    result += "test success !";
  }
  return result;
}

void BMM150::setReg(uint8_t regAddr, uint8_t *regData, uint8_t len)
{
  delay(3);
  writeData(regAddr, regData, len);
}

void BMM150::getReg(uint8_t regAddr, uint8_t *regData, uint8_t len)
{
  delay(3);
  readData(regAddr, regData, len);
}

void BMM150::setXYRep(uint8_t repXY)
{
  uint8_t data = repXY;
  setReg(BMM150_REG_REP_XY, &data, 1);
}

void BMM150::setZRep(uint8_t repZ)
{
  uint8_t data = repZ;
  setReg(BMM150_REG_REP_Z, &data, 1);
}

void BMM150::setPowerControlBit(uint8_t powerBit)
{
  uint8_t regData = 0;
  getReg(BMM150_REG_POWER_CONTROL, &regData, 1);
  regData = BMM150_SET_BITS_POS_0(regData, BMM150_PWR_CNTRL, powerBit);
  setReg(BMM150_REG_POWER_CONTROL, &regData, 1);
}

void BMM150::getTrimData(void)
{
  uint8_t trimX1Y1[2] = { 0 };
  uint8_t trimXYXData[4] = { 0 };
  uint8_t trimXY1XY2[10] = { 0 };
  uint16_t tempMsb = 0;

  /* Trim register value is read */
  getReg(BMM150_DIG_X1, trimX1Y1, 2);
  getReg(BMM150_DIG_Z4_LSB, trimXYXData, 4);
  getReg(BMM150_DIG_Z2_LSB, trimXY1XY2, 10);

  // Trim data which is read is updated in the device structure
  _trimData.digX1 = (int8_t)trimX1Y1[0];
  _trimData.digY1 = (int8_t)trimX1Y1[1];
  _trimData.digX2 = (int8_t)trimXYXData[2];
  _trimData.digY2 = (int8_t)trimXYXData[3];
  tempMsb = ((uint16_t)trimXY1XY2[3]) << 8;
  _trimData.digZ1 = (uint16_t)(tempMsb | trimXY1XY2[2]);
  tempMsb = ((uint16_t)trimXY1XY2[1]) << 8;
  _trimData.digZ2 = (int16_t)(tempMsb | trimXY1XY2[0]);
  tempMsb = ((uint16_t)trimXY1XY2[7]) << 8;
  _trimData.digZ3 = (int16_t)(tempMsb | trimXY1XY2[6]);
  tempMsb = ((uint16_t)trimXYXData[1]) << 8;
  _trimData.digZ4 = (int16_t)(tempMsb | trimXYXData[0]);
  _trimData.digXY1 = trimXY1XY2[9];
  _trimData.digXY2 = (int8_t)trimXY1XY2[8];
  tempMsb = ((uint16_t)(trimXY1XY2[5] & 0x7F)) << 8;
  _trimData.digXYZ1 = (uint16_t)(tempMsb | trimXY1XY2[4]);
}

int16_t BMM150::compensateX(int16_t magDataX, uint16_t dataRhall)
{
  int16_t retval, processCompX1;
  uint16_t processCompX0, processCompX2;
  int32_t processCompX3, processCompX4, processCompX5, processCompX6, processCompX7, processCompX8, processCompX9, processCompX10;
  /* Overflow condition check */
  if (magDataX != BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP){
    if (dataRhall != 0){
      /* Availability of valid data */
      processCompX0 = dataRhall;
    }else if (_trimData.digXYZ1 != 0){
      processCompX0 = _trimData.digXYZ1;
    }else{
      processCompX0 = 0;
    }
    if (processCompX0 != 0){
      processCompX1 = ((int32_t)_trimData.digXYZ1) * 16384 / processCompX0;
      processCompX2 = ((uint16_t)(processCompX1)) - ((uint16_t)0x4000);
      retval = ((int16_t)processCompX2);
      processCompX3 = (((int32_t)retval) * ((int32_t)retval));
      processCompX4 = (((int32_t)_trimData.digXY2) * (processCompX3 / 128));
      processCompX5 = (int32_t)(((int16_t)_trimData.digXY1) * 128);
      processCompX6 = ((int32_t)retval) * processCompX5;
      processCompX7 = (((processCompX4 + processCompX6) / 512) + ((int32_t)0x100000));
      processCompX8 = ((int32_t)(((int16_t)_trimData.digX2) + ((int16_t)0xA0)));
      processCompX9 = ((processCompX7 * processCompX8) / 4096);
      processCompX10 = ((int32_t)magDataX) * processCompX9;
      retval = ((int16_t)(processCompX10 / 8192));
      retval = (retval + (((int16_t)_trimData.digX1) * 8)) / 16;
    }else{
      retval = BMM150_OVERFLOW_OUTPUT;
    }
  }else{
    /* Overflow condition */
    retval = BMM150_OVERFLOW_OUTPUT;
  }
  return retval;
}

int16_t BMM150::compensateY(int16_t magDataY, uint16_t dataRhall)
{
  int16_t retval;
  uint16_t processCompY0, processCompY2;
  int32_t processCompY1, processCompY3, processCompY4, processCompY5, processCompY6, processCompY7, processCompY8, processCompY9;
  /* Overflow condition check */
  if (magDataY != BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP){
    if (dataRhall != 0){
      /* Availability of valid data */
      processCompY0 = dataRhall;
    }else if (_trimData.digXYZ1 != 0){
      processCompY0 = _trimData.digXYZ1;
    }else{
      processCompY0 = 0;
    }
    if (processCompY0 != 0){
      /* Processing compensation equations */
      processCompY1 = (((int32_t)_trimData.digXYZ1) * 16384) / processCompY0;
      processCompY2 = ((uint16_t)processCompY1) - ((uint16_t)0x4000);
      retval = ((int16_t)processCompY2);
      processCompY3 = ((int32_t) retval) * ((int32_t)retval);
      processCompY4 = ((int32_t)_trimData.digXY2) * (processCompY3 / 128);
      processCompY5 = ((int32_t)(((int16_t)_trimData.digXY1) * 128));
      processCompY6 = ((processCompY4 + (((int32_t)retval) * processCompY5)) / 512);
      processCompY7 = ((int32_t)(((int16_t)_trimData.digY2) + ((int16_t)0xA0)));
      processCompY8 = (((processCompY6 + ((int32_t)0x100000)) * processCompY7) / 4096);
      processCompY9 = (((int32_t)magDataY) * processCompY8);
      retval = (int16_t)(processCompY9 / 8192);
      retval = (retval + (((int16_t)_trimData.digY1) * 8)) / 16;
    }else{
      retval = BMM150_OVERFLOW_OUTPUT;
    }
  }else{
    /* Overflow condition */
    retval = BMM150_OVERFLOW_OUTPUT;
  }
    return retval;
}

int16_t BMM150::compensateZ(int16_t magDataZ, uint16_t dataRhall)
{
  int32_t retval, processCompZ1, processCompZ2, processCompZ3;
  int16_t processCompZ0, processCompZ4;
  if (magDataZ != BMM150_OVERFLOW_ADCVAL_ZAXIS_HALL){
    if ((_trimData.digZ2 != 0) && (_trimData.digZ1 != 0) && (dataRhall != 0) &&(_trimData.digXYZ1 != 0)){
      /*Processing compensation equations */
      processCompZ0 = ((int16_t)dataRhall) - ((int16_t) _trimData.digXYZ1);
      processCompZ1 = (((int32_t)_trimData.digZ3) * ((int32_t)(processCompZ0))) / 4;
      processCompZ2 = (((int32_t)(magDataZ - _trimData.digZ4)) * 32768);
      processCompZ3 = ((int32_t)_trimData.digZ1) * (((int16_t)dataRhall) * 2);
      processCompZ4 = (int16_t)((processCompZ3 + (32768)) / 65536);
      retval = ((processCompZ2 - processCompZ1) / (_trimData.digZ2 + processCompZ4));

      /* Saturate result to +/- 2 micro-tesla */
      if (retval > BMM150_POSITIVE_SATURATION_Z){
        retval = BMM150_POSITIVE_SATURATION_Z;
      }else if (retval < BMM150_NEGATIVE_SATURATION_Z){
         retval = BMM150_NEGATIVE_SATURATION_Z;
      }
      /* Conversion of LSB to micro-tesla */
      retval = retval / 16;
    }else{
      retval = BMM150_OVERFLOW_OUTPUT;
    }
  }else{
    /* Overflow condition */
    retval = BMM150_OVERFLOW_OUTPUT;
  }
  return (int16_t)retval;
}


float BMM150::fcompensateX(int16_t magDataX, uint16_t dataRhall)
{
  float retval = 0;
  float process_comp_x0;
  float process_comp_x1;
  float process_comp_x2;
  float process_comp_x3;
  float process_comp_x4;

  /* Overflow condition check */
  if ((magDataX != BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP) && (dataRhall != 0) && (_trimData.digXYZ1 != 0)) {
      /*Processing compensation equations*/
      process_comp_x0 = (((float)_trimData.digXYZ1) * 16384.0f / dataRhall);
      retval = (process_comp_x0 - 16384.0f);
      process_comp_x1 = ((float)_trimData.digXY2) * (retval * retval / 268435456.0f);
      process_comp_x2 = process_comp_x1 + retval * ((float)_trimData.digXY1) / 16384.0f;
      process_comp_x3 = ((float)_trimData.digX2) + 160.0f;
      process_comp_x4 = magDataX * ((process_comp_x2 + 256.0f) * process_comp_x3);
      retval = ((process_comp_x4 / 8192.0f) + (((float)_trimData.digX1) * 8.0f)) / 16.0f;
  } else {
    retval = (float)BMM150_OVERFLOW_OUTPUT;
  }

  return retval;
}

float BMM150::fcompensateY(int16_t magDataY, uint16_t dataRhall)
{
  float retval = 0;
  float process_comp_y0;
  float process_comp_y1;
  float process_comp_y2;
  float process_comp_y3;
  float process_comp_y4;

  /* Overflow condition check */
  if ((magDataY != BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP)  && (dataRhall != 0) && (_trimData.digXYZ1 != 0)) {
      /*Processing compensation equations*/
      process_comp_y0 = ((float)_trimData.digXYZ1) * 16384.0f / dataRhall;
      retval = process_comp_y0 - 16384.0f;
      process_comp_y1 = ((float)_trimData.digXY2) * (retval * retval / 268435456.0f);
      process_comp_y2 = process_comp_y1 + retval * ((float)_trimData.digXY1) / 16384.0f;
      process_comp_y3 = ((float)_trimData.digY2) + 160.0f;
      process_comp_y4 = magDataY * (((process_comp_y2) + 256.0f) * process_comp_y3);
      retval = ((process_comp_y4 / 8192.0f) + (((float)_trimData.digY1) * 8.0f)) / 16.0f;
  } else {
    retval = (float)BMM150_OVERFLOW_OUTPUT;
  }

  return retval;
}

float BMM150::fcompensateZ(int16_t magDataZ, uint16_t dataRhall)
{
    float retval = 0;
    float process_comp_z0;
    float process_comp_z1;
    float process_comp_z2;
    float process_comp_z3;
    float process_comp_z4;
    float process_comp_z5;
    if (magDataZ != BMM150_OVERFLOW_ADCVAL_ZAXIS_HALL && (_trimData.digZ2 != 0) && (_trimData.digZ1 != 0) && (dataRhall != 0) &&(_trimData.digXYZ1 != 0)){
      process_comp_z0 = ((float)magDataZ) - ((float)_trimData.digZ4);
      process_comp_z1 = ((float)dataRhall) - ((float)_trimData.digXYZ1);
      process_comp_z2 = (((float)_trimData.digZ3) * process_comp_z1);
      process_comp_z3 = ((float)_trimData.digZ1) * ((float)dataRhall) / 32768.0f;
      process_comp_z4 = ((float)_trimData.digZ2) + process_comp_z3;
      process_comp_z5 = (process_comp_z0 * 131072.0f) - process_comp_z2;
      retval = (process_comp_z5 / ((process_comp_z4) * 4.0f)) / 16.0f;
    }else{
      retval = (float)BMM150_OVERFLOW_OUTPUT;
    }
    return retval;
}

int8_t BMM150::normalSelfTest(void)
{
  uint8_t selfTestBit;
  uint8_t regData;
  uint8_t selfTestValue = 0x01;

  /* Read the data from register 0x4C */
  getReg(BMM150_REG_OP_MODE, &regData, 1);
  /* Set the Self Test bit(bit0) of the 0x4C register */
  regData = BMM150_SET_BITS_POS_0(regData, BMM150_SELF_TEST, selfTestValue);
  /* Write the data to 0x4C register to trigger self test */
  setReg(BMM150_REG_OP_MODE, &regData, 1);
  delay(300);
  /* Read the data from register 0x4C */
  getReg(BMM150_REG_OP_MODE, &regData, 1);
  // Self Test bit(bit0) is stored in self_test_enable, It will be reset to zero after the self test is over
  selfTestBit = BMM150_GET_BITS_POS_0(regData, BMM150_SELF_TEST);

  /* Check for self test completion status */
  if(selfTestBit == 0){
    /* Validates the self test results for all 3 axes */
    return validatNormalSelfTest();
  }else{
    return 7;
  }
}

int8_t BMM150::validatNormalSelfTest(void)
{
  int8_t rslt;
  uint8_t status;
  uint8_t self_test_rslt[5];

  /* Read the data from register 0x42 to 0x46 */
  getReg(BMM150_REG_DATA_X_LSB, self_test_rslt, BMM150_LEN_SELF_TEST);

  /* Parse and get the self test status bits */
  /* X-Self-Test (bit0) of 0x42 register is stored */
  self_test_rslt[0] = BMM150_GET_BITS_POS_0(self_test_rslt[0], BMM150_SELF_TEST);

  /* Y-Self-Test (bit0) of 0x44 register is stored */
  self_test_rslt[2] = BMM150_GET_BITS_POS_0(self_test_rslt[2], BMM150_SELF_TEST);
  
  /* Z-Self-Test (bit0) of 0x46 register is stored */
  self_test_rslt[4] = BMM150_GET_BITS_POS_0(self_test_rslt[4], BMM150_SELF_TEST);

  /* Combine the self test status and store it in the first
  * 3 bits of the status variable for processing
  */
  status = (uint8_t)((self_test_rslt[4] << 2) | (self_test_rslt[2] << 1) | self_test_rslt[0]);

  /* Validate status and store Self test result in "rslt" */
  if (status == BMM150_SELF_TEST_STATUS_SUCCESS)
  {
    /* Self test is success when all status bits are set */
    rslt = BMM150_OK;
  }else{
    if (status == BMM150_SELF_TEST_STATUS_XYZ_FAIL){
      /* Self test - all axis fail condition */
      rslt = SELF_TEST_XYZ_FAIL;
    }else{
      /* Self test - some axis fail condition */
      rslt = (int8_t)status;
    }
  }
  return rslt;
}

int8_t BMM150::advSelfTest(void)
{
  int8_t rslt;
  int16_t postiveDataZ;
  int16_t negativeDataZ;

  /* Set the power mode as sleep mode */
  setOperationMode(BMM150_POWERMODE_SLEEP);

  /* Disable XY-axis measurement */
  setMeasurementXYZ(MEASUREMENT_X_DISABLE, MEASUREMENT_Y_DISABLE, MEASUREMENT_Z_ENABLE);
  
  setZRep(BMM150_SELF_TEST_REP_Z);

  /* Measure the Z axes data with positive self-test current */
  postiveDataZ = advSelfTestMeasurement(BMM150_ENABLE_POSITIVE_CURRENT);
  delay(10);
  negativeDataZ = advSelfTestMeasurement(BMM150_ENABLE_NEGATIVE_CURRENT);

  /* Disable self-test current */
  setAdvSelfTestCurrent(BMM150_DISABLE_SELF_TEST_CURRENT);
  
  /* Validate the advanced self test */
  rslt = validateAdvSelfTest(postiveDataZ, negativeDataZ);
  return rslt;
}

int16_t BMM150::advSelfTestMeasurement(uint8_t selfTestCurrent)
{
  sBmm150MagData_t magData;
  setAdvSelfTestCurrent(selfTestCurrent);
  setOperationMode(BMM150_POWERMODE_FORCED);
  magData = getGeomagneticData();
  return magData.z;
}

int8_t BMM150::validateAdvSelfTest(int16_t postiveDataZ, int16_t negativeDataZ)
{
  int32_t adv_self_test_rslt;

  // Advanced self test difference between the Z axis mag data obtained by the positive and negative self-test current
  adv_self_test_rslt = postiveDataZ - negativeDataZ;

  // Advanced self test validation, Value of adv_self_test_rslt should be in between 180-240 micro-tesla */
  if((adv_self_test_rslt > 180) && (adv_self_test_rslt < 240)){
    /* Advanced self test success */
    return BMM150_OK;
  }else{
    /* Advanced self test fail */
    return 7;
  }
}

void BMM150::setAdvSelfTestCurrent(uint8_t selfTestCurrent)
{
  uint8_t regData;
  /* Read the 0x4C register */
  getReg(BMM150_REG_OP_MODE, &regData, 1);
  // Set the self test current value in the Adv. ST bits (bit6 and bit7) of 0x4c register
  regData = BMM150_SET_BITS(regData, BMM150_ADV_SELF_TEST, selfTestCurrent);
  setReg(BMM150_REG_OP_MODE, &regData, 1);
}

void BMM150::setDataOverrun(uint8_t modes)
{
  uint8_t regData = 0;

  getReg(BMM150_REG_INT_CONFIG, &regData, 1);
  if(modes == DATA_OVERRUN_DISABLE){
    regData &= 0x7F;
  }else{
    regData |= 0x80;
  }
  setReg(BMM150_REG_INT_CONFIG, &regData, 1);
}

bool BMM150::getDataOverrunState(void)
{
  uint8_t regData = 0;
  getReg(BMM150_REG_INT_CONFIG, &regData, 1);
  if(regData|0x80){
    return true;
  }else{
    return false;
  }
}

void BMM150::setOverflowPin(uint8_t modes)
{
  uint8_t regData = 0;
  getReg(BMM150_REG_INT_CONFIG, &regData, 1);
  if(modes == OVERFLOW_INT_DISABLE){
    regData &= 0xBF;
  }else{
    regData |= 0x40;
  }
  setReg(BMM150_REG_INT_CONFIG, &regData, 1);
}

bool BMM150::getOverflowState(void)
{
  uint8_t regData = 0;
  getReg(BMM150_REG_INT_CONFIG, &regData, 1);
  if(regData|0x40){
    return true;
  }else{
    return false;
  }
}

BMM150_I2C::BMM150_I2C(MF_I2C *pWire, uint8_t addr)
{
  _i2c = pWire;
  this->_I2C_addr = addr;
}

uint8_t BMM150_I2C::begin()
{
  _i2c->begin();
  _i2c->beginTransmission(_I2C_addr);
  if(_i2c->endTransmission() == 0){
    if(sensorInit()){
      return 0;
    }else{
      DBG("Chip id error ,please check sensor!");
      return 2;
    }
  }else{
    DBG("I2C device address error or no connection!");
    return 1;
  }
}

void BMM150_I2C::writeData(uint8_t Reg, uint8_t *Data, uint8_t len)
{
  _i2c->beginTransmission(this->_I2C_addr);
  _i2c->write(Reg);
  for(uint8_t i = 0; i < len; i++)
  {
    _i2c->write(Data[i]);
  }
  _i2c->endTransmission();
}

int16_t BMM150_I2C::readData(uint8_t Reg, uint8_t *Data ,uint8_t len)
{
  int i=0;
  _i2c->beginTransmission(this->_I2C_addr);
  _i2c->write(Reg);
  if(_i2c->endTransmission() != 0) return -1;

  int bytesReceived = _i2c->requestFrom((uint8_t)this->_I2C_addr, (uint8_t)len); //this also calls endTransmission(), don't call again
  if(bytesReceived > 0) {
    _i2c->readBytes(Data, bytesReceived);
  }
  return 0;
}

BMM150_SPI::BMM150_SPI(uint8_t csPin, SPIClass *pSpi)
{
  _pSpi = pSpi;
  _csPin = csPin;
}

uint8_t BMM150_SPI::begin(void)
{
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, 1);
  _pSpi->begin();
  if(sensorInit()){
    return 0;
  }else{
    DBG("Chip id error ,please check sensor!");
    return 2;
  }
}

void BMM150_SPI::writeData(uint8_t Reg, uint8_t *Data, uint8_t len)
{
  digitalWrite(_csPin, 0);
  _pSpi->transfer(Reg&0x7F);
  _pSpi->transfer(Data[0]);
  digitalWrite(_csPin, 1);
  for(uint8_t i = 1; i < len; i++){
    _pSpi->transfer(Data[i]);
  }
}

int16_t BMM150_SPI::readData(uint8_t Reg, uint8_t *Data, uint8_t len)
{
  digitalWrite(_csPin, 0);
  _pSpi->transfer(Reg|0x80);
  Data[0] = _pSpi->transfer(0xFF);
  for(uint8_t i = 1; i < len; i++){
    Data[i] = _pSpi->transfer(0xFF);
  }
  digitalWrite(_csPin, 1);
  return 0;
}
