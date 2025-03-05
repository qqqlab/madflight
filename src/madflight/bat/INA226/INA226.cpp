#include "INA226.h"

bool INA226::begin(MF_I2C *i2c, uint8_t address)
{
    _i2c = i2c;
    inaAddress = address;
    return true;
}

bool INA226::configure(ina226_averages_t avg, ina226_busConvTime_t busConvTime, ina226_shuntConvTime_t shuntConvTime, ina226_mode_t mode)
{
    uint16_t config = 0;

    config |= (avg << 9 | busConvTime << 6 | shuntConvTime << 3 | mode);

    vBusMax = 36;
    vShuntMax = 0.08192f;

    writeRegister16(INA226_REG_CONFIG, config);

    return true;
}

//NOTE: setting lower iMaxExpected than actual max scale does not improve resolution of the CURRENT register when averaging
//for example if CAL = 10*2048, then CURRENT register will always be a multiple of 10
bool INA226::calibrate(float rShuntValue, float iMaxExpected)
{
    rShunt = rShuntValue;
    
    if(iMaxExpected <= 0) iMaxExpected = vShuntMax / rShunt; //default to full range for rShunt

    shuntLSB = 2.5e-6 / rShunt; //I[A] = regShuntVoltage * shuntLSB

    //calculate best fitting CAL value
    float minimumLSB = iMaxExpected / 32768;
    uint32_t calibrationValue = 0.00512 / (minimumLSB * rShunt) + 0.5;
    if (calibrationValue < 1) calibrationValue = 1;
    if (calibrationValue > 0xffff) calibrationValue = 0xffff;
    writeRegister16(INA226_REG_CALIBRATION, (uint16_t)calibrationValue);

    //calculate currentLSB based on used CAL value
    currentLSB = 0.00512 / (calibrationValue * rShunt);
    powerLSB = currentLSB * 25;

    return true;
}

float INA226::getMaxPossibleCurrent(void)
{
    return (vShuntMax / rShunt);
}

float INA226::getMaxCurrent(void)
{
    float maxCurrent = (currentLSB * 32767);
    float maxPossible = getMaxPossibleCurrent();

    if (maxCurrent > maxPossible)
    {
        return maxPossible;
    } else
    {
        return maxCurrent;
    }
}

float INA226::getMaxShuntVoltage(void)
{
    float maxVoltage = getMaxCurrent() * rShunt;

    if (maxVoltage >= vShuntMax)
    {
        return vShuntMax;
    } else
    {
        return maxVoltage;
    }
}

float INA226::getMaxPower(void)
{
    return (getMaxCurrent() * vBusMax);
}

float INA226::readBusPower(void)
{
    return (readRegister16(INA226_REG_POWER) * powerLSB);
}

float INA226::readShuntCurrent(void)
{ 
    return (readRegister16(INA226_REG_CURRENT) * currentLSB);
}

int16_t INA226::readRawShuntCurrent(void)
{
    return readRegister16(INA226_REG_CURRENT);
}

float INA226::readShuntVoltage(void)
{
    return (readRawShuntVoltage() * 0.0000025);
}

int16_t INA226::readRawShuntVoltage(void)
{
    return readRegister16(INA226_REG_SHUNTVOLTAGE);
}

float INA226::readBusVoltage(void)
{
    int16_t voltage;

    voltage = readRegister16(INA226_REG_BUSVOLTAGE);

    return (voltage * 0.00125);
}

ina226_averages_t INA226::getAverages(void)
{
    uint16_t value;

    value = readRegister16(INA226_REG_CONFIG);
    value &= 0b0000111000000000;
    value >>= 9;

    return (ina226_averages_t)value;
}

ina226_busConvTime_t INA226::getBusConversionTime(void)
{
    uint16_t value;

    value = readRegister16(INA226_REG_CONFIG);
    value &= 0b0000000111000000;
    value >>= 6;

    return (ina226_busConvTime_t)value;
}

ina226_shuntConvTime_t INA226::getShuntConversionTime(void)
{
    uint16_t value;

    value = readRegister16(INA226_REG_CONFIG);
    value &= 0b0000000000111000;
    value >>= 3;

    return (ina226_shuntConvTime_t)value;
}

ina226_mode_t INA226::getMode(void)
{
    uint16_t value;

    value = readRegister16(INA226_REG_CONFIG);
    value &= 0b0000000000000111;

    return (ina226_mode_t)value;
}

void INA226::setMaskEnable(uint16_t mask)
{
    writeRegister16(INA226_REG_MASKENABLE, mask);
}

uint16_t INA226::getMaskEnable(void)
{
    return readRegister16(INA226_REG_MASKENABLE);
}

void INA226::enableShuntOverLimitAlert(void)
{
    writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_SOL);
}

void INA226::enableShuntUnderLimitAlert(void)
{
    writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_SUL);
}

void INA226::enableBusOvertLimitAlert(void)
{
    writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_BOL);
}

void INA226::enableBusUnderLimitAlert(void)
{
    writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_BUL);
}

void INA226::enableOverPowerLimitAlert(void)
{
    writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_POL);
}

void INA226::enableConversionReadyAlert(void)
{
    writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_CNVR);
}

void INA226::disableAlerts(void)
{
    writeRegister16(INA226_REG_MASKENABLE, 0);
}

void INA226::setBusVoltageLimit(float voltage)
{
    uint16_t value = voltage / 0.00125;
    writeRegister16(INA226_REG_ALERTLIMIT, value);
}

void INA226::setShuntVoltageLimit(float voltage)
{
    uint16_t value = voltage / 0.0000025;
    writeRegister16(INA226_REG_ALERTLIMIT, value);
}

void INA226::setPowerLimit(float watts)
{
    uint16_t value = watts / powerLSB;
    writeRegister16(INA226_REG_ALERTLIMIT, value);
}

void INA226::setAlertInvertedPolarity(bool inverted)
{
    uint16_t temp = getMaskEnable();

    if (inverted)
    {
        temp |= INA226_BIT_APOL;
    } else
    {
        temp &= ~INA226_BIT_APOL;
    }

    setMaskEnable(temp);
}

void INA226::setAlertLatch(bool latch)
{
    uint16_t temp = getMaskEnable();

    if (latch)
    {
        temp |= INA226_BIT_LEN;
    } else
    {
        temp &= ~INA226_BIT_LEN;
    }

    setMaskEnable(temp);
}

bool INA226::isMathOverflow(void)
{
    return ((getMaskEnable() & INA226_BIT_OVF) == INA226_BIT_OVF);
}

bool INA226::isConversionReady(void)
{
    return ((getMaskEnable() & INA226_BIT_CVRF) != 0); //this clears the flag
}


bool INA226::isAlert(void)
{
    return ((getMaskEnable() & INA226_BIT_AFF) == INA226_BIT_AFF);
}

int16_t INA226::readRegister16(uint8_t reg)
{
    int16_t value;

    _i2c->beginTransmission(inaAddress);
    _i2c->write(reg);
    _i2c->endTransmission();

    _i2c->requestFrom(inaAddress, 2);
    uint8_t vha = _i2c->read();
    uint8_t vla = _i2c->read();
    value = vha << 8 | vla;

    return value;
}

void INA226::writeRegister16(uint8_t reg, uint16_t val)
{
    uint8_t vla;
    vla = (uint8_t)val;
    val >>= 8;

    _i2c->beginTransmission(inaAddress);
    _i2c->write(reg);
    _i2c->write((uint8_t)val);
    _i2c->write(vla);
    _i2c->endTransmission();
}
