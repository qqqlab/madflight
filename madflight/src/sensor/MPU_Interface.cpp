#include "MPU_Interface.h"

//================================================================
// Interface
//================================================================

unsigned int MPU_Interface::WriteReg(uint8_t reg, uint8_t data)
{
  if(_use_spi) 
    return _WriteReg_SPI(reg, data);
  else
    return _WriteReg_I2C(reg, data);
}

void MPU_Interface::ReadRegs( uint8_t reg, uint8_t *data, int n )
{
  if(_use_spi) 
    _ReadRegs_SPI(reg, data, n);
  else
    _ReadRegs_I2C(reg, data, n);
}

unsigned int MPU_Interface::ReadReg( uint8_t reg )
{
    uint8_t data = 0;
    ReadRegs(reg, &data, 1);
    return data;
}

//================================================================
// SPI
//================================================================

#define READ_FLAG   0x80

void MPU_Interface::set_spi_freq(int freq) {
    _spi_freq = freq;
}

unsigned int MPU_Interface::_WriteReg_SPI( uint8_t reg, uint8_t data )
{
    _spi->beginTransaction(SPISettings(_spi_freq, MSBFIRST, SPI_MODE3));
    digitalWrite(_spi_cs, LOW);
    _spi->transfer(reg);
    unsigned int temp_val = _spi->transfer(data);
    digitalWrite(_spi_cs, HIGH);
    _spi->endTransaction();
    return temp_val;
}
void MPU_Interface::_ReadRegs_SPI( uint8_t reg, uint8_t *buf, int n )
{
    _spi->beginTransaction(SPISettings(_spi_freq, MSBFIRST, SPI_MODE3));
    digitalWrite(_spi_cs, LOW);
    _spi->transfer(reg | READ_FLAG);
    for(int i = 0; i < n; i++) {
        buf[i] = _spi->transfer(0x00);
    }
    digitalWrite(_spi_cs, HIGH);
    _spi->endTransaction();
}

//================================================================
// I2C
//================================================================

unsigned int MPU_Interface::_WriteReg_I2C( uint8_t reg, uint8_t data )
{
  _i2c->beginTransmission(_i2c_adr); 
  _i2c->write(reg);
  _i2c->write(data);
  _i2c->endTransmission();
  return 0;
}

void MPU_Interface::_ReadRegs_I2C( uint8_t reg, uint8_t *data, uint8_t n )
{
  _i2c->beginTransmission(_i2c_adr); 
  _i2c->write(reg);
  _i2c->endTransmission(false); //false = repeated start
  uint8_t bytesReceived = _i2c->requestFrom(_i2c_adr, n);
  if(bytesReceived == n) {
    _i2c->readBytes(data, bytesReceived);
  }
}
