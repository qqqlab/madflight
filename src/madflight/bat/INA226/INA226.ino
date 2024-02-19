
#include <Wire.h>

//RP2040 I2C:
const int HW_PIN_I2C_SDA  = 20; //Wire: 0, 4(default), 8, 12, 16, 20   Wire1: 2, 6, 10, 14, 18, 26(default)
const int HW_PIN_I2C_SCL  = 21; //Wire: 1, 5(default), 9, 13, 17, 21   Wire1: 3, 7, 11, 15, 19, 27(default)
typedef TwoWire HW_WIRETYPE; //define the class to use for I2C
HW_WIRETYPE *i2c = &Wire;

void hw_setup() {
  //I2C
  i2c->setSDA(HW_PIN_I2C_SDA);
  i2c->setSCL(HW_PIN_I2C_SCL);
  i2c->setClock(1000000);
  i2c->begin();
}

#include "INA226.h"
INA226 ina;

void ina_setup() {
  float Rshunt = 0.01; //ohm
  float iMaxExpected = 0.080 / Rshunt; // ampere (max 80mv shunt voltage)

  // Default INA226 address is 0x40
  ina.begin(i2c);

  // Configure INA226 -> sample time = 2 * 128 * 140us = 36ms => 28Hz
  ina.configure(INA226_AVERAGES_128, INA226_BUS_CONV_TIME_140US, INA226_SHUNT_CONV_TIME_140US, INA226_MODE_SHUNT_BUS_CONT);

  // Calibrate INA226.
  ina.calibrate(Rshunt, iMaxExpected);
}

bool ina_update() {
  static uint32_t ts = micros();
  if(!ina.isConversionReady()) return false;
  uint32_t now = micros();
  uint32_t dt = now - ts;
  ts = now;
  Serial.printf("dt:%d V:%.5f\tW:%.5f\tVshunt:%.5f\tA:%.5f\n", int(dt), ina.readBusVoltage(), ina.readBusPower(), ina.readShuntVoltage(), ina.readShuntCurrent());
  return true;
}


void setup() 
{
  Serial.begin(115200);

  for(int i=0;i<5;i++) {Serial.println("starting...");delay(500);}

  hw_setup();

  ina_setup();
}


void loop()
{
  ina_update();
}

