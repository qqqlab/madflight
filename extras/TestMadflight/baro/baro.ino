//test program for baro.h - used with madflight 1.2.3

#define MF_TEST MF_TEST_BARO

#define BARO_USE  BARO_USE_BMP390 // BARO_USE_BMP390, BARO_USE_BMP388, BARO_USE_BMP280, BARO_USE_MS5611, BARO_USE_NONE
#define BARO_I2C_ADR 0x77
#define HW_PIN_I2C_SDA    23
#define HW_PIN_I2C_SCL    22

#include <madflight.h>

void setup() {
  Serial.begin(115200);
  while(!Serial);

  hw_setup(); //hardware specific setup for spi and Wire (see hw_xxx.h)
 
  int rv = baro.setup(100);
  if(rv) {
    while(1) {
      Serial.printf("sensor error %d\n",rv);
      delay(1000);
    }
  }
}

uint32_t ts = 0;

void loop() {
  if(baro.update()) {
    uint32_t now = micros();
    Serial.printf("ts:%d\tp:%f\tt:%f\talt:%f\n",(int)(now-ts),baro.press,baro.temp,baro.alt);
    ts = now;
  }
}

