/*====================================================================
 mag.ino - demo program for mag sensors
====================================================================*/

#define ESP32_SDA_PIN 23
#define ESP32_SCL_PIN 22

#define RP2040_SDA_PIN 20
#define RP2040_SCL_PIN 21

#define I2C_FRQ 400000

#include "Wire.h"

//void (*mag_read_uT)(float*,float*,float*); 

#include "QMC5883L.h"
QMC5883L<TwoWire> mag(&Wire);

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("===================\nmag.ino\n===================");

  //start Wire
  #if defined ARDUINO_ARCH_ESP32
    Wire.begin(ESP32_SDA_PIN, ESP32_SCL_PIN, I2C_FRQ);
  #elif defined ARDUINO_ARCH_RP2040
    Wire.setSDA(RP2040_SDA_PIN);
    Wire.setSCL(RP2040_SCL_PIN);
    Wire.setClock(I2C_FRQ);
    Wire.begin();
  #else 
    #warning "Using default I2C pins"
    Wire.begin();
  #endif

  //start Mag
  mag.begin();
}

void loop() {
  float mx,my,mz;
  mag.read_uT(&mx,&my,&mz);
  Serial.printf("mx:%f\tmy:%f\tmz:%f\n",mx,my,mz);
  delay(10);
}
