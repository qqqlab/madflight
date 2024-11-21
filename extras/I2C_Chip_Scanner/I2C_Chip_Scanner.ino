/*====================================================================
 I2C Chip Scanner - scan I2C bus and try to identify chips

Example output:

===================
I2C Chip Scanner
===================
I2C: Scanning ...
I2C: Found address: 0x1E (decimal 30)
      MATCH (adr=0x1E reg=0x0A result=0x483433): HMC5883L
I2C: Found address: 0x29 (decimal 41)
      POTENTIAL MATCH (address 0x29 match only): VL53L7CX      
I2C: Found address: 0x68 (decimal 104)
      MATCH (adr=0x68 reg=0x75 result=0x68): MPU6000, MPU6050, or MPU9150
I2C: Found address: 0x77 (decimal 119)
      MATCH (adr=0x77 reg=0xD0 result=0x55): BMP180
I2C: Found 4 device(s)
====================================================================*/

#include "Wire.h"

TwoWire *i2c = &Wire;

#define ESP32_SDA_PIN 23
#define ESP32_SCL_PIN 22

#define RP2040_SDA_PIN 20 //Wire: 0, 4(default), 8, 12, 16, 20   Wire1: 2, 6, 10, 14, 18, 26(default)
#define RP2040_SCL_PIN 21 //Wire: 1, 5(default), 9, 13, 17, 21   Wire1: 3, 7, 11, 15, 19, 27(default)

#define STM32_SDA_PIN PB9
#define STM32_SCL_PIN PB8

#define I2C_FRQ 400000




void setup() {
  Serial.begin(115200);

  //start WIRE
  #if defined ARDUINO_ARCH_ESP32
    i2c->begin(ESP32_SDA_PIN, ESP32_SCL_PIN, I2C_FRQ);
  #elif defined ARDUINO_ARCH_RP2040
    i2c->setSDA(RP2040_SDA_PIN);
    i2c->setSCL(RP2040_SCL_PIN);
    i2c->setClock(I2C_FRQ);
    i2c->begin();
  #elif defined ARDUINO_ARCH_STM32
    i2c->setSDA(STM32_SDA_PIN);
    i2c->setSCL(STM32_SCL_PIN);
    i2c->setClock(I2C_FRQ);
    i2c->begin();    
  #else 
    #warning "Using default I2C pins"
    i2c->begin();
  #endif
}

void loop() {
  Serial.println("===================\nI2C Chip Scanner\n===================");

  Serial.printf("I2C: Scanning ...\n");
  int num_adr_found = 0;
  for (uint8_t adr = 8; adr < 120; adr++) {
    i2c->beginTransmission(adr);       // Begin I2C transmission Address (i)
    if (i2c->endTransmission() == 0) { // Receive 0 = success (ACK response) 
      Serial.printf("I2C: Found address: 0x%02X (decimal %d)\n",adr,adr);
      num_adr_found++;
      i2c_identify_chip(adr);
    }
  }
  Serial.printf("I2C: Found %d device(s)\n", num_adr_found);
  delay(1000);
}

struct test_struct{
    uint8_t adr1;
    uint8_t adr2;    
    uint8_t reg;
    uint8_t len;
    uint32_t expected;
    String descr;
};

//table of addresses, register and expected reply
test_struct tests[] = {
// adr1  adr2  reg  len exp       descr
  {0x0D, 0x0D, 0x0D, 1, 0xFF, "QMC5883L magnetometer"},
  {0x1E, 0x1E, 0x0A, 3, 0x483433, "HMC5883L magnetometer"}, // ID-Reg-A 'H', ID-Reg-B '4', ID-Reg-C '3'
  {0x40, 0x4F, 0x00, 2, 0x399F, "INA219 current sensor"},
  {0x40, 0x4F, 0x00, 2, 0x4127, "INA226 current sensor"},
  {0x48, 0x4F, 0x01, 2, 0x8583, "ADS1113, ADS1114, or ADS1115 ADC"},
  {0x68, 0x69, 0x00, 1, 0xEA, "ICM20948 9DOF motion"},
  {0x68, 0x69, 0x00, 1, 0x68, "MPU3050 motion"},
  {0x68, 0x69, 0x00, 1, 0x69, "MPU3050 motion"},
  {0x68, 0x69, 0x75, 1, 0x12, "ICM20602 6DOF motion"},
  {0x68, 0x69, 0x75, 1, 0x19, "MPU6886 6DOF motion"},
  {0x68, 0x69, 0x75, 1, 0x47, "ICM42688P 6DOF motion"},  
  {0x68, 0x69, 0x75, 1, 0x68, "MPU6000, MPU6050, or MPU9150 6/9DOF motion"}, 
  {0x68, 0x69, 0x75, 1, 0x69, "FAKE MPU6050, got WHO_AM_I=0x69, real chip returns 0x68"},   
  {0x68, 0x69, 0x75, 1, 0x70, "MPU6500 6DOF motion"},
  {0x68, 0x69, 0x75, 1, 0x71, "MPU9250 9DOF motion"},
  {0x68, 0x69, 0x75, 1, 0x72, "FAKE MPU6050, got WHO_AM_I=0x72, real chip returns 0x68"},
  {0x68, 0x69, 0x75, 1, 0x73, "MPU9255 9DOF motion"},  
  {0x68, 0x69, 0x75, 1, 0x74, "MPU9515 motion"},
  {0x68, 0x69, 0x75, 1, 0x75, "FAKE MPU9250, got WHO_AM_I=0x75, real chip returns 0x71"}, 
  {0x68, 0x69, 0x75, 1, 0x78, "FAKE MPU9250, got WHO_AM_I=0x78, real chip returns 0x71"},  
  {0x68, 0x69, 0x75, 1, 0x98, "ICM20689 6DOF motion"},
  {0x76, 0x77, 0x00, 1, 0x50, "BMP388 pressure"},
  {0x76, 0x77, 0x00, 1, 0x60, "BMP390 pressure"},  
  {0x76, 0x77, 0x0D, 1, 0x10, "DPS310, HP303B, or SPL06 pressure"},
  {0x76, 0x77, 0xD0, 1, 0x55, "BMP180 pressure"},
  {0x76, 0x77, 0xD0, 1, 0x56, "BMP280 pressure"},
  {0x76, 0x77, 0xD0, 1, 0x57, "BMP280 pressure"},
  {0x76, 0x77, 0xD0, 1, 0x58, "BMP280 pressure"},
  {0x76, 0x77, 0xD0, 1, 0x60, "BME280 pressure, temperature, humidity"},
  {0x76, 0x77, 0xD0, 1, 0x61, "BME680 pressure, temperature, humidity, gas sensor"},

  {0,0,0,0,0,""} //end
};

struct adrtest_struct{
    uint8_t adr1;
    uint8_t adr2;    
    char descr[60];
};

//address only tests
adrtest_struct adrtests[] = {
// adr1  adr2  descr
  {0x29, 0x29, "VL53L7CX"}, //no public register map available
  {0x77, 0x77, "MS5611"}, //no ID register
  {0,0,""} //end
};  

void i2c_identify_chip(uint8_t adr) {
  bool found = false;
  
  uint32_t tried[256] = {0};

  //try adr,reg,result match
  int i = 0;
  while(tests[i].adr1) {
    uint8_t adr1 = tests[i].adr1;
    uint8_t adr2 = tests[i].adr2;    
    uint8_t reg = tests[i].reg;
    uint8_t len = tests[i].len;    
    uint32_t expected = tests[i].expected;
    uint32_t received = 0;
    if(adr1<=adr && adr<=adr2) {
      uint8_t data[4];
      i2c_ReadRegs(adr, reg, data, len);
      for(int i=0;i<len;i++) received = (received<<8) + data[i];
      tried[reg] = received;      
      if(received == expected) {
        Serial.printf("      MATCH: reg=0x%02X result=0x%02X -> %s\n", reg, (int)received, tests[i].descr.c_str());
        found = true;
      }
    }
    i++;
  }
  
  if(!found) {
    //show tries
    for(int i=0;i<256;i++) if(tried[i]!=0) {
      Serial.printf("      tried: reg=0x%02X result=0x%02X\n", i, (int)tried[i]); 
    }

    //try address only match
    int i = 0;    
    while(adrtests[i].adr1) {
      int adr1 = adrtests[i].adr1;
      int adr2 = adrtests[i].adr2;    
      if(adr1<=adr && adr<=adr2) {
        Serial.printf("      POTENTIAL MATCH (address 0x%02X match only) -> %s\n", adr, adrtests[i].descr);
      }
      i++;
    }
  }
}

void WriteReg( uint8_t adr, uint8_t reg, uint8_t data ) {
  i2c->beginTransmission(adr); 
  i2c->write(reg);       
  i2c->write(data);              
  i2c->endTransmission();
}

unsigned int i2c_ReadReg( uint8_t adr, uint8_t reg ) {
    uint8_t data = 0;
    i2c_ReadRegs(adr, reg, &data, 1);
    return data;
}

void i2c_ReadRegs( uint8_t adr, uint8_t reg, uint8_t *data, uint8_t n ) {
  i2c->beginTransmission(adr); 
  i2c->write(reg);
  i2c->endTransmission(false); //false = repeated start
  uint8_t bytesReceived = i2c->requestFrom(adr, n);
  if(bytesReceived == n) {
    i2c->readBytes(data, bytesReceived);
  }
}

void i2c_scan() {
  Serial.printf("I2C: Scanning ...\n");
  byte count = 0;
  i2c->begin();
  for (byte i = 8; i < 120; i++) {
    i2c->beginTransmission(i);          // Begin I2C transmission Address (i)
    if (i2c->endTransmission() == 0) { // Receive 0 = success (ACK response) 
      Serial.printf("I2C: Found address: 0x%02X (%d)\n",i,i);
      count++;
    }
  }
  Serial.printf("I2C: Found %d device(s)\n", count);      
}
