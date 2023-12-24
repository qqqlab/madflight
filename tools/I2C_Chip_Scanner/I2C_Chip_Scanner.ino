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

#define ESP32_SDA_PIN 23
#define ESP32_SCL_PIN 22

#define RP2040_SDA_PIN 20
#define RP2040_SCL_PIN 21

#define I2C_FRQ 400000

#include "Wire.h"

void setup() {
  Serial.begin(115200);

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
}

void loop() {
  Serial.println("===================\nI2C Chip Scanner\n===================");

  Serial.printf("I2C: Scanning ...\n");
  int num_adr_found = 0;
  for (uint8_t adr = 8; adr < 120; adr++) {
    Wire.beginTransmission(adr);       // Begin I2C transmission Address (i)
    if (Wire.endTransmission() == 0) { // Receive 0 = success (ACK response) 
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
    char descr[60];
};

//table of addresses, register and expected reply
test_struct tests[] = {
// adr1  adr2  reg  len exp       descr
  {0x0D, 0x0D, 0x0D, 1, 0xFF, "QMC5883L"},
  {0x1E, 0x1E, 0x0A, 3, 0x483433, "HMC5883L"}, // ID-Reg-A 'H', ID-Reg-B '4', ID-Reg-C '3'
  {0x40, 0x4F, 0x00, 2, 0x399F, "INA219"},
  {0x40, 0x4F, 0x00, 2, 0x4127, "INA226"},
  {0x48, 0x4F, 0x01, 2, 0x8583, "ADS1113, ADS1114, or ADS1115" },  
  {0x68, 0x69, 0x00, 1, 0x68, "MPU3050"},
  {0x68, 0x69, 0x00, 1, 0x69, "MPU3050"},
  {0x68, 0x69, 0x75, 1, 0x19, "MPU6886"},
  {0x68, 0x69, 0x75, 1, 0x68, "MPU6000, MPU6050, or MPU9150"}, 
  {0x68, 0x69, 0x75, 1, 0x70, "MPU6500"},
  {0x68, 0x69, 0x75, 1, 0x71, "MPU9250"},
  {0x68, 0x69, 0x75, 1, 0x72, "FAKE MPU6050, got WHO_AM_I=0x72, real chip returns 0x68"},  
  {0x68, 0x69, 0x75, 1, 0x98, "ICM-20689"},
  {0x76, 0x76, 0xD0, 1, 0x56, "BMP280"},
  {0x76, 0x76, 0xD0, 1, 0x57, "BMP280"},
  {0x76, 0x76, 0xD0, 1, 0x58, "BMP280"},
  {0x76, 0x76, 0xD0, 1, 0x60, "BME280"},
  {0x76, 0x76, 0xD0, 1, 0x61, "BMP680"},
  {0x76, 0x77, 0x0D, 1, 0x10, "DPS310 or HP303B"},   
  {0x77, 0x77, 0xD0, 1, 0x55, "BMP180"},

  
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
      if(received == expected) {
        Serial.printf("      MATCH (adr=0x%02X reg=0x%02X result=0x%02X): %s\n", adr, reg, received, tests[i].descr);
        found = true;
      }
    }
    i++;
  }
  
  if(!found) {
    //try address only match
    int i = 0;    
    while(adrtests[i].adr1) {
      int adr1 = adrtests[i].adr1;
      int adr2 = adrtests[i].adr2;    
      if(adr1<=adr && adr<=adr2) {
        Serial.printf("      POTENTIAL MATCH (address 0x%02X match only): %s\n", adr, adrtests[i].descr);
      }
      i++;
    }
  }
}

void WriteReg( uint8_t adr, uint8_t reg, uint8_t data ) {
  Wire.beginTransmission(adr); 
  Wire.write(reg);       
  Wire.write(data);              
  Wire.endTransmission();
}

unsigned int i2c_ReadReg( uint8_t adr, uint8_t reg ) {
    uint8_t data = 0;
    i2c_ReadRegs(adr, reg, &data, 1);
    return data;
}

void i2c_ReadRegs( uint8_t adr, uint8_t reg, uint8_t *data, uint8_t n ) {
  Wire.beginTransmission(adr); 
  Wire.write(reg);
  Wire.endTransmission(false); //false = repeated start
  uint8_t bytesReceived = Wire.requestFrom(adr, n);
  if(bytesReceived == n) {
    Wire.readBytes(data, bytesReceived);
  }
}

void i2c_scan() {
  Serial.printf("I2C: Scanning ...\n");
  byte count = 0;
  Wire.begin();
  for (byte i = 8; i < 120; i++) {
    Wire.beginTransmission(i);          // Begin I2C transmission Address (i)
    if (Wire.endTransmission() == 0) { // Receive 0 = success (ACK response) 
      Serial.printf("I2C: Found address: 0x%02X (%d)\n",i,i);
      count++;
    }
  }
  Serial.printf("I2C: Found %d device(s)\n", count);      
}