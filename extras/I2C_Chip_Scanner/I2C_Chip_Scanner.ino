#define APP_DATE "2025-08-18"

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

#define RP2040_SDA_PIN 32 //RP2040/RP2350A Wire: 0, 4(default), 8, 12, 16, 20   Wire1: 2, 6, 10, 14, 18, 26(default)
#define RP2040_SCL_PIN 33 //RP2040/RP2350A Wire: 1, 5(default), 9, 13, 17, 21   Wire1: 3, 7, 11, 15, 19, 27(default)

#define STM32_SDA_PIN PB9
#define STM32_SCL_PIN PB8

#define I2C_FRQ 100000




void setup() {
  Serial.begin(115200);

  //start WIRE
  #if defined ARDUINO_ARCH_ESP32
    i2c->begin(ESP32_SDA_PIN, ESP32_SCL_PIN, I2C_FRQ);
  #elif defined ARDUINO_ARCH_RP2040
    i2c->setSDA(RP2040_SDA_PIN);
    i2c->setSCL(RP2040_SCL_PIN);
    i2c->setClock(I2C_FRQ);
    i2c->setTimeout(25, true); //timeout, reset_with_timeout
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
  uint8_t num_adr_found = 0;
  uint8_t adr_found[128];
  String adr_msg[128];

  Serial.printf("===========================\n");
  Serial.printf("I2C Chip Scanner " APP_DATE "\n");
  Serial.printf("===========================\n");

  //scan addresses to find devices
  Serial.printf("Step 1: Scanning Adressses ... ");
  for (uint8_t adr = 1; adr < 128; adr++) {
    if (i2c_TestAddress(adr)) {
      Serial.printf("0x%02X(%d) ",adr,adr);
      adr_found[num_adr_found] = adr;
      num_adr_found++;
    }
  }
  Serial.printf(" - Found %d devices\n", num_adr_found);

  //try to identify the devices
  Serial.printf("Step 2: Identifying Devices ...\n");
  for (uint8_t i = 0; i < num_adr_found; i++) {
    i2c_identify_chip(adr_found[i], adr_msg[i]);
  }

  Serial.printf("\nStep 3: Summary\n\n");
  for (uint8_t i = 0; i < num_adr_found; i++) {
    Serial.printf("address 0x%02X (decimal %3d) %s\n", adr_found[i], adr_found[i], adr_msg[i].c_str());
  }

  delay(1000);
}

struct test_struct{
    uint8_t adr1;
    uint8_t adr2;    
    uint16_t reg;
    uint8_t len;
    uint32_t expected;
    String descr;
};

//table of addresses, register and expected reply
test_struct tests[] = {
// adr1  adr2  reg  len exp       descr
  {0x0D, 0x0D, 0x0D, 1, 0xFF, "QMC5883L magnetometer"},
  {0x1E, 0x1E, 0x0A, 3, 0x483433, "HMC5883L magnetometer"}, // ID-Reg-A 'H', ID-Reg-B '4', ID-Reg-C '3'
  {0x30, 0x30, 0x00, 2, 0x01B0, "HM01B0 camera"},
  {0x30, 0x30, 0x0A, 1, 0x26, "OV2460 camera"},
  {0x30, 0x30, 0x0A, 2, 0x9711, "OV9712 / OV9211 camera"},
  {0x3C, 0x3C, 0x300A, 2, 0x3660, "OV3660 camera"},
  {0x3C, 0x3C, 0x300A, 2, 0x5640, "OV5640 camera"},  
  {0x40, 0x4F, 0x00, 2, 0x399F, "INA219 current sensor"},
  {0x40, 0x4F, 0x00, 2, 0x4127, "INA226 current sensor"},
  {0x48, 0x4F, 0x01, 2, 0x8583, "ADS1113 / ADS1114 / ADS1115 ADC"},  
  {0x42, 0x42, 0x00, 1, 0x9B, "GC0308 camera"},
  {0x42, 0x42, 0x0A, 1, 0x76, "OV7670 camera"},
  {0x42, 0x42, 0x0A, 1, 0x77, "OV7725 camera"},
  {0x46, 0x47, 0x01, 1, 0x50, "BMP580 / BMP581 barometer"},
  {0x46, 0x47, 0x01, 1, 0x51, "BMP585 barometer"},  
  {0x68, 0x69, 0x00, 1, 0xEA, "ICM20948 9DOF motion"},
  {0x68, 0x69, 0x00, 1, 0x68, "MPU3050 motion"},
  {0x68, 0x69, 0x00, 1, 0x69, "MPU3050 motion"},
  {0x68, 0x69, 0x75, 1, 0x12, "ICM20602 6DOF motion"},
  {0x68, 0x69, 0x75, 1, 0x19, "MPU6886 6DOF motion"},
  {0x68, 0x69, 0x75, 1, 0x47, "ICM42688P 6DOF motion"},  
  {0x68, 0x69, 0x75, 1, 0x68, "MPU6000 / MPU6050 / MPU9150 6/9DOF motion"}, 
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
  {0x76, 0x77, 0x0D, 1, 0x10, "DPS310 / HP303B / SPL06 pressure"},
  {0x76, 0x77, 0x8F, 1, 0x80, "HP203B pressure"},
  {0x76, 0x77, 0xD0, 1, 0x55, "BMP180 pressure"},
  {0x76, 0x77, 0xD0, 1, 0x56, "BMP280 pressure"},
  {0x76, 0x77, 0xD0, 1, 0x57, "BMP280 pressure"},
  {0x76, 0x77, 0xD0, 1, 0x58, "BMP280 pressure"},
  {0x76, 0x77, 0xD0, 1, 0x60, "BME280 pressure, temperature, humidity"},
  {0x76, 0x77, 0xD0, 1, 0x61, "BME680 pressure, temperature, humidity, gas"},
  {0x7C, 0x7C, 0x00, 1, 0x90, "QMC6309 magnetometer"},

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

void i2c_identify_chip(uint8_t adr, String &msg) {
  //try adr,reg,result match
  int i = 0;
  while(tests[i].adr1) {
    uint8_t adr1 = tests[i].adr1;
    uint8_t adr2 = tests[i].adr2;    
    uint16_t reg = tests[i].reg;
    uint8_t len = tests[i].len;    
    uint32_t expected = tests[i].expected;
    uint32_t received = 0;
    if(adr1<=adr && adr<=adr2) {
      uint8_t data[4];
      Serial.printf("try: %s --> read adr:0x%02X reg:0x%02X len:%d --> ", tests[i].descr.c_str(), adr, reg, len);
      i2c_ReadRegs(adr, reg, data, len, true);
      for(int i=0;i<len;i++) received = (received<<8) + data[i];   
      Serial.printf("received:0x%02X ", (int)received);
      if(received == expected) {
        msg = tests[i].descr;
        Serial.printf("--> %s\n",msg.c_str());
        return;
      } 
      Serial.printf("\n");
    }
    i++;
  }
  
  //try address only match
  i = 0;  
  while(adrtests[i].adr1) {
    int adr1 = adrtests[i].adr1;
    int adr2 = adrtests[i].adr2;    
    if(adr1<=adr && adr<=adr2) {
      msg = adrtests[i].descr + String(" - ADDRESS MATCH ONLY");
      return;
    }
    i++;
  }

  msg = "UNKNOWN";
}

bool i2c_TestAddress(uint8_t adr) {
  i2c->beginTransmission(adr);       // Begin I2C transmission Address (i)
  return (i2c->endTransmission() == 0); // Receive 0 = success (ACK response) 
}

void i2c_WriteByte( uint8_t adr, uint8_t reg ) {
  i2c->beginTransmission(adr); 
  i2c->write(reg);
  i2c->endTransmission();
}

//returns -1 on fail
int i2c_ReadByte( uint8_t adr ) {
  int rv = -1;
  uint8_t data;
  uint8_t bytesReceived = i2c->requestFrom(adr, 1);
  if(bytesReceived == 1) {
    i2c->readBytes(&data, 1);
    rv = data;
  }
  //i2c->endTransmission(); -- don't do this
  return rv;
}

void i2c_WriteReg( uint8_t adr, uint8_t reg, uint8_t data ) {
  i2c->beginTransmission(adr); 
  i2c->write(reg);
  i2c->write(data);
  i2c->endTransmission();
}



int i2c_ReadRegs( uint8_t adr, uint16_t reg, uint8_t *data, uint8_t n, bool stop ) {
  i2c->beginTransmission(adr);
  if(reg>0xff) i2c->write(reg>>8);
  i2c->write(reg);
  i2c->endTransmission(stop); //false = repeated start, true = stop + start
  int bytesReceived = i2c->requestFrom(adr, n);
  if(bytesReceived > 0) {
    i2c->readBytes(data, bytesReceived);
  }
  //i2c->endTransmission(); -- don't do this
  return bytesReceived;
}

int i2c_ReadRegs( uint8_t adr, uint8_t reg, uint8_t *data, uint8_t n) {
  return i2c_ReadRegs( adr, reg, data, n, false);
}

//returns -1 on fail
int i2c_ReadReg( uint8_t adr, uint8_t reg, bool stop ) {
    uint8_t data = 0;
    int bytesReceived = i2c_ReadRegs(adr, reg, &data, 1);
    if(bytesReceived == 1) {
      return data;
    }
    return -1;
}

int i2c_ReadReg( uint8_t adr, uint8_t reg) {
  return i2c_ReadReg( adr, reg, false );
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
