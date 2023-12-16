//test program for MPU sensor lib

#include <SPI.h>
#include "MPU9250.h"

#define SS_PIN   17

MPU9250 mpu(&SPI, SS_PIN); 

void setup() {
	Serial.begin(115200);
	SPI.begin();
	mpu.begin();
}

void loop() {
//  test_read();
  test_motion9NED();
//  test_hispeed();
}


void test_read() {
  mpu.read();
  Serial.printf("gx:%+.2f\tgy:%+.2f\tgz:%+.2f\t",mpu.gyro[0],mpu.gyro[1],mpu.gyro[2]);
  Serial.printf("ax:%+.2f\tay:%+.2f\taz:%+.2f\t",mpu.accel[0],mpu.accel[1],mpu.accel[2]);
  Serial.printf("mx:%+.2f\tmy:%+.2f\tmz:%+.2f\t",mpu.mag[0],mpu.mag[1],mpu.mag[2]);  
  Serial.printf("temp:%+.2f\n",mpu.temperature);
	delay(10);
}

void test_motion9NED() {
  float ax,ay,az,gx,gy,gz,mx,my,mz;
  mpu.getMotion9NED(&ax,&ay,&az,&gx,&gy,&gz,&mx,&my,&mz);
  Serial.printf("gx:%+.2f\tgy:%+.2f\tgz:%+.2f\t",gx,gy,gz);
  Serial.printf("ax:%+.2f\tay:%+.2f\taz:%+.2f\t",ax,ay,az);
  Serial.printf("mx:%+.2f\tmy:%+.2f\tmz:%+.2f\n",mx,my,mz);  
	delay(10);
}


//high speed measurement
const int MEASCNT = 1000;
float meas[MEASCNT+10];
int cnt = 0;

void test_hispeed() {
  uint32_t t0 = micros();
  uint32_t tt = t0;
  while(cnt<MEASCNT) {
    uint32_t t = micros();
    mpu.read();
    uint32_t dt = micros() - t;
    meas[cnt++]=t-t0;
    meas[cnt++]=dt;
    meas[cnt++]=mpu.gyro[0];
    meas[cnt++]=mpu.mag[0];
    while(micros()-tt<500);
    tt+=500;
  }
  cnt = 0;
  while(cnt<MEASCNT){
    Serial.print(cnt/4);
    for(int i=0;i<4;i++) {
      Serial.print('\t');
      Serial.print(meas[cnt++]);
    }
    Serial.println();
    delay(1);
  }
  delay(5000);
}
