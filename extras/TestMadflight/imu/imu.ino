//test program for imu.h - used with madflight 1.1.3

#define IMU_USE  IMU_USE_I2C_MPU9250
#define IMU_I2C_ADR  0x68

#define MF_TEST MF_TEST_IMU
#include <madflight.h>


void setup() {
  Serial.begin(115200);
  while(!Serial);

  hw_setup(); //hardware specific setup for spi and Wire (see hw_xxx.h)

  //IMU: keep on trying until no error is returned (some sensors need a couple of tries...)
  while(true) {       
    int rv = imu.setup(); //request 1000 Hz sample rate, returns 0 on success, positive on error, negative on warning
    if(rv<=0) break;
    Serial.print("IMU: init failed rv= " + String(rv) + ". Retrying...\n");
    i2c->begin();
    delay(1000);
  }

  //start IMU update handler
  imu.onUpdate = imu_loop;
  while(!imu.waitNewSample()) Serial.println("IMU interrupt not firing.");
}

int imu_loop_cnt = 0;

void imu_loop() {
  imu_loop_cnt++;
}

void loop() {
  Serial.printf("imu_loop_cnt:%d az:%f gz:%f mz:%f\n",imu_loop_cnt, imu.az, imu.gz, imu.mz);
  delay(1000);
}

