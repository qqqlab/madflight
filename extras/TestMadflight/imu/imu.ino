//test program for imu.h - used with madflight 1.1.3

#define IMU_USE  IMU_USE_SPI_MPU6500 // IMU_USE_SPI_BMI270, IMU_USE_SPI_MPU9250, IMU_USE_SPI_MPU6500, IMU_USE_SPI_MPU6000, IMU_USE_I2C_MPU9250, IMU_USE_I2C_MPU9150, IMU_USE_I2C_MPU6500, IMU_USE_I2C_MPU6050, IMU_USE_I2C_MPU6000
#define IMU_I2C_ADR  0x68

/* Override default pins

//ESP32 WeMos LOLIN32-Lite
//IMU SPI:
#define HW_PIN_SPI_MISO  25
#define HW_PIN_SPI_MOSI  14
#define HW_PIN_SPI_SCLK  12
#define HW_PIN_IMU_CS    32
#define HW_PIN_IMU_EXTI  33 //external interrupt pin

//I2C for BARO, MAG, BAT sensors and for IMU if not using SPI
#define HW_PIN_I2C_SDA   23
#define HW_PIN_I2C_SCL   19
//*/

#define MF_TEST MF_TEST_IMU
#include <madflight.h>


void setup() {
  Serial.begin(115200);
  while(!Serial);

  hal_setup(); //hardware specific setup for spi and Wire (see hw_xxx.h)

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
  Serial.printf("imu_loop_cnt:%d ax:%.2f ay:%.2f az:%.2f gx:%.2f gy:%.2f gz:%.2f mx:%.2f my:%.2f mz:%.2f\n",imu_loop_cnt, imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz, imu.mx, imu.my, imu.mz);
  delay(1000);
}

