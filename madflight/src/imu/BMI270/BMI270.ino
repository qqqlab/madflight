// madflight https://github.com/qqqlab/madflight
// Test Program for BMI270 SPI IMU sensor class


#include <SPI.h>
#include "BMI270.h"

static const uint8_t SCLK_PIN = PA5;
static const uint8_t MISO_PIN = PA6;
static const uint8_t MOSI_PIN = PA7;
static const uint8_t CS_PIN   = PC2;
static const uint8_t INT_PIN  = PC3;
static const uint8_t LED_PIN  = PB9;

static SPIClass spi = SPIClass(MOSI_PIN, MISO_PIN, SCLK_PIN);
static BMI270 imu = BMI270( &spi, CS_PIN);

static volatile int interrupt_cnt = 0;
float ax, ay, az, gx, gy, gz;

static void handleInterrupt(void)
{
    imu.getMotion6NED(&ax, &ay, &az, &gx, &gy, &gz);
    interrupt_cnt++;
}

void setup(void)
{
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

  //3 second startup delay
  for(int i=10;i>0;i--) { 
    Serial.printf("starting %d ...\n",i);
    delay(300);
  }

    spi.begin();
    imu.begin(2000,16,1000); //gyro range in dps, acc range in g, data rate in hz
    Serial.printf("sensor id=0x%02X (should be 0x24)\n",imu.who_am_i());
    attachInterrupt(INT_PIN, handleInterrupt, RISING);
}

uint32_t start = 0;
void loop(void) 
{
    static uint32_t ts = micros();
    static int last_cnt = 0;

    float rate = 1000000.0 * (interrupt_cnt - last_cnt) / (micros() - ts);
    ts = micros();
    last_cnt = interrupt_cnt;
    Serial.printf("rate_hz:%.0f\tax:%.2f\tay:%.2f\taz:%.2f\tgx:%.2f\tgy:%.2f\tgz:%.2f\n", rate, ax, ay, az, gx, gy, gz);

    delay(100);
}

