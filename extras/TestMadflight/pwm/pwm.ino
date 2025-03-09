//pwm test program, used for madflight v1.2.0

//LED:
#define HW_PIN_LED       -1
#define HW_LED_ON         0 //0:low is on, 1:high is on

//IMU SPI:
#define HW_PIN_SPI_MISO  -1
#define HW_PIN_SPI_MOSI  -1
#define HW_PIN_SPI_SCLK  -1
#define HW_PIN_IMU_CS    -1
#define HW_PIN_IMU_EXTI  -1 //external interrupt pin

//I2C for BARO, MAG, BAT sensors and for IMU if not using SPI
#define HW_PIN_I2C_SDA   -1
#define HW_PIN_I2C_SCL   -1

//Motor/Servo Outputs:
#define HW_OUT_COUNT     6 //number of outputs
//#define HW_PIN_OUT_LIST  {13,14,21,47,48,45} //list of output pins
#define HW_PIN_OUT_LIST  {12,14,27,26,25,33} //ESP32 devkit list of output pins
//#define HW_PIN_OUT_LIST  {0,1,2,3,4,5,6} //RPi Pico list of output pins

//Serial debug on USB Serial port (no GPIO pins)

//RC Receiver:
#define HW_PIN_RCIN_RX    -1
#define HW_PIN_RCIN_TX    -1
#define HW_PIN_RCIN_INVERTER -1 //only used for STM32 targets

//GPS:
#define HW_PIN_GPS_RX     -1 
#define HW_PIN_GPS_TX     -1
#define HW_PIN_GPS_INVERTER -1 //only used for STM32 targets

//Battery ADC
#define HW_PIN_BAT_V      -1
#define HW_PIN_BAT_I      -1

//BlackBox SPI:
#define HW_PIN_SPI2_MISO  -1
#define HW_PIN_SPI2_MOSI  -1
#define HW_PIN_SPI2_SCLK  -1
#define HW_PIN_BB_CS      -1
//*/


#define MF_TEST MF_TEST_BASE
#include <madflight.h>

void setup() {
  Serial.begin(115100);

  hal_setup(); //hardware specific setup for spi and Wire (see hw_xxx.h)

  for(int i=0;i<HW_OUT_COUNT;i++) {
    //out[i].begin(HW_PIN_OUT[i], 50, 1000, 2000); //Standard servo at 50Hz
    //out[i].begin(HW_PIN_OUT[i], 400, 950, 2000); //Standard PWM: 400Hz, 950-2000 us
    out[i].begin(HW_PIN_OUT[i], 2000, 125, 250); //Oneshot125: 2000Hz, 125-250 us

    out[i].writeFactor(0.5); //start the PWM output to the servos
  } 
}

int i=0;
void loop() {
  Serial.println(i++);
  delay(1000);
}
