/*########################################################################################################################
This file contains all necessary functions and code for specific hardware platforms to avoid cluttering the main code

This file defines:
  HW_PIN_xxx -> The pin assignments
  *rcin_Serial -> Serial port for RCIN
  *spi -> SPI port
  *i2c -> I2C port
  HW_WIRETYPE -> the class to use for I2C
  hw_Setup() -> function to init the hardware
  HW_xxx and hw_xxx -> all other hardware platform specifi stuff
########################################################################################################################*/

//======================================================================================================================//
//                         HARDWARE DEFINITION for Espressif ESP32 DevKitC 38 pin board                                 //
//======================================================================================================================//
//ESP32 - Most pins can be assigned freely
//This pin layout is optimized for Espressif ESP32 DevKitC 38 pin board, use "ESP32 Dev Module" as board in Arduino IDE

#include <SPI.h>                         //SPI communication
#include "src/HW_ESP32/ESP32_PWM.h"      //Servo and onshot

//NOTE: DON'T USE SAME PIN TWICE. All pins here get configured, even if they are not used. Set pin to -1 to disable.

//LED
const int HW_PIN_LED      =  2; //Note: ESP32 DevKitC has no on-board LED

//Serial Debug on tx0 (pin 1), rx0 (pin 3) connected to serial->USB converter

//RC Receiver:
const int HW_PIN_RCIN_RX  = 35; //also used as PPM input
const int HW_PIN_RCIN_TX  = 32;
HardwareSerial *rcin_Serial = &Serial1; //&Serial1 or &Serial2 (&Serial is used for debugging)

//IMU:
const int HW_PIN_IMU_INT = 39; //VN only used when USE_IMU_INTERRUPT is defined
#define HW_RTOS_IMUTASK_PRIORITY 31 //IMU Interrupt task priority, higher number is higher priority. Max priority on ESP32 is 31

//I2C:
const int HW_PIN_I2C_SDA  = 23; //default: Wire 21
const int HW_PIN_I2C_SCL  = 22; //default: Wire 22

/*--------------------------------------------------------------------------------------------------
  IMPORTANT
  
  ESP32 Wire has a bug in I2C which causes the bus to hang for 1 second after a failed read, which can 
  happen a couple times per minute. This makes Wire I2C for IMU not a real option... 
  See --> https://github.com/espressif/esp-idf/issues/4999

  Uncomment USE_ESP32_SOFTWIRE to use software I2C, but this does not work well with all sensors...
  
  So, until a better I2C solution is available: use an SPI IMU sensor on ESP32!!!!
----------------------------------------------------------------------------------------------------*/  
//#define USE_ESP32_SOFTWIRE

#ifdef USE_ESP32_SOFTWIRE
  #include "src/HW_ESP32/ESP32_SoftWire.h"
  typedef SoftWire HW_WIRETYPE; //typedef to force IMU to use SoftWire
  typedef SoftWire TwoWire; //typedef to force BARO to use SoftWire
  HW_WIRETYPE *i2c = new HW_WIRETYPE();  //create a ESP32_SoftWire instance
#else
  #include <Wire.h>
  typedef TwoWire HW_WIRETYPE; //typedef HW_WIRETYPE with the class to use for I2C
  HW_WIRETYPE *i2c = &Wire; //&Wire or &Wire1
#endif

//SPI:
const int HW_PIN_SPI_MOSI = 21; //   defaults: VSPI 23, HSPI 13
const int HW_PIN_SPI_MISO = 36; //VP defaults: VSPI 19, HSPI 12
const int HW_PIN_SPI_SCLK = 19; //   defaults: VSPI 18, HSPI 14
const int HW_PIN_SPI_CS   = 18; //   defaults: VSPI  5, HSPI 15
SPIClass *spi = new SPIClass(HSPI); // VSPI or HSPI(default)

//Outputs:
#define HW_OUT_COUNT 13
const int8_t HW_PIN_OUT[HW_OUT_COUNT] = {33,25,26,27,14,12,13,15,0,4,16,17,5}; //for ESP32 it is recommended to use only pins 2,4,12-19,21-23,25-27,32-33 for motors/servos

//--------------------------------------------------------------------
// RTOS task for setup1() and loop1() on second core
//--------------------------------------------------------------------
TaskHandle_t loop1TaskHandle = NULL;
extern void setup1() __attribute__((weak));
extern void loop1() __attribute__((weak));

void yieldIfNecessary(void){
    static uint32_t lastYield = 0;
    uint32_t now = millis();
    if((now - lastYield) > 2000) {
        lastYield = now;
        vTaskDelay(1); //delay 1 RTOS tick to let IDLE task reset the task watchdog
    }
}

void loop1Task(void *pvParameters) {
  if(setup1) setup1();
  for(;;) {
    yieldIfNecessary();
    if (loop1) loop1();
  }
}

void startLoop1Task() {
#ifndef CONFIG_FREERTOS_UNICORE
  //start setup1() and loop1() on second core
  if(setup1 || loop1) {
    Serial.println("Starting loop1Task");
    xTaskCreateUniversal(loop1Task, "loop1Task", getArduinoLoopTaskStackSize(), NULL, 1, &loop1TaskHandle, ((ARDUINO_RUNNING_CORE)+1)%2);
  }
#endif
}

//--------------------------------------------------------------------
// hw_setup()
//--------------------------------------------------------------------
void hw_setup()
{
  delay(1000);
  Serial.println("USE_HW_ESP32");

  rcin_Serial->setPins(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);

  i2c->begin(HW_PIN_I2C_SDA, HW_PIN_I2C_SCL, 1000000); //Note: this is 2.5 times the MPU6050/MPU9150 spec sheet 400 kHz max... 

  spi->begin(HW_PIN_SPI_SCLK, HW_PIN_SPI_MISO, HW_PIN_SPI_MOSI, HW_PIN_SPI_CS);

  startLoop1Task();
}
