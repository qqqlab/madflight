/*########################################################################################################################
This file contains all necessary functions and code for specific hardware platforms to avoid cluttering the main code

This file defines:
  xxx_PIN -> The pin assignments
  *rcin_Serial -> Serial port for RCIN
  *spi -> SPI port
  *i2c -> I2C port
  HW_WIRETYPE -> the class to use for I2C  
  hw_Setup() -> function to init the hardware
########################################################################################################################*/

//======================================================================================================================//
//                         HARDWARE DEFINITION for Espressif ESP32 DevKitC 38 pin board                                 //
//======================================================================================================================//
//ESP32 - Most pins can be assigned freely
//This pin layout is optimized for Espressif ESP32 DevKitC 38 pin board, use "ESP32 Dev Module" as board in Arduino IDE

#include "src/HW_ESP32/ESP32_SoftWire.h" //I2C communication - ESP32 has a bug in I2C which causes the bus to hang for 1 second after a failed read, makes I2C for this project unusable... --> https://github.com/espressif/esp-idf/issues/4999
#include <SPI.h>                         //SPI communication
#include "src/HW_ESP32/ESP32_PWM.h"      //Servo and onshot

//LED
const int led_PIN      =  2; //Note: ESP32 DevKitC has no on-board LED

//Serial Debug on tx0 (pin 1), rx0 (pin 3) connected to serial->USB converter

//RC Receiver:
const int rcin_PPM_PIN = 35; //can be same as rcin_RX_PIN
const int rcin_RX_PIN  = 35;
const int rcin_TX_PIN  = 32;
HardwareSerial *rcin_Serial = &Serial1; // can use &Serial, &Serial1, or &Serial2

//IMU:
const int imu_INT_PIN = 39; //only used when USE_IMU_INTERRUPT is defined
#define HW_RTOS_IMUTASK_PRIORITY 31 //IMU Interrupt task priority. ESP32 max priority is 31.

//I2C:
const int i2c_SDA_PIN  = 23; //default: Wire 21
const int i2c_SCL_PIN  = 22; //default: Wire 22
typedef SoftWire HW_WIRETYPE; //define the class to use for I2C
//TwoWire *i2c = &Wire; //&Wire or &Wire1 - when using <Wire.h>
HW_WIRETYPE *i2c = new HW_WIRETYPE();  //create a ESP32_SoftWire instance

//SPI:
const int spi_MOSI_PIN = 21; //default: VSPI 23, HSPI 13
const int spi_MISO_PIN = 36; //default: VSPI 19, HSPI 12
const int spi_SCLK_PIN = 19; //default: VSPI 18, HSPI 14
const int spi_CS_PIN   = 18; //default: VSPI  5, HSPI 15
SPIClass *spi = new SPIClass(VSPI); // VSPI(default) or HSPI

//Outputs:
#define hw_OUT_COUNT 13
const int8_t out_PIN[hw_OUT_COUNT] = {33,25,26,27,14,12,13,15,0,4,16,17,5}; //for ESP32 it is recommended to use only pins 2,4,12-19,21-23,25-27,32-33 for motors/servos

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

  rcin_Serial->setPins(rcin_RX_PIN, rcin_TX_PIN);

  i2c->begin(i2c_SDA_PIN, i2c_SCL_PIN, 1000000); //Note: this is 2.5 times the MPU6050/MPU9150 spec sheet 400 kHz max...
  i2c->setTimeout(1); //timeout in milliseconds, default 50 ms  

  spi->begin(spi_MOSI_PIN, spi_MISO_PIN, spi_SCLK_PIN, spi_CS_PIN);

  startLoop1Task();
}
