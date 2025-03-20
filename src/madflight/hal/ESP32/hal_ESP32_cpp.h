
//Arduino version string
#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 0
#endif
#ifndef ESP_ARDUINO_VERSION_MINOR
  #define ESP_ARDUINO_VERSION_MINOR 0
#endif
#ifndef ESP_ARDUINO_VERSION_PATCH
  #define ESP_ARDUINO_VERSION_PATCH 0
#endif
#define mf_df2xstr(s)              #s
#define mf_df2str(s)               mf_df2xstr(s)
#define HAL_ARDUINO_STR "Arduino-ESP32 v" mf_df2str(ESP_ARDUINO_VERSION_MAJOR) "." mf_df2str(ESP_ARDUINO_VERSION_MINOR) "." mf_df2str(ESP_ARDUINO_VERSION_PATCH)

//======================================================================================================================//
//                    IMU
//======================================================================================================================//
#define IMU_EXEC IMU_EXEC_FREERTOS //ESP32 always uses FreeRTOS on core0 (can't used float on core1)
#define IMU_FREERTOS_TASK_PRIORITY 31 //IMU Interrupt task priority, higher number is higher priority. Max priority on ESP32 is 31

//======================================================================================================================//
//                    FREERTOS
//======================================================================================================================//
#define FREERTOS_DEFAULT_STACK_SIZE 2048 //stack size on ESP32 is in bytes, not in 32bit words

//======================================================================================================================//
//  hal_setup()
//======================================================================================================================//

/*--------------------------------------------------------------------------------------------------
  IMPORTANT
  
  ESP32 Wire has a bug in I2C which causes the bus to hang for 1 second after a failed read, which can 
  happen a couple times per minute. This makes Wire I2C for IMU not a real option... 
  See --> https://github.com/espressif/esp-idf/issues/4999

  Uncomment USE_ESP32_SOFTWIRE to use software I2C, but this does not work well with all sensors...
  
  So, until a better I2C solution is available: use an SPI IMU sensor on ESP32!!!!
----------------------------------------------------------------------------------------------------*/  
//#define USE_ESP32_SOFTWIRE //uncomment to use SoftWire instead of Wire

//-------------------------------------
//Include Libraries
//-------------------------------------
#ifdef USE_ESP32_SOFTWIRE
  #include "ESP32_SoftWire.h"
#else
  #include <Wire.h>
#endif
#include <SPI.h> //SPI communication
#include "ESP32_PWM.h" //Servo and onshot
#include "../MF_Serial.h"
#include "../MF_I2C.h"

//-------------------------------------
//Bus Setup
//-------------------------------------

#define HAL_SER_NUM 3
#define HAL_I2C_NUM 2
#define HAL_SPI_NUM 2

MF_I2C    *hal_i2c[HAL_I2C_NUM] = {};
MF_Serial *hal_ser[HAL_SER_NUM] = {};
SPIClass  *hal_spi[HAL_SPI_NUM] = {};

#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
  #define VSPI FSPI
#endif
SPIClass spi0 = SPIClass(HSPI); // VSPI or HSPI(default)
SPIClass spi1 = SPIClass(VSPI); // VSPI(default) or HSPI

//prototypes
void hal_eeprom_begin();
void startLoop1Task();

void hal_setup()
{
  //Serial BUS (&Serial, &Serial1, &Serial2) - ser0 &Serial is used for CLI via uart->USB converter
  if(cfg.pin_ser0_tx >= 0 && cfg.pin_ser0_rx >= 0) {
    auto *ser = &Serial1;
    ser->setPins(cfg.pin_ser0_rx, cfg.pin_ser0_tx);
    ser->setTxBufferSize(256);
    ser->setRxBufferSize(256);
    hal_ser[0] = new MF_SerialPtrWrapper<decltype(ser)>( ser );
  }
  if(cfg.pin_ser1_tx >= 0 && cfg.pin_ser1_rx >= 0) {
    auto *ser = &Serial2;
    ser->setPins(cfg.pin_ser1_rx, cfg.pin_ser1_tx);
    ser->setTxBufferSize(256);
    ser->setRxBufferSize(256);
    hal_ser[1] = new MF_SerialPtrWrapper<decltype(ser)>( ser );
  }

  //I2C BUS (&Wire, &Wire1)
  if(cfg.pin_i2c0_sda >= 0 && cfg.pin_i2c0_scl >= 0) {
    #ifdef USE_ESP32_SOFTWIRE
      SoftWire *i2c = new SoftWire();  //create a ESP32_SoftWire instance
    #else
      TwoWire *i2c = &Wire;
    #endif
    i2c->begin(cfg.pin_i2c0_sda, cfg.pin_i2c0_scl, 1000000);
    hal_i2c[0] = new MF_I2CPtrWrapper<decltype(i2c)>( i2c );
  }
  if(cfg.pin_i2c1_sda >= 0 && cfg.pin_i2c1_scl >= 0) {
    #ifdef USE_ESP32_SOFTWIRE
      SoftWire *i2c = new SoftWire();  //create a ESP32_SoftWire instance
    #else
      TwoWire *i2c = &Wire1; 
    #endif
    i2c->begin(cfg.pin_i2c1_sda, cfg.pin_i2c1_scl, 1000000);
    hal_i2c[1] = new MF_I2CPtrWrapper<decltype(i2c)>( i2c );
  }

  //SPI BUS
  if(cfg.pin_spi0_miso >= 0 && cfg.pin_spi0_mosi >= 0 && cfg.pin_spi0_sclk >= 0) {
    spi0.begin(cfg.pin_spi0_sclk, cfg.pin_spi0_miso, cfg.pin_spi0_mosi);
    hal_spi[0] = &spi0;
  }
  if(cfg.pin_spi1_miso >= 0 && cfg.pin_spi1_mosi >= 0 && cfg.pin_spi1_sclk >= 0) {
    spi0.begin(cfg.pin_spi1_sclk, cfg.pin_spi1_miso, cfg.pin_spi1_mosi);
    hal_spi[1] = &spi1;
  }

  hal_eeprom_begin();

  startLoop1Task();
}

//======================================================================================================================//
// RTOS task for setup1() and loop1() on second core
//======================================================================================================================//
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


//======================================================================================================================//
//  EEPROM
//======================================================================================================================//
#include <EEPROM.h>

void hal_eeprom_begin() {
  EEPROM.begin(4096);
}

uint8_t hal_eeprom_read(uint32_t adr) {
  return EEPROM.read(adr);
}

void hal_eeprom_write(uint32_t adr, uint8_t val) {
  EEPROM.write(adr, val);
}

void hal_eeprom_commit() {
  EEPROM.commit();
}


//======================================================================================================================//
//  MISC
//======================================================================================================================//

void hal_reboot() {
  ESP.restart();
}

uint32_t hal_get_core_num() {
  return xPortGetCoreID();
}

int hal_get_pin_number(String val) {
  return val.toInt();
}

void hal_print_pin_name(int pinnum) {
  Serial.printf("%d",pinnum);
}
