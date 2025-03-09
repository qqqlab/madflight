/*########################################################################################################################
This file contains all necessary functions and code for ESP32 hardware platforms

This file defines:
  HW_PIN_xxx -> The pin assignments
  *rcin_Serial -> Serial port for RCIN as MF_Serial object
  *gps_Serial -> Serial port for GPS as MF_Serial object
  *mf_i2c -> I2C port as MF_I2C object
  *spi -> SPI port
  hal_setup() -> function to init the hardware
  HAL_xxx and hal_xxx -> all other hardware platform specific stuff
########################################################################################################################*/

//Arduino-ESP32 version string
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
#define HW_ARDUINO_STR "Arduino-ESP32 v" mf_df2str(ESP_ARDUINO_VERSION_MAJOR) "." mf_df2str(ESP_ARDUINO_VERSION_MINOR) "." mf_df2str(ESP_ARDUINO_VERSION_PATCH)

//======================================================================================================================//
//                    DEFAULT PINS
//======================================================================================================================//
#ifndef HW_BOARD_NAME
  #ifdef CONFIG_IDF_TARGET_ESP32S3
    #include <madflight_board_default_ESP32-S3.h>
  #else
    #include <madflight_board_default_ESP32.h>
  #endif
#endif

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

const int HW_PIN_OUT[] = HW_PIN_OUT_LIST;

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
#include <SPI.h>                         //SPI communication
#include "ESP32_PWM.h"      //Servo and onshot
#include "../../common/MF_Serial.h"

//-------------------------------------
//Bus Setup
//-------------------------------------

#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define VSPI FSPI
#endif  
SPIClass spi1 = SPIClass(HSPI); // VSPI or HSPI(default) - used for IMU
SPIClass spi2 = SPIClass(VSPI);  // VSPI(default) or HSPI - used for BB and other functions

SPIClass *spi = &spi1;
SPIClass *bb_spi = &spi2;

//prototypes
void hal_eeprom_begin();
void startLoop1Task();

void hal_setup()
{
  Serial.println(HW_BOARD_NAME);

  //rcin_Serial - global serial port for RCIN
  #if defined(HW_PIN_RCIN_TX) && defined(HW_PIN_RCIN_RX)
    auto *rcin_ser = &Serial1; //&Serial1 or &Serial2 (&Serial is used for debugging)
    rcin_ser->setPins(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);
    rcin_ser->setTxBufferSize(256);
    rcin_ser->setRxBufferSize(256);
    rcin_Serial = new MF_SerialPtrWrapper<decltype(rcin_ser)>( rcin_ser );
  #endif

  //gps_Serial - global serial port for GPS
  #if defined(HW_PIN_GPS_TX) && defined(HW_PIN_GPS_RX)
    auto *gps_ser = &Serial2; //&Serial1 or &Serial2 (&Serial is used for debugging)
    gps_ser->setPins(HW_PIN_GPS_RX, HW_PIN_GPS_TX);
    gps_ser->setTxBufferSize(256);
    gps_ser->setRxBufferSize(256);
    rcin_Serial = new MF_SerialPtrWrapper<decltype(gps_ser)>( gps_ser );
  #endif

  //mf_i2c - global I2C interface
  #if defined(HW_PIN_I2C_SDA) && defined(HW_PIN_I2C_SCL)
    #ifdef USE_ESP32_SOFTWIRE
      SoftWire *i2c_ptr = new SoftWire();  //create a ESP32_SoftWire instance
    #else
      TwoWire *i2c_ptr = &Wire; //&Wire or &Wire1
    #endif
    i2c_ptr->begin(HW_PIN_I2C_SDA, HW_PIN_I2C_SCL, 1000000);
    mf_i2c = new MF_I2CPtrWrapper<decltype(i2c_ptr)>( i2c_ptr );
  #endif

  #if defined(HW_PIN_SPI_SCLK) && defined(HW_PIN_SPI_MISO) && defined(HW_PIN_SPI_MOSI)
    spi1.begin(HW_PIN_SPI_SCLK, HW_PIN_SPI_MISO, HW_PIN_SPI_MOSI);
  #endif

  #if defined(HW_PIN_SPI2_SCLK) && defined(HW_PIN_SPI2_MISO) && defined(HW_PIN_SPI2_MOSI)
    spi2.begin(HW_PIN_SPI2_SCLK, HW_PIN_SPI2_MISO, HW_PIN_SPI2_MOSI);
  #endif

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
