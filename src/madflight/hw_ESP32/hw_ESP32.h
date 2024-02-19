/*########################################################################################################################
This file contains all necessary functions and code for ESP32 hardware platforms

This file defines:
  HW_PIN_xxx -> The pin assignments
  *rcin_Serial -> Serial port for RCIN
  *spi -> SPI port
  *i2c -> I2C port
  HW_WIRETYPE -> the class to use for I2C
  hw_Setup() -> function to init the hardware
  HW_xxx and hw_xxx -> all other hardware platform specific stuff
########################################################################################################################*/

//======================================================================================================================//
//                    DEFAULT BOARD
//======================================================================================================================//
#ifndef HW_BOARD_NAME
  #ifdef CONFIG_IDF_TARGET_ESP32S3
    #include "../../madflight_board_default_ESP32-S3.h"
  #else
    #include "../../madflight_board_default_ESP32.h"
  #endif
#endif

//======================================================================================================================//
//                    hw_setup()
//======================================================================================================================//
#define HW_USE_FREERTOS //ESP32 always uses FreeRTOS
#define HW_RTOS_IMUTASK_PRIORITY 31 //IMU Interrupt task priority, higher number is higher priority. Max priority on ESP32 is 31

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

//======================================================================================================================//
//  EEPROM
//======================================================================================================================//
#include <EEPROM.h>

void hw_eeprom_begin() {
  EEPROM.begin(4096);
}

uint8_t hw_eeprom_read(uint32_t adr) {
  return EEPROM.read(adr);
}

void hw_eeprom_write(uint32_t adr, uint8_t val) {
  EEPROM.write(adr, val);
}

void hw_eeprom_commit() {
  EEPROM.commit();
}

//======================================================================================================================//
//  hw_setup()
//======================================================================================================================//

void hw_setup()
{
  Serial.println(HW_BOARD_NAME);

  rcin_Serial->setPins(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);

  gps_Serial.setPins(HW_PIN_GPS_RX, HW_PIN_GPS_TX);

  i2c->begin(HW_PIN_I2C_SDA, HW_PIN_I2C_SCL, 1000000);

  if(HW_PIN_SPI_SCLK>=0 && HW_PIN_SPI_MISO>=0 && HW_PIN_SPI_MOSI>=0) {
    spi1.begin(HW_PIN_SPI_SCLK, HW_PIN_SPI_MISO, HW_PIN_SPI_MOSI);
  }

  if(HW_PIN_SPI2_SCLK>=0 && HW_PIN_SPI2_MISO>=0 && HW_PIN_SPI2_MOSI>=0) {
    spi2.begin(HW_PIN_SPI2_SCLK, HW_PIN_SPI2_MISO, HW_PIN_SPI2_MOSI);
  }

  hw_eeprom_begin();

  startLoop1Task();
}

void hw_reboot() {	
  ESP.restart();
}
