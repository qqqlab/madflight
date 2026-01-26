/*==========================================================================================
MIT License

Copyright (c) 2023-2026 https://madflight.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
===========================================================================================*/

#ifdef ARDUINO_ARCH_ESP32

#include "../hal.h"
#include "../../cfg/cfg.h"
#include <SPI.h> //SPI communication
#include "ESP32_PWM_cpp.h" //Servo and oneshot
#ifdef USE_ESP32_SOFTWIRE
  #include "ESP32_SoftWire.h"
#else
  #include <Wire.h>
#endif

//Bus Setup
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

void hal_startup() {} // USB MSC not implemented for ESP32
void hal_usb_setup() {}
void hal_print_resources() {}

void hal_setup()
{
  //print bus config
  Serial.printf("HAL: SER bus 0 is hardware Serial1 with TX:%d RX:%d\n", (int)cfg.pin_ser0_tx, (int)cfg.pin_ser0_rx);
  Serial.printf("HAL: SER bus 1 is hardware Serial2 with TX:%d RX:%d\n", (int)cfg.pin_ser1_tx, (int)cfg.pin_ser1_rx);
  Serial.printf("HAL: I2C bus 0 is hardware i2c0 with SDA:%d SCL:%d\n", (int)cfg.pin_i2c0_sda, (int)cfg.pin_i2c0_scl);
  Serial.printf("HAL: I2C bus 1 is hardware i2c1 with SDA:%d SCL:%d\n", (int)cfg.pin_i2c1_sda, (int)cfg.pin_i2c1_scl);
  Serial.printf("HAL: SPI bus 0 is hardware spi0 with MISO:%d SCLK:%d MOSI:%d\n", (int)cfg.pin_spi0_miso, (int)cfg.pin_spi0_sclk, (int)cfg.pin_spi0_mosi);
  Serial.printf("HAL: SPI bus 1 is hardware spi1 with MISO:%d SCLK:%d MOSI:%d\n", (int)cfg.pin_spi1_miso, (int)cfg.pin_spi1_sclk, (int)cfg.pin_spi1_mosi);

  //Serial BUS uses late binding (i.e. gets created when used)

  //I2C BUS (&Wire, &Wire1)
  if(cfg.pin_i2c0_sda >= 0 && cfg.pin_i2c0_scl >= 0) {
    #ifdef USE_ESP32_SOFTWIRE
      SoftWire *i2c = new SoftWire();  //create a ESP32_SoftWire instance
    #else
      TwoWire *i2c = &Wire;
    #endif
    i2c->begin(cfg.pin_i2c0_sda, cfg.pin_i2c0_scl);
    hal_i2c[0] = new MF_I2CPtrWrapper<decltype(i2c)>( i2c );
    hal_i2c[0]->setClock(1000000); //set clock here so that MF_I2C wrapper knows the clock speed
  }
  if(cfg.pin_i2c1_sda >= 0 && cfg.pin_i2c1_scl >= 0) {
    #ifdef USE_ESP32_SOFTWIRE
      SoftWire *i2c = new SoftWire();  //create a ESP32_SoftWire instance
    #else
      TwoWire *i2c = &Wire1; 
    #endif
    i2c->begin(cfg.pin_i2c1_sda, cfg.pin_i2c1_scl);
    hal_i2c[1] = new MF_I2CPtrWrapper<decltype(i2c)>( i2c );
    hal_i2c[1]->setClock(1000000); //set clock here so that MF_I2C wrapper knows the clock speed
  }

  //SPI BUS
  if(cfg.pin_spi0_miso >= 0 && cfg.pin_spi0_mosi >= 0 && cfg.pin_spi0_sclk >= 0) {
    spi0.begin(cfg.pin_spi0_sclk, cfg.pin_spi0_miso, cfg.pin_spi0_mosi);
    hal_spi[0] = &spi0;
  }
  if(cfg.pin_spi1_miso >= 0 && cfg.pin_spi1_mosi >= 0 && cfg.pin_spi1_sclk >= 0) {
    spi1.begin(cfg.pin_spi1_sclk, cfg.pin_spi1_miso, cfg.pin_spi1_mosi);
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


//create/get Serial bus (late binding)
//Serial BUS (&Serial, &Serial1, &Serial2) - ser0 &Serial is used for CLI via uart->USB converter

/*

  https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/HardwareSerial.h

  // When pins are changed, it will detach the previous ones
  // if pin is negative, it won't be set/changed and will be kept as is
  // timeout_ms is used in baudrate detection (ESP32, ESP32S2 only)
  // invert will invert RX/TX polarity
  // rxfifo_full_thrhd if the UART Flow Control Threshold in the UART FIFO (max 127)
  void begin(
    unsigned long baud, uint32_t config = SERIAL_8N1, int8_t rxPin = -1, int8_t txPin = -1, bool invert = false, unsigned long timeout_ms = 20000UL,
    uint8_t rxfifo_full_thrhd = 120
  );
*/

MF_Serial* hal_get_ser_bus(int bus_id, int baud, MF_SerialMode mode, bool invert) {
  if(bus_id < 0 || bus_id >= HAL_SER_NUM) return nullptr;

  uint32_t config;

  switch(mode) {
    case MF_SerialMode::mf_SERIAL_8N1:
      config = SERIAL_8N1;
      break;
    case MF_SerialMode::mf_SERIAL_8E2:
      config = SERIAL_8E2;
      break;
    default:
      Serial.printf("\nERROR: hal_get_ser_bus bus_id=%d invalid mode\n\n", bus_id);
      return nullptr;
      break;
  }

  int pin_tx = -1;
  int pin_rx = -1;
  HardwareSerial *ser;
  switch(bus_id) {
    case 0: {
      pin_tx = cfg.pin_ser0_tx;
      pin_rx = cfg.pin_ser0_rx;
      ser = &Serial1;
      break;
    }
    case 1: {
      pin_tx = cfg.pin_ser1_tx;
      pin_rx = cfg.pin_ser1_rx;
      ser = &Serial2;
      break;
    }
    default:
      return nullptr;
  }

  //exit if no pins defined
  if(pin_tx < 0 && pin_rx < 0) return nullptr;

  //create new MF_SerialPtrWrapper
  if(!hal_ser[bus_id]) {
    hal_ser[bus_id] = new MF_SerialPtrWrapper<decltype(ser)>( ser );
  }

  //get ser from MF_SerialPtrWrapper, and (re)configure it
  //not needed, ser already pointing to Serial1 or Serial2 ... ser = ((MF_SerialPtrWrapper<HardwareSerial*>*)hal_ser[bus_id])->_serial;
  ser->end();
  ser->setTxBufferSize(256);
  ser->setRxBufferSize(256);
  ser->begin(baud, config, pin_rx, pin_tx, invert);

  return hal_ser[bus_id];
}

MF_I2C* hal_get_i2c_bus(int bus_id) { 
  if(bus_id < 0 || bus_id >= HAL_I2C_NUM) return nullptr;
  MF_I2C *i2c_bus = hal_i2c[bus_id];
  if(!i2c_bus) return nullptr;
  return i2c_bus;
}

SPIClass* hal_get_spi_bus(int bus_id) {
  if(bus_id < 0 || bus_id >= HAL_SPI_NUM) return nullptr;
  SPIClass *spi_bus = hal_spi[bus_id];
  if(!spi_bus) return nullptr;
  return spi_bus;
}

#endif //#ifdef ARDUINO_ARCH_ESP32
