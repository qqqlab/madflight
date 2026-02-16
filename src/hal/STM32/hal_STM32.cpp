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

#ifdef ARDUINO_ARCH_STM32

//Note: this implementation does not check if a PWM frequency overwrites the frequency of previously instantiated PWM instance

//Arduino IDE settings:
//Board: Generic STM32xxx
//Board part number: select according your board
//C Runtime Library: "Newlib Nano + Float Printf"
//USB support: "CDC (general Serial supperseed U(S)ART)"
//U(S)ART) support: "Enabled (generic 'Serial')"

//Programming STM32 targets:
//USB cable: upload method "STM32CubeProgrammer (DFU)" --> press boot button, connect usb cable (or press/release reset) 
//ST-LINK dongle: upload method "STM32CubeProgrammer (SWD)" --> press boot, press/release reset button (or power board)

//Arduino F411 defines: -DSTM32F4xx -DARDUINO=10607 -DARDUINO_GENERIC_F411CEUX -DARDUINO_ARCH_STM32 -DBOARD_NAME="GENERIC_F411CEUX" -DVARIANT_H="variant_generic.h" -DSTM32F411xE -DUSBCON -DUSBD_VID=0 -DUSBD_PID=0 -DHAL_PCD_MODULE_ENABLED -DUSBD_USE_CDC -DHAL_UART_MODULE_ENABLED

#include "../hal.h"
#include "../../cfg/cfg.h"
#include <Wire.h> //I2C communication
#include <SPI.h> //SPI communication
#include "STM32_PWM.h" //Servo and oneshot

//Bus Setup
MF_I2C    *hal_i2c[HAL_I2C_NUM] = {};
MF_Serial *hal_ser[HAL_SER_NUM] = {};
SPIClass  *hal_spi[HAL_SPI_NUM] = {};

//prototype
void hal_eeprom_begin();

void hal_startup() {} // USB MSC not implemented for STM32
void hal_usb_setup() {}
void hal_print_resources() {}

void hal_setup() 
{ 
  //Serial BUS late binding

  //NOTE: default serial buffer size is 64, and is defined in HardwareSerial.h
  //SERIAL_RX_BUFFER_SIZE and SERIAL_TX_BUFFER_SIZE
  //can't set that here :-(
  //need to use compiler -D arguments or modify HardwareSerial.h ...
  #if SERIAL_RX_BUFFER_SIZE<256 || SERIAL_TX_BUFFER_SIZE<256
    #warning "RCL/GPS might need larger buffers. Set SERIAL_RX_BUFFER_SIZE 256 and SERIAL_TX_BUFFER_SIZE 256 in HardwareSerial.h"
  #endif

  //I2C BUS
  if(cfg.pin_i2c0_sda >= 0 && cfg.pin_i2c0_scl >= 0) {
    auto *i2c = &Wire;
    i2c->setSDA(cfg.pin_i2c0_sda);
    i2c->setSCL(cfg.pin_i2c0_scl);
    //i2c->setClock(1000000);
    i2c->begin();
    hal_i2c[0] = new MF_I2CPtrWrapper<decltype(i2c)>( i2c );
    hal_i2c[0]->setClock(1000000); //set clock here so that MF_I2C wrapper knows the clock speed
  }
  if(cfg.pin_i2c1_sda >= 0 && cfg.pin_i2c1_scl >= 0) {
    TwoWire *i2c = new TwoWire(cfg.pin_i2c1_sda, cfg.pin_i2c1_scl);
    //i2c->setClock(1000000);
    i2c->begin();
    hal_i2c[1] = new MF_I2CPtrWrapper<decltype(i2c)>( i2c );
    hal_i2c[1]->setClock(1000000); //set clock here so that MF_I2C wrapper knows the clock speed 
  }

  //SPI
  if(cfg.pin_spi0_miso >= 0 && cfg.pin_spi0_mosi >= 0 && cfg.pin_spi0_sclk >= 0) {
    SPIClass *spi = &SPI;
    spi->setMISO(cfg.pin_spi0_miso);
    spi->setSCLK(cfg.pin_spi0_sclk);
    spi->setMOSI(cfg.pin_spi0_mosi);
    //spi->setSSEL(PIN_IMU_CS); //don't set CS here, it is done in the driver to be compatible with other hardware platforms
    spi->begin();
    hal_spi[0] = spi;
  }
  if(cfg.pin_spi1_miso >= 0 && cfg.pin_spi1_mosi >= 0 && cfg.pin_spi1_sclk >= 0) {
    SPIClass *spi = new SPIClass(cfg.pin_spi1_mosi, cfg.pin_spi1_miso, cfg.pin_spi1_sclk);
    spi->begin();
    hal_spi[1] = spi;
  }
  
  hal_eeprom_begin();
}

//======================================================================================================================//
//  EEPROM
//======================================================================================================================//
#if defined(DATA_EEPROM_BASE)
  //----------------------------------------------------------------------------------------------------------
  //unbuffered write - very slow because writes whole flash page for each byte, i.e. 1 second per changed byte
  //----------------------------------------------------------------------------------------------------------
  #include <EEPROM.h>

  void hal_eeprom_begin() {
    Serial.println("EEPROM: using Unbuffered IO");
    //EEPROM.begin(); //STM does not use size in begin() call
  }

  uint8_t hal_eeprom_read(uint32_t adr) {
    uint8_t val = EEPROM.read(adr);
    //Serial.printf("EEPROM.read(%d) = 0x%02X\n", adr, val);
    return val;
  }

  void hal_eeprom_write(uint32_t adr, uint8_t val) {
    EEPROM.update(adr, val); //update only writes when changed
    //Serial.printf("EEPROM.write(%d, 0x%02X)\n", adr, val);
  }

  void hal_eeprom_commit() {
    //EEPROM.commit();  //STM does not use commit(), write() also executes commit()
  }
#else
  //----------------------------------------------------------------------------------------------------------
  //buffered write - takes approx. 1 second to write full config
  //----------------------------------------------------------------------------------------------------------
  #include <EEPROM.h>

  void hal_eeprom_begin() {
    (void)(EEPROM); //keep compiler happy
    Serial.println("EEPROM: using Buffered IO");
    //Serial.println("START reading from flash");Serial.flush();
    eeprom_buffer_fill(); //Copy the data from the flash to the buffer
    //Serial.println("DONE reading");Serial.flush();
  }

  uint8_t hal_eeprom_read(uint32_t adr) {
    uint8_t val = eeprom_buffered_read_byte(adr); //read from buffer
    //Serial.printf("hal_eeprom_read(%d) = 0x%02X\n", adr, val);Serial.flush();
    return val;
  }

  void hal_eeprom_write(uint32_t adr, uint8_t val) {
    //Serial.printf("hal_eeprom_write(%d, 0x%02X)\n", adr, val);Serial.flush();
    eeprom_buffered_write_byte(adr, val); //write to buffer
  }

  void hal_eeprom_commit() {
    //Serial.println("START writing to flash");Serial.flush();
    eeprom_buffer_flush(); //Copy the data from the buffer to the flash
    eeprom_buffer_flush(); //TODO: calling flush twice seems to do the trick???
    //Serial.println("DONE writing");Serial.flush();
  } 
#endif

//======================================================================================================================//
//  MISC
//======================================================================================================================//
void hal_reboot() {
  NVIC_SystemReset();
}
/*
void hal_disable_irq() {
  __disable_irq();
}

void hal_enable_irq() {
  __enable_irq();
}
*/

uint32_t hal_get_core_num() {
  return 0;
}

int hal_get_pin_number(String s) {
  //integers
  if(s.toInt() != 0) return s.toInt();
  if(s == "0") return 0;
  //pin names like PB3, PC13
  if(s.length() < 3) return -1;
  if(s[0] != 'P') return -1;
  if(s[1]<'A' || s[1] > 'H') return -1;
  int port = s[1] - 'A';
  int pin = s.substring(2).toInt();
  if(pin<0 || pin>15) return -1;
  return port * 16 + pin;
}

void hal_print_pin_name(int pinnum) {
  if(pinnum<0) {
    Serial.printf("%d",pinnum);
  }else{
    Serial.printf("P%c%d", pinnum / 16 + 'A', pinnum % 16);
  }
}

#if MF_HACK_STM32_INTERRUPT_PRIORITY
IRQn_Type hal_get_irqn_from_pin(int pin) {
  // converts an arduino pin number to a STM32 HAL interrupt id
  // taken from https://github.com/stm32duino/Arduino_Core_STM32/blob/main/libraries/SrcWrapper/src/stm32/interrupt.cpp#L158
  PinName pinName = digitalPinToPinName(pin);
  uint16_t gpioPin = STM_GPIO_PIN(pinName);
  uint8_t id = 0;

  while (gpioPin != 0x0001) {
    gpioPin = gpioPin >> 1;
    id++;
  }

  switch (id) {
    case 0: return EXTI0_IRQn;
    case 1: return EXTI1_IRQn;
    case 2: return EXTI2_IRQn;
    case 3: return EXTI3_IRQn;
    case 4: return EXTI4_IRQn;
    case 5: case 6: case 7: case 8: case 9:
      return EXTI9_5_IRQn;
    default:
      return EXTI15_10_IRQn;
  }
}
#endif //MF_HACK_STM32_INTERRUPT_PRIORITY

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
  int pin_inv = -1;
  switch(bus_id) {
    case 0: {
      pin_tx = cfg.pin_ser0_tx;
      pin_rx = cfg.pin_ser0_rx;
      pin_inv = cfg.pin_ser0_inv;
      break;
    }
    case 1: {
      pin_tx = cfg.pin_ser1_tx;
      pin_rx = cfg.pin_ser1_rx;
      pin_inv = cfg.pin_ser1_inv;
      break;
    }
    case 2: {
      pin_tx = cfg.pin_ser2_tx;
      pin_rx = cfg.pin_ser2_rx;
      pin_inv = cfg.pin_ser2_inv;
      break;
    }
    case 3: {
      pin_tx = cfg.pin_ser3_tx;
      pin_rx = cfg.pin_ser3_rx;
      pin_inv = cfg.pin_ser3_inv;
      break;
    }
    case 4: {
      pin_tx = cfg.pin_ser4_tx;
      pin_rx = cfg.pin_ser4_rx;
      pin_inv = cfg.pin_ser4_inv;
      break;
    }
    case 5: {
      pin_tx = cfg.pin_ser5_tx;
      pin_rx = cfg.pin_ser5_rx;
      pin_inv = cfg.pin_ser5_inv;
      break;
    }
    case 6: {
      pin_tx = cfg.pin_ser6_tx;
      pin_rx = cfg.pin_ser6_rx;
      pin_inv = cfg.pin_ser6_inv;
      break;
    }
    case 7: {
      pin_tx = cfg.pin_ser7_tx;
      pin_rx = cfg.pin_ser7_rx;
      pin_inv = cfg.pin_ser7_inv;
      break;
    }
    default:
      return nullptr;
  }

  //exit if no pins defined
  if(pin_tx < 0 && pin_rx < 0) return nullptr;

  //create new MF_SerialPtrWrapper
  if(!hal_ser[bus_id]) {
    HardwareSerial *ser = new HardwareSerial(pin_rx, pin_tx);
    hal_ser[bus_id] = new MF_SerialPtrWrapper<decltype(ser)>( ser );
  }

  //get ser from MF_SerialPtrWrapper, and configure it
  HardwareSerial *ser = ((MF_SerialPtrWrapper<HardwareSerial*>*)hal_ser[bus_id])->_serial;
  ser->begin(baud, config);
  if(pin_inv >= 0) {
    pinMode(pin_inv, OUTPUT);
    digitalWrite(pin_inv, (invert ? HIGH : LOW) );
  }
  if(invert && pin_inv < 0) {
    Serial.printf("\nERROR: Serial bus %d requested inverted, but does not have an inverter pin\n\n");
  }

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

void hal_meminfo() {}

void hal_print_businfo() {
  Serial.println("\n=== BUS INFO ===\n");
  Serial.printf("Notice: availability of hardware busses depends on chip used, and is not checked here.\n", cfg.pin_ser0_tx, cfg.pin_ser0_rx);
  Serial.printf("SER bus 0 is hardware uart with TX:%d RX:%d\n", cfg.pin_ser0_tx, cfg.pin_ser0_rx);
  Serial.printf("SER bus 1 is hardware uart with TX:%d RX:%d\n", cfg.pin_ser1_tx, cfg.pin_ser1_rx);
  Serial.printf("SER bus 2 is hardware uart with TX:%d RX:%d\n", cfg.pin_ser2_tx, cfg.pin_ser2_rx);
  Serial.printf("SER bus 3 is hardware uart with TX:%d RX:%d\n", cfg.pin_ser3_tx, cfg.pin_ser3_rx);
  Serial.printf("SER bus 4 is hardware uart with TX:%d RX:%d\n", cfg.pin_ser4_tx, cfg.pin_ser4_rx);
  Serial.printf("SER bus 5 is hardware uart with TX:%d RX:%d\n", cfg.pin_ser5_tx, cfg.pin_ser5_rx);
  Serial.printf("SER bus 6 is hardware uart with TX:%d RX:%d\n", cfg.pin_ser6_tx, cfg.pin_ser6_rx);
  Serial.printf("SER bus 7 is hardware uart with TX:%d RX:%d\n", cfg.pin_ser7_tx, cfg.pin_ser7_rx);
  Serial.printf("I2C bus 0 is hardware i2c with SDA:%d SCL:%d\n", cfg.pin_i2c0_sda, cfg.pin_i2c0_scl);
  Serial.printf("I2C bus 1 is hardware i2c with SDA:%d SCL:%d\n", cfg.pin_i2c1_sda, cfg.pin_i2c1_scl);
  Serial.printf("SPI bus 0 is hardware spi with MISO:%d SCLK:%d MOSI:%d\n", cfg.pin_spi0_miso, cfg.pin_spi0_sclk, cfg.pin_spi0_mosi);
  Serial.printf("SPI bus 1 is hardware spi with MISO:%d SCLK:%d MOSI:%d\n", cfg.pin_spi1_miso, cfg.pin_spi1_sclk, cfg.pin_spi1_mosi);

}

BaseType_t hal_xTaskCreate( TaskFunction_t pvTaskCode,
                            const char * const pcName,
                            const uint32_t usStackDepth,
                            void * const pvParameters,
                            UBaseType_t uxPriority,
                            TaskHandle_t * const pvCreatedTask,
                            const int xCoreID) {
  return xTaskCreate(pvTaskCode, pcName, usStackDepth, pvParameters, uxPriority, pvCreatedTask);
}

#endif //#ifdef ARDUINO_ARCH_STM32
