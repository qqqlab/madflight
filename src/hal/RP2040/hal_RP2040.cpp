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
#ifdef ARDUINO_ARCH_RP2040

#include "../hal.h"
#include "../../cfg/cfg.h"
#include <Wire.h>                 // I2C communication
#include <SPI.h>                  // SPI communication
#include "RP2040_PWM.h"           // Servo and oneshot
#include "Serial/SerialPioIRQ.h"  // Replacement high performance PIO serial driver
#if MF_SERIAL_DMA
  #include "Serial/SerialDMA.h"     // Replacement high performance hardware DMA serial driver
#else
  #include "Serial/SerialIRQ.h"     // Replacement high performance hardware serial driver
#endif

//Bus Setup
MF_I2C    *hal_i2c[HAL_I2C_NUM] = {};
MF_Serial *hal_ser[HAL_SER_NUM] = {};
SPIClass  *hal_spi[HAL_SPI_NUM] = {};

//prototypes
void hal_eeprom_begin();
void bbx_rp2_usb_detach(); //defined in bbx/BbxGizmoSdcard_RP2.cpp
void bbx_rp2_usb_setup(); //defined in bbx/BbxGizmoSdcard_RP2.cpp

void hal_startup() {
  bbx_rp2_usb_detach();

  // remove Arduino timer state machine
  //pio_sm_set_enabled(pio2, 0, false);
  //pio_sm_unclaim(pio2, 0);

  pio_registry_name_unclaimed("Arduino");
}

void hal_usb_setup() {
  bbx_rp2_usb_setup();
}

void hal_print_resources() {
  pio_registry_print();
}

#if !MF_SERIAL_DMA
  uint8_t ser0_txbuf[256];
  uint8_t ser0_rxbuf[256];
  uint8_t ser1_txbuf[256];
  uint8_t ser1_rxbuf[256];
#endif

void hal_setup() {
  //SER BUS - is configured on demand

  //I2C BUS
  if(cfg.pin_i2c0_sda >= 0 && cfg.pin_i2c0_scl >= 0) {
    auto *i2c = &Wire; //type is TwoWire
    i2c->setSDA(cfg.pin_i2c0_sda);
    i2c->setSCL(cfg.pin_i2c0_scl);
    //i2c->setClock(1000000);
    i2c->setTimeout(25, true); //timeout, reset_with_timeout
    i2c->begin();
    hal_i2c[0] = new MF_I2CPtrWrapper<decltype(i2c)>( i2c );
    hal_i2c[0]->setClock(1000000); //set clock here so that MF_I2C wrapper knows the clock speed
  }
  if(cfg.pin_i2c1_sda >= 0 && cfg.pin_i2c1_scl >= 0) {
    auto *i2c = &Wire1; //type is TwoWire
    i2c->setSDA(cfg.pin_i2c1_sda);
    i2c->setSCL(cfg.pin_i2c1_scl);
    //i2c->setClock(1000000);
    i2c->setTimeout(25, true); //timeout, reset_with_timeout
    i2c->begin();
    hal_i2c[1] = new MF_I2CPtrWrapper<decltype(i2c)>( i2c );
    hal_i2c[1]->setClock(1000000); //set clock here so that MF_I2C wrapper knows the clock speed
  }

  //SPI BUS
  if(cfg.pin_spi0_miso >= 0 && cfg.pin_spi0_mosi >= 0 && cfg.pin_spi0_sclk >= 0) {
    hal_spi[0] = new SPIClassRP2040(spi0, cfg.pin_spi0_miso, -1, cfg.pin_spi0_sclk, cfg.pin_spi0_mosi);
    hal_spi[0]->begin();
  }
  if(cfg.pin_spi1_miso >= 0 && cfg.pin_spi1_mosi >= 0 && cfg.pin_spi1_sclk >= 0) {
    hal_spi[1] = new SPIClassRP2040(spi1, cfg.pin_spi1_miso, -1, cfg.pin_spi1_sclk, cfg.pin_spi1_mosi);
    hal_spi[1]->begin();
  }

  hal_eeprom_begin();

  //IMU
  if(cfg.pin_imu_int >= 0) {
    pinMode(cfg.pin_imu_int, INPUT); //apparently needed for RP2350, should not hurt for RP2040
  }
}

//======================================================================================================================//
//  EEPROM
//======================================================================================================================//
#include <EEPROM.h>

//#define DEBUG_EEPROM

void hal_eeprom_begin() {
  #ifdef DEBUG_EEPROM
    Serial.printf("hal_eeprom_begin()\n");
  #endif 
  EEPROM.begin(4096);
}

uint8_t hal_eeprom_read(uint32_t adr) {
  uint8_t val =EEPROM.read(adr);
  #ifdef DEBUG_EEPROM
    Serial.printf("EEr %04X:%02X\n", adr, val);
  #endif  
  return val;
}

void hal_eeprom_write(uint32_t adr, uint8_t val) {
  #ifdef DEBUG_EEPROM
    Serial.printf("EEw %04X:%02X\n", adr, val);
  #endif
  EEPROM.write(adr, val);
}

void hal_eeprom_commit() {
  #ifdef DEBUG_EEPROM
    Serial.printf("hal_eeprom_commit()\n");
  #endif 
  EEPROM.commit();
}

//======================================================================================================================//
//  MISC
//======================================================================================================================//

void hal_reboot() {
  //does not always work...
  taskENTER_CRITICAL(); //stop FreeRtos scheduler
  watchdog_enable(10, false); //uint32_t delay_ms, bool pause_on_debug
  while(1){}

  //does not always work...
  //taskENTER_CRITICAL(); //stop FreeRtos scheduler
  //watchdog_reboot(0, 0, 10); //uint32_t pc, uint32_t sp, uint32_t delay_ms
  //while(1);

  //alternate method - does not work...
  //#define AIRCR_Register (*((volatile uint32_t*)(PPB_BASE + 0x0ED0C)))
  //AIRCR_Register = 0x5FA0004;
}

uint32_t hal_get_core_num() {
  return get_core_num();
}

int hal_get_pin_number(String val) {
  return val.toInt();
}

void hal_print_pin_name(int pinnum) {
  Serial.printf("%d",pinnum);
}

MF_Serial* hal_get_ser_bus(int bus_id, int baud, MF_SerialMode mode, bool invert) {
  //exit on invalid bus_id
  if(bus_id < 0 || bus_id >= HAL_SER_NUM) return nullptr;

  //exit when exists
  if(hal_ser[bus_id]) return hal_ser[bus_id];

  uint8_t bits = 8;
  char parity;
  uint8_t stop;

  switch(mode) {
    case MF_SerialMode::mf_SERIAL_8N1:
      bits = 8; parity='N'; stop=1;
      break;
    case MF_SerialMode::mf_SERIAL_8E2: //for SBUS
      bits = 8; parity='E'; stop=2;
      break;
    default:
      Serial.printf("\nERROR: hal_get_ser_bus bus_id=%d invalid mode\n\n", bus_id);
      return nullptr;
      break;
  }

  switch(bus_id) {
    //Hardware UARTs
#if MF_SERIAL_DMA
    case 0: {
      int pin_tx = cfg.pin_ser0_tx;
      int pin_rx = cfg.pin_ser0_rx;
      if(pin_tx >= 0 || pin_rx >= 0) {
        auto *ser = new SerialDMA();
        ser->begin(0, baud, pin_tx, pin_rx, 256, 256, bits, parity, stop, invert);
        hal_ser[bus_id] = new MF_SerialPtrWrapper<decltype(ser)>( ser );
      }
      break;
    }
    case 1: {
      int pin_tx = cfg.pin_ser1_tx;
      int pin_rx = cfg.pin_ser1_rx;
      if(pin_tx >= 0 || pin_rx >= 0) {
        auto *ser = new SerialDMA();
        ser->begin(1, baud, pin_tx, pin_rx, 256, 256, bits, parity, stop, invert);
        hal_ser[bus_id] = new MF_SerialPtrWrapper<decltype(ser)>( ser );
      }
      break;
    }
#else //!MF_SERIAL_DMA
    case 0: {
      int pin_tx = cfg.pin_ser0_tx;
      int pin_rx = cfg.pin_ser0_rx;
      if(pin_tx >= 0 || pin_rx >= 0) {
        auto *ser = new SerialIRQ(uart0, pin_tx, ser0_txbuf, sizeof(ser0_txbuf), pin_rx, ser0_rxbuf, sizeof(ser0_rxbuf));
        ser->begin(baud, bits, parity, stop, invert);
        hal_ser[bus_id] = new MF_SerialPtrWrapper<decltype(ser)>( ser );
      }
      break;
    }
    case 1: {
      int pin_tx = cfg.pin_ser1_tx;
      int pin_rx = cfg.pin_ser1_rx;
      if(pin_tx >= 0 || pin_rx >= 0) {
        auto *ser = new SerialIRQ(uart1, pin_tx, ser1_txbuf, sizeof(ser1_txbuf), pin_rx, ser1_rxbuf, sizeof(ser1_rxbuf));
        ser->begin(baud, bits, parity, stop, invert);
        hal_ser[bus_id] = new MF_SerialPtrWrapper<decltype(ser)>( ser );
      }
      break;
    }
#endif //MF_SERIAL_DMA

    //PIO UARTs 
    case 2: {
      int pin_tx = cfg.pin_ser2_tx;
      int pin_rx = cfg.pin_ser2_rx;
      if(mode != MF_SerialMode::mf_SERIAL_8N1) Serial.printf("\nERROR: hal_get_ser_bus bus_id=%d invalid mode (PIO driver only supports 8N1)\n\n", bus_id);
      if(pin_tx >= 0 || pin_rx >= 0) {
        auto *ser = new SerialPioIRQ((pin_tx>=0 ? pin_tx : 0xFF), (pin_rx>=0 ? pin_rx : 0xFF), 256, 256);
        ser->begin(baud);
        if(!hal_ser[bus_id]) hal_ser[bus_id] = new MF_SerialPtrWrapper<decltype(ser)>( ser );
      }
      break;
    }
    case 3: {
      int pin_tx = cfg.pin_ser3_tx;
      int pin_rx = cfg.pin_ser3_rx;
      if(mode != MF_SerialMode::mf_SERIAL_8N1) Serial.printf("\nERROR: hal_get_ser_bus bus_id=%d invalid mode (PIO driver only supports 8N1)\n\n", bus_id);
      if(pin_tx >= 0 || pin_rx >= 0) {
        auto *ser = new SerialPioIRQ((pin_tx>=0 ? pin_tx : 0xFF), (pin_rx>=0 ? pin_rx : 0xFF), 256, 256);
        ser->begin(baud);
        if(!hal_ser[bus_id]) hal_ser[bus_id] = new MF_SerialPtrWrapper<decltype(ser)>( ser );
      }
      break;
    }
    case 4: {
      int pin_tx = cfg.pin_ser4_tx;
      int pin_rx = cfg.pin_ser4_rx;
      if(mode != MF_SerialMode::mf_SERIAL_8N1) Serial.printf("\nERROR: hal_get_ser_bus bus_id=%d invalid mode (PIO driver only supports 8N1)\n\n", bus_id);      
      if(pin_tx >= 0 || pin_rx >= 0) {
        auto *ser = new SerialPioIRQ((pin_tx>=0 ? pin_tx : 0xFF), (pin_rx>=0 ? pin_rx : 0xFF), 256, 256);
        ser->begin(baud);
        if(!hal_ser[bus_id]) hal_ser[bus_id] = new MF_SerialPtrWrapper<decltype(ser)>( ser );
      }
      break;
    }
    case 5: {
      int pin_tx = cfg.pin_ser5_tx;
      int pin_rx = cfg.pin_ser5_rx;
      if(mode != MF_SerialMode::mf_SERIAL_8N1) Serial.printf("\nERROR: hal_get_ser_bus bus_id=%d invalid mode (PIO driver only supports 8N1)\n\n", bus_id);      
      if(pin_tx >= 0 || pin_rx >= 0) {
        auto *ser = new SerialPioIRQ((pin_tx>=0 ? pin_tx : 0xFF), (pin_rx>=0 ? pin_rx : 0xFF), 256, 256);
        ser->begin(baud);
        if(!hal_ser[bus_id]) hal_ser[bus_id] = new MF_SerialPtrWrapper<decltype(ser)>( ser );
      }
      break;
    }
    default:
      return nullptr;
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

void hal_meminfo() {
  #ifdef PICO_RP2350
    const int avail = 524288; //512K
  #else
    const int avail = 262144; //256K
  #endif
  int used = avail - rp2040.getFreeHeap();
  Serial.printf("MEM: Used %.1f%% RAM (used %d of %d bytes)\n", 100.f * used / avail, used, avail);
}

void hal_print_businfo() {
  Serial.println("\n=== BUS INFO ===\n");
#if MF_SERIAL_DMA  
  Serial.printf("SER bus 0 is hardware DMA uart0 with TX:%d RX:%d\n", cfg.pin_ser0_tx, cfg.pin_ser0_rx);
  Serial.printf("SER bus 1 is hardware DMA uart1 with TX:%d RX:%d\n", cfg.pin_ser1_tx, cfg.pin_ser1_rx);
#else
  Serial.printf("SER bus 0 is hardware uart0 with TX:%d RX:%d\n", cfg.pin_ser0_tx, cfg.pin_ser0_rx);
  Serial.printf("SER bus 1 is hardware uart1 with TX:%d RX:%d\n", cfg.pin_ser1_tx, cfg.pin_ser1_rx);
#endif
  Serial.printf("SER bus 2 is pio uart with TX:%d RX:%d\n", cfg.pin_ser2_tx, cfg.pin_ser2_rx);
  Serial.printf("SER bus 3 is pio uart with TX:%d RX:%d\n", cfg.pin_ser3_tx, cfg.pin_ser3_rx);
  Serial.printf("SER bus 4 is pio uart with TX:%d RX:%d\n", cfg.pin_ser4_tx, cfg.pin_ser4_rx);
  Serial.printf("SER bus 5 is pio uart with TX:%d RX:%d\n", cfg.pin_ser5_tx, cfg.pin_ser5_rx);
  Serial.printf("I2C bus 0 is hardware i2c0 with SDA:%d SCL:%d\n", cfg.pin_i2c0_sda, cfg.pin_i2c0_scl);
  Serial.printf("I2C bus 1 is hardware i2c1 with SDA:%d SCL:%d\n", cfg.pin_i2c1_sda, cfg.pin_i2c1_scl);
  Serial.printf("SPI bus 0 is hardware spi0 with MISO:%d SCLK:%d MOSI:%d\n", cfg.pin_spi0_miso, cfg.pin_spi0_sclk, cfg.pin_spi0_mosi);
  Serial.printf("SPI bus 1 is hardware spi1 with MISO:%d SCLK:%d MOSI:%d\n", cfg.pin_spi1_miso, cfg.pin_spi1_sclk, cfg.pin_spi1_mosi);
}

BaseType_t hal_xTaskCreate( TaskFunction_t pvTaskCode,
                            const char * const pcName,
                            const uint32_t usStackDepth,
                            void * const pvParameters,
                            UBaseType_t uxPriority,
                            TaskHandle_t * const pvCreatedTask,
                            const int xCoreID) {
  if(xCoreID >= 0) {
    TaskHandle_t task_handle;
    BaseType_t rv = xTaskCreate(pvTaskCode, pcName, usStackDepth, pvParameters, 0, &task_handle); //create with idle priority (i.e. task will not run yet)
    vTaskCoreAffinitySet(task_handle, (1<<xCoreID));
    vTaskPrioritySet(task_handle, uxTaskPriorityGet(NULL)); //raise priority
    return rv;
  }else{
    return xTaskCreate(pvTaskCode, pcName, usStackDepth, pvParameters, uxPriority, pvCreatedTask);
  }
}

#endif //#ifdef ARDUINO_ARCH_RP2040
