//Arduino version string
#define HAL_ARDUINO_STR "Arduino-Pico v" ARDUINO_PICO_VERSION_STR 

#ifndef MF_MCU_NAME
  #ifdef PICO_RP2350
    #if !PICO_RP2350A
      #define MF_MCU_NAME "RP2350B (48 GPIO)"
    #else
      #define MF_MCU_NAME "RP2350A (30 GPIO)"
    #endif
  #else
    #define MF_MCU_NAME "RP2040"
  #endif
#endif

//======================================================================================================================//
//                    IMU
//======================================================================================================================//
#ifndef IMU_EXEC
  #define IMU_EXEC IMU_EXEC_FREERTOS_OTHERCORE
  //#define IMU_EXEC IMU_EXEC_FREERTOS
#endif
#ifndef IMU_FREERTOS_TASK_PRIORITY
  #define IMU_FREERTOS_TASK_PRIORITY 7
#endif

//======================================================================================================================//
//                    hal_setup()
//======================================================================================================================//

//-------------------------------------
//Include Libraries
//-------------------------------------
#include <Wire.h> //I2C communication
#include <SPI.h> //SPI communication
#include "RP2040_PWM.h"  //Servo and oneshot
#include "../MF_I2C.h"
#include "../MF_Serial.h"
#include "Serial/SerialIRQ.h"  //Replacement high performance hardware serial driver
#include "Serial/SerialPioIRQ.h"  //Replacement high performance PIO serial driver

//-------------------------------------
//Bus Setup
//-------------------------------------

#define HAL_SER_NUM 2
#define HAL_I2C_NUM 2
#define HAL_SPI_NUM 2

MF_I2C    *hal_i2c[HAL_I2C_NUM] = {};
MF_Serial *hal_ser[HAL_SER_NUM] = {};
SPIClass  *hal_spi[HAL_SPI_NUM] = {};

//prototype
void hal_eeprom_begin();

uint8_t ser0_txbuf[256];
uint8_t ser0_rxbuf[256];
uint8_t ser1_txbuf[256];
uint8_t ser1_rxbuf[256];

void hal_setup() {
  //Serial BUS on demand

  //I2C BUS
  if(cfg.pin_i2c0_sda >= 0 && cfg.pin_i2c0_scl >= 0) {
    auto *i2c = &Wire; //type is TwoWire
    i2c->setSDA(cfg.pin_i2c0_sda);
    i2c->setSCL(cfg.pin_i2c0_scl);
    i2c->setClock(1000000);
    i2c->setTimeout(25, true); //timeout, reset_with_timeout
    i2c->begin();
    hal_i2c[0] = new MF_I2CPtrWrapper<decltype(i2c)>( i2c );
  }
  if(cfg.pin_i2c1_sda >= 0 && cfg.pin_i2c1_scl >= 0) {
    auto *i2c = &Wire1; //type is TwoWire
    i2c->setSDA(cfg.pin_i2c1_sda);
    i2c->setSCL(cfg.pin_i2c1_scl);
    i2c->setClock(1000000);
    i2c->setTimeout(25, true); //timeout, reset_with_timeout
    i2c->begin();
    hal_i2c[1] = new MF_I2CPtrWrapper<decltype(i2c)>( i2c );
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

inline uint32_t hal_get_core_num() {
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
    case 0: {
      int pin_tx = cfg.pin_ser0_tx;
      int pin_rx = cfg.pin_ser0_rx;
      auto *ser = new SerialIRQ(uart0, pin_tx, ser0_txbuf, sizeof(ser0_txbuf), pin_rx, ser0_rxbuf, sizeof(ser0_rxbuf));
      if(pin_tx >= 0 || pin_rx >= 0) {
        ser->begin(baud, bits, parity, stop, invert);
        if(!hal_ser[bus_id]) hal_ser[bus_id] = new MF_SerialPtrWrapper<decltype(ser)>( ser );
      }
      break;
    }
    case 1: {
      int pin_tx = cfg.pin_ser1_tx;
      int pin_rx = cfg.pin_ser1_rx;
      auto *ser = new SerialIRQ(uart1, pin_tx, ser1_txbuf, sizeof(ser1_txbuf), pin_rx, ser1_rxbuf, sizeof(ser1_rxbuf));
      if(pin_tx >= 0 || pin_rx >= 0) {
        ser->begin(baud, bits, parity, stop, invert);
        if(!hal_ser[bus_id]) hal_ser[bus_id] = new MF_SerialPtrWrapper<decltype(ser)>( ser );
      }
      break;
    }

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
