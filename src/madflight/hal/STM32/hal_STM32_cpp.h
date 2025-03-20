#include "variant_generic.h"
#include "pins_arduino.h"

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

#define df2xstr(s)              #s
#define df2str(s)               df2xstr(s)
#define HAL_ARDUINO_STR "Arduino_Core_STM32 v" df2str(STM32_CORE_VERSION_MAJOR) "." df2str(STM32_CORE_VERSION_MINOR) "." df2str(STM32_CORE_VERSION_PATCH)


#ifndef HAL_MCU
  #ifdef BOARD_NAME
    #define HAL_MCU BOARD_NAME
  #endif
#endif


//======================================================================================================================//
//                    IMU
//======================================================================================================================//
#ifndef IMU_EXEC
  #define IMU_EXEC IMU_EXEC_IRQ //Use IMU as interrupt by default
#endif

//======================================================================================================================//
//                    FREERTOS
//======================================================================================================================//
#include <STM32FreeRTOS.h>
#define FREERTOS_DEFAULT_STACK_SIZE 512 //stack size in 32bit words

//======================================================================================================================//
//                    hal_setup()
//======================================================================================================================//

//-------------------------------------
//Include Libraries
//-------------------------------------
#include <Wire.h> //I2C communication
#include <SPI.h> //SPI communication
#include "STM32_PWM.h" //Servo and oneshot
#include "../../hal/MF_Serial.h"

//-------------------------------------
//Bus Setup
//-------------------------------------

#define HAL_SER_NUM 8
#define HAL_I2C_NUM 2
#define HAL_SPI_NUM 2

MF_I2C    *hal_i2c[HAL_I2C_NUM] = {};
MF_Serial *hal_ser[HAL_SER_NUM] = {};
SPIClass  *hal_spi[HAL_SPI_NUM] = {};

//prototype
void hal_eeprom_begin();

void hal_setup() 
{ 
  //Serial BUS

  //NOTE: default serial buffer size is 64, and is defined in HardwareSerial.h
  //SERIAL_RX_BUFFER_SIZE and SERIAL_TX_BUFFER_SIZE
  //can't set that here :-(
  //need to use compiler -D arguments or modify HardwareSerial.h ...
  #if SERIAL_RX_BUFFER_SIZE<256 || SERIAL_TX_BUFFER_SIZE<256
    #warning "RCL/GPS might need larger buffers. Set SERIAL_RX_BUFFER_SIZE 256 and SERIAL_TX_BUFFER_SIZE 256 in HardwareSerial.h"
  #endif

  if(cfg.pin_ser0_tx >= 0 && cfg.pin_ser0_rx >= 0) {
    auto *ser =new HardwareSerial(cfg.pin_ser0_rx, cfg.pin_ser0_tx);
    hal_ser[0] = new MF_SerialPtrWrapper<decltype(ser)>( ser );
    if(cfg.pin_ser0_inv >= 0) {
      pinMode(cfg.pin_ser0_inv, OUTPUT);
      digitalWrite(cfg.pin_ser0_inv, LOW); //not inverted
    }
  }
  if(cfg.pin_ser1_tx >= 0 && cfg.pin_ser1_rx >= 0) {
    auto *ser =new HardwareSerial(cfg.pin_ser1_rx, cfg.pin_ser1_tx);
    hal_ser[1] = new MF_SerialPtrWrapper<decltype(ser)>( ser );
    if(cfg.pin_ser1_inv >= 0) {
      pinMode(cfg.pin_ser1_inv, OUTPUT);
      digitalWrite(cfg.pin_ser1_inv, LOW); //not inverted
    }
  }
  if(cfg.pin_ser2_tx >= 0 && cfg.pin_ser2_rx >= 0) {
    auto *ser =new HardwareSerial(cfg.pin_ser2_rx, cfg.pin_ser2_tx);
    hal_ser[2] = new MF_SerialPtrWrapper<decltype(ser)>( ser );
    if(cfg.pin_ser2_inv >= 0) {
      pinMode(cfg.pin_ser2_inv, OUTPUT);
      digitalWrite(cfg.pin_ser2_inv, LOW); //not inverted
    }
  }
  if(cfg.pin_ser3_tx >= 0 && cfg.pin_ser3_rx >= 0) {
    auto *ser =new HardwareSerial(cfg.pin_ser3_rx, cfg.pin_ser3_tx);
    hal_ser[3] = new MF_SerialPtrWrapper<decltype(ser)>( ser );
    if(cfg.pin_ser3_inv >= 0) {
      pinMode(cfg.pin_ser3_inv, OUTPUT);
      digitalWrite(cfg.pin_ser3_inv, LOW); //not inverted
    }
  }
  if(cfg.pin_ser4_tx >= 0 && cfg.pin_ser4_rx >= 0) {
    auto *ser =new HardwareSerial(cfg.pin_ser4_rx, cfg.pin_ser4_tx);
    hal_ser[4] = new MF_SerialPtrWrapper<decltype(ser)>( ser );
    if(cfg.pin_ser4_inv >= 0) {
      pinMode(cfg.pin_ser4_inv, OUTPUT);
      digitalWrite(cfg.pin_ser4_inv, LOW); //not inverted
    }
  }
  if(cfg.pin_ser5_tx >= 0 && cfg.pin_ser5_rx >= 0) {
    auto *ser =new HardwareSerial(cfg.pin_ser5_rx, cfg.pin_ser5_tx);
    hal_ser[5] = new MF_SerialPtrWrapper<decltype(ser)>( ser );
    if(cfg.pin_ser5_inv >= 0) {
      pinMode(cfg.pin_ser5_inv, OUTPUT);
      digitalWrite(cfg.pin_ser5_inv, LOW); //not inverted
    }
  }
  if(cfg.pin_ser6_tx >= 0 && cfg.pin_ser6_rx >= 0) {
    auto *ser =new HardwareSerial(cfg.pin_ser6_rx, cfg.pin_ser6_tx);
    hal_ser[6] = new MF_SerialPtrWrapper<decltype(ser)>( ser );
    if(cfg.pin_ser6_inv >= 0) {
      pinMode(cfg.pin_ser6_inv, OUTPUT);
      digitalWrite(cfg.pin_ser6_inv, LOW); //not inverted
    }
  }
  if(cfg.pin_ser7_tx >= 0 && cfg.pin_ser7_rx >= 0) {
    auto *ser =new HardwareSerial(cfg.pin_ser7_rx, cfg.pin_ser7_tx);
    hal_ser[7] = new MF_SerialPtrWrapper<decltype(ser)>( ser );
    if(cfg.pin_ser7_inv >= 0) {
      pinMode(cfg.pin_ser7_inv, OUTPUT);
      digitalWrite(cfg.pin_ser7_inv, LOW); //not inverted
    }
  }

  //I2C BUS
  if(cfg.pin_i2c0_sda >= 0 && cfg.pin_i2c0_scl >= 0) {
    auto *i2c = &Wire;
    i2c->setSDA(cfg.pin_i2c0_sda);
    i2c->setSCL(cfg.pin_i2c0_scl);
    i2c->setClock(1000000);
    i2c->begin();
    hal_i2c[0] = new MF_I2CPtrWrapper<decltype(i2c)>( i2c );
  }
  if(cfg.pin_i2c1_sda >= 0 && cfg.pin_i2c1_scl >= 0) {
    TwoWire *i2c = new TwoWire(cfg.pin_i2c1_sda, cfg.pin_i2c1_scl);
    i2c->setClock(1000000);
    i2c->begin();
    hal_i2c[1] = new MF_I2CPtrWrapper<decltype(i2c)>( i2c );
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
