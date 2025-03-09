#include "variant_generic.h"
#include "pins_arduino.h"

//Note: this implementation does not check if a PWM frequency overwrites the frequency of previously instantiated PWM instance

/*########################################################################################################################
This file contains all necessary functions and code for STM32 hardware platforms

This file defines:
  HW_PIN_xxx -> The pin assignments
  *rcin_Serial -> Serial port for RCIN as MF_Serial object
  *gps_Serial -> Serial port for GPS as MF_Serial object
  *mf_i2c -> I2C port as MF_I2C object
  *spi -> SPI port
  hal_setup() -> function to init the hardware
  HAL_xxx and hal_xxx -> all other hardware platform specific stuff
########################################################################################################################*/

#define df2xstr(s)              #s
#define df2str(s)               df2xstr(s)
#define HW_ARDUINO_STR "Arduino_Core_STM32 v" df2str(STM32_CORE_VERSION_MAJOR) "." df2str(STM32_CORE_VERSION_MINOR) "." df2str(STM32_CORE_VERSION_PATCH)

//Arduino IDE settings:
//Board: Generic STM32xxx
//Board part number: select according your board
//C Runtime Library: "Newlib Nano + Float Printf"
//USB support: "CDC (general Serial supperseed U(S)ART)"
//U(S)ART) support: "Enabled (generic 'Serial')"

//Programming STM32 targets:
//USB cable: upload method "STM32CubeProgrammer (DFU)" --> press boot button, connect usb cable (or press/release reset) 
//ST-LINK dongle: upload method "STM32CubeProgrammer (SWD)" --> press boot, press/release reset button (or power board)

//======================================================================================================================//
//                    DEFAULT PINS
//======================================================================================================================//
#ifndef HW_BOARD_NAME
  #include <madflight_board_default_STM32.h>
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
const int HW_PIN_OUT[] = HW_PIN_OUT_LIST;

//Include Libraries
#include <Wire.h> //I2C communication
#include <SPI.h> //SPI communication
#include "STM32_PWM.h" //Servo and oneshot
#include "../../common/MF_Serial.h"

//Bus Setup
SPIClass *spi = &SPI;
//TODO: SPIClass *bb_spi;

//prototype
void hal_eeprom_begin();

void hal_setup() 
{ 
  Serial.println("USE_HW_STM32");

  //NOTE: default serial buffer size is 64, and is defined in HardwareSerial.h
  //SERIAL_RX_BUFFER_SIZE and SERIAL_TX_BUFFER_SIZE
  //can't set that here :-(
  //need to use compiler -D arguments or modify HardwareSerial.h ...
  #if SERIAL_RX_BUFFER_SIZE<256 || SERIAL_TX_BUFFER_SIZE<256
    #warning "RCIN/GPS might need larger buffers. Set SERIAL_RX_BUFFER_SIZE 256 and SERIAL_TX_BUFFER_SIZE 256 in HardwareSerial.h"
  #endif

  //rcin_Serial - global serial port for RCIN
  #if defined(HW_PIN_RCIN_TX) && defined(HW_PIN_RCIN_RX)
    auto *rcin_ser = new HardwareSerial(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);
    //rcin_ser->setTxBufferSize(256);
    //rcin_ser->setRxBufferSize(256);
    rcin_Serial = new MF_SerialPtrWrapper<decltype(rcin_ser)>( rcin_ser );
    #ifdef HW_PIN_RCIN_INVERTER
      pinMode(HW_PIN_RCIN_INVERTER, OUTPUT);
      digitalWrite(HW_PIN_RCIN_INVERTER, LOW); //not inverted
    #endif    
  #endif

  //gps_Serial - global serial port for GPS
  #if defined(HW_PIN_GPS_TX) && defined(HW_PIN_GPS_RX)
    auto *gps_ser = new HardwareSerial(HW_PIN_GPS_RX, HW_PIN_GPS_TX);
    //gps_ser->setTxBufferSize(256);
    //gps_ser->setRxBufferSize(256);
    rcin_Serial = new MF_SerialPtrWrapper<decltype(gps_ser)>( gps_ser );
    #ifdef HW_PIN_GPS_INVERTER  
      pinMode(HW_PIN_GPS_INVERTER, OUTPUT);
      digitalWrite(HW_PIN_GPS_INVERTER, LOW); //not inverted
    #endif
  #endif

  //mf_i2c - global I2C interface
  #if defined(HW_PIN_I2C_SDA) && defined(HW_PIN_I2C_SCL)
    auto *i2c_ptr = &Wire; //&Wire or &Wire1 - type is TwoWire
    i2c_ptr->setSDA(HW_PIN_I2C_SDA);
    i2c_ptr->setSCL(HW_PIN_I2C_SCL);
    i2c_ptr->setClock(1000000);
    i2c_ptr->begin();
    mf_i2c = new MF_I2CPtrWrapper<decltype(i2c_ptr)>( i2c_ptr );
  #endif

  //SPI
  #if defined(HW_PIN_SPI_SCLK) && defined(HW_PIN_SPI_MISO) && defined(HW_PIN_SPI_MOSI)
    spi->setMISO(HW_PIN_SPI_MISO);
    spi->setSCLK(HW_PIN_SPI_SCLK);
    spi->setMOSI(HW_PIN_SPI_MOSI);
    //spi->setSSEL(HW_PIN_IMU_CS); //don't set CS here, it is done in the driver to be compatible with other hardware platforms
    spi->begin();
  #endif

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
