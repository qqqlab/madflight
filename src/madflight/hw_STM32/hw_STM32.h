#include "variant_generic.h"
#include "pins_arduino.h"

//Note: this implementation does not check if a PWM frequency overwrites the frequency of previously instantiated PWM instance

/*########################################################################################################################
This file contains all necessary functions and code for STM32 hardware platforms

This file defines:
  HW_PIN_xxx -> The pin assignments
  *rcin_Serial -> Serial port for RCIN
  *spi -> SPI port
  *i2c -> I2C port
  HW_WIRETYPE -> the class to use for I2C
  hw_Setup() -> function to init the hardware
  HW_xxx and hw_xxx -> all other hardware platform specific stuff
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
#define IMU_EXEC IMU_EXEC_IRQ //STM FreeRTOS not supported (yet), so use IMU as interrupt

//======================================================================================================================//
//                    FREERTOS
//======================================================================================================================//
#include <STM32FreeRTOS.h>
#define FREERTOS_DEFAULT_STACK_SIZE 512 //stack size in 32bit words

//======================================================================================================================//
//                    hw_setup()
//======================================================================================================================//
const int HW_PIN_OUT[] = HW_PIN_OUT_LIST;

//Include Libraries
#include <Wire.h> //I2C communication
#include <SPI.h> //SPI communication
#include "madflight/hw_STM32/STM32_PWM.h" //Servo and oneshot

//Bus Setup
#if defined(HW_PIN_RCIN_RX) && defined(HW_PIN_RCIN_TX)
  HardwareSerial *rcin_Serial = new HardwareSerial(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);
#else
  HardwareSerial *rcin_Serial = new HardwareSerial(-1, -1); //DUMMY
#endif

#if defined(HW_PIN_RCIN_TX) && defined(HW_PIN_RCIN_RX)
  HardwareSerial gps_Serial(HW_PIN_GPS_RX, HW_PIN_GPS_TX);
#else
  HardwareSerial gps_Serial(-1, -1); //DUMMY
#endif

typedef TwoWire HW_WIRETYPE; //define the class to use for I2C
HW_WIRETYPE *i2c = &Wire; //&Wire or &Wire1

SPIClass *spi = &SPI;

//TODO: SPIClass *bb_spi;

//prototype
void hw_eeprom_begin();

void hw_setup() 
{ 
  Serial.println("USE_HW_STM32");
  
  //Serial RX Inverters
  #ifdef HW_PIN_RCIN_INVERTER
    pinMode(HW_PIN_RCIN_INVERTER, OUTPUT);
    digitalWrite(HW_PIN_RCIN_INVERTER, LOW); //not inverted
  #endif
  #ifdef HW_PIN_GPS_INVERTER  
    pinMode(HW_PIN_GPS_INVERTER, OUTPUT);
    digitalWrite(HW_PIN_GPS_INVERTER, LOW); //not inverted
  #endif
  
  //I2C
  #if defined(HW_PIN_I2C_SDA) && defined(HW_PIN_I2C_SCL)
    i2c->setSDA(HW_PIN_I2C_SDA);
    i2c->setSCL(HW_PIN_I2C_SCL);
    i2c->setClock(1000000);
    i2c->begin();
  #endif

  //SPI
  #if defined(HW_PIN_SPI_SCLK) && defined(HW_PIN_SPI_MISO) && defined(HW_PIN_SPI_MOSI)
    spi->setMISO(HW_PIN_SPI_MISO);
    spi->setSCLK(HW_PIN_SPI_SCLK);
    spi->setMOSI(HW_PIN_SPI_MOSI);
    //spi->setSSEL(HW_PIN_IMU_CS); //don't set CS here, it is done in the driver to be compatible with other hardware platforms
    spi->begin();
  #endif

  hw_eeprom_begin();
}

//======================================================================================================================//
//  EEPROM
//======================================================================================================================//
#if defined(DATA_EEPROM_BASE)
  //----------------------------------------------------------------------------------------------------------
  //unbuffered write - very slow because writes whole flash page for each byte, i.e. 1 second per changed byte
  //----------------------------------------------------------------------------------------------------------
  #include <EEPROM.h>

  void hw_eeprom_begin() {
    Serial.println("EEPROM: using Unbuffered IO");
    //EEPROM.begin(); //STM does not use size in begin() call
  }

  uint8_t hw_eeprom_read(uint32_t adr) {
    uint8_t val = EEPROM.read(adr);
    //Serial.printf("EEPROM.read(%d) = 0x%02X\n", adr, val);
    return val;
  }

  void hw_eeprom_write(uint32_t adr, uint8_t val) {
    EEPROM.update(adr, val); //update only writes when changed
    //Serial.printf("EEPROM.write(%d, 0x%02X)\n", adr, val);
  }

  void hw_eeprom_commit() {
    //EEPROM.commit();  //STM does not use commit(), write() also executes commit()
  }
#else
  //----------------------------------------------------------------------------------------------------------
  //buffered write - takes approx. 1 second to write full config
  //----------------------------------------------------------------------------------------------------------
  #include <EEPROM.h>

  void hw_eeprom_begin() {
    (void)(EEPROM); //keep compiler happy
    Serial.println("EEPROM: using Buffered IO");
    //Serial.println("START reading from flash");Serial.flush();
    eeprom_buffer_fill(); //Copy the data from the flash to the buffer
    //Serial.println("DONE reading");Serial.flush();
  }

  uint8_t hw_eeprom_read(uint32_t adr) {
    uint8_t val = eeprom_buffered_read_byte(adr); //read from buffer
    //Serial.printf("hw_eeprom_read(%d) = 0x%02X\n", adr, val);Serial.flush();
    return val;
  }

  void hw_eeprom_write(uint32_t adr, uint8_t val) {
    //Serial.printf("hw_eeprom_write(%d, 0x%02X)\n", adr, val);Serial.flush();
    eeprom_buffered_write_byte(adr, val); //write to buffer
  }

  void hw_eeprom_commit() {
    //Serial.println("START writing to flash");Serial.flush();
    eeprom_buffer_flush(); //Copy the data from the buffer to the flash
    eeprom_buffer_flush(); //TODO: calling flush twice seems to do the trick???
    //Serial.println("DONE writing");Serial.flush();
  } 
#endif

//======================================================================================================================//
//  MISC
//======================================================================================================================//
void hw_reboot() {
  NVIC_SystemReset();
}
/*
void hw_disable_irq() {
  __disable_irq();
}

void hw_enable_irq() {
  __enable_irq();
}
*/

uint32_t hw_get_core_num() {
  return 0;
}
