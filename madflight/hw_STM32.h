#include "variant_generic.h"
#include "pins_arduino.h"

//Note: this implementation does not check if a PWM frequency overwrites the frequency of previously instantiated PWM instance

/*########################################################################################################################
This file contains all necessary functions and code for specific hardware platforms to avoid cluttering the main code

This file defines:
  HW_PIN_xxx -> The pin assignments
  *rcin_Serial -> Serial port for RCIN
  *spi -> SPI port
  *i2c -> I2C port
  HW_WIRETYPE -> the class to use for I2C
  hw_Setup() -> function to init the hardware
  HW_xxx and hw_xxx -> all other hardware platform specific stuff
########################################################################################################################*/

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
//                    DEFAULT BOARD (used if no board set in madflight.ino)                                             //
//======================================================================================================================//
#ifndef HW_BOARD_NAME
#define HW_BOARD_NAME "DEFAULT STM32 BOARD - Black Pill STM32F411CEUx" //This pin layout is based on MATEK F411SE (MTKS-MATEKF411SE betaflight target)
#define HW_MCU "STM32F411CEUx" //STM32F411CEUx - not all pin combinations are allowed, see datasheet


//Arduino F411 defines: -DSTM32F4xx -DARDUINO=10607 -DARDUINO_GENERIC_F411CEUX -DARDUINO_ARCH_STM32 -DBOARD_NAME="GENERIC_F411CEUX" -DVARIANT_H="variant_generic.h" -DSTM32F411xE -DUSBCON -DUSBD_VID=0 -DUSBD_PID=0 -DHAL_PCD_MODULE_ENABLED -DUSBD_USE_CDC -DHAL_UART_MODULE_ENABLED

//Arduino F411: Serial and Serial1 both map TX1/RX1 on pin A9/A10. 
//Arduino F411: Serial debug on USB Serial port (USB is on on PA11,PA12, shared with USART6)
//TX1:PA9,PA15,PB6
//RX1:PA10,PB3,PB7
//TX2:PA2
//RX2:PA3
//TX6:PA11
//RX6:PA12

//-------------------------------------
// PIN DEFINITIONS
//-------------------------------------
//NOTE: DON'T USE SAME PIN TWICE. All pins here get configured, even if they are not used. Set pin to -1 to disable.

//LED:
const int HW_PIN_LED      = PC13;
const int HW_LED_ON       = 0; //0:low is on, 1:high is on

//IMU SPI: (SPI1)
const int HW_PIN_SPI_MISO = PA6;
const int HW_PIN_SPI_MOSI = PA7;
const int HW_PIN_SPI_SCLK = PA5;
const int HW_PIN_IMU_CS   = PA4;
const int HW_PIN_IMU_EXTI = PB10;

//BARO/MAG I2C: (I2C1)
const int HW_PIN_I2C_SDA  = PB6;
const int HW_PIN_I2C_SCL  = PB7;

//Outputs:
const int HW_OUT_COUNT    = 6;
const int HW_PIN_OUT[HW_OUT_COUNT] = {PB2,PB5,PA8,PA9,PA10,PB8};

//RC Receiver: (SERIAL1)
const int HW_PIN_RCIN_RX  = PA3;
const int HW_PIN_RCIN_TX  = PA2;
const int HW_PIN_RCIN_INVERTER = -1;

//GPS: (SERIAL3)
const int HW_PIN_GPS_RX   = PB3;
const int HW_PIN_GPS_TX   = PA15;
const int HW_PIN_GPS_INVERTER = -1;

//Battery ADC (uncomment at least one pin to enable the battery monitor)
#define HW_PIN_BAT_V PB0
#define HW_PIN_BAT_I PB1

//BlackBox SPI:
const int HW_PIN_SPI2_MISO = -1;
const int HW_PIN_SPI2_MOSI = -1;
const int HW_PIN_SPI2_SCLK = -1;
const int HW_PIN_BB_CS     = -1;

//Include Libraries
#include <Wire.h>                      //I2C communication
#include <SPI.h>                       //SPI communication
#include "src/hw_STM32/STM32_PWM.h"    //Servo and onshot

//Bus Setup
HardwareSerial *rcin_Serial = new HardwareSerial(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);
HardwareSerial gps_Serial(HW_PIN_GPS_RX, HW_PIN_GPS_TX);
typedef TwoWire HW_WIRETYPE; //define the class to use for I2C
HW_WIRETYPE *i2c = &Wire; //&Wire or &Wire1
SPIClass *spi = &SPI;
SPIClass bb_spi = SPIClass(HW_PIN_SPI2_MOSI, HW_PIN_SPI2_MISO, HW_PIN_SPI2_SCLK); //do not define HW_PIN_BB_CS here

#endif //#ifndef HW_BOARD_NAME

//======================================================================================================================//
//  EEPROM
//======================================================================================================================//

#include <EEPROM.h>

#if defined(DATA_EEPROM_BASE)
  //----------------------------------------------------------------------------------------------------------
  //unbuffered write - very slow because writes whole flash page for each byte, i.e. 1 second per changed byte
  //----------------------------------------------------------------------------------------------------------

  void hw_eeprom_begin() {
    //EEPROM.begin(); //STM does not use size in begin() call
  }

  uint8_t hw_eeprom_read(uint32_t adr) {
    return EEPROM.read(adr);
  }

  void hw_eeprom_write(uint32_t adr, uint8_t val) {
    EEPROM.update(adr, val); //update only writes when changed
  }

  void hw_eeprom_commit() {
    //EEPROM.commit();  //STM does not use commit(), write() also executes commit()
  }
#else
  //----------------------------------------------------------------------------------------------------------
  //buffered write - takes approx. 1 second to write full config
  //----------------------------------------------------------------------------------------------------------
  void hw_eeprom_begin() {
    UNUSED(EEPROM);    
    //Serial.println("START reading from flash");Serial.flush();
    eeprom_buffer_fill(); //Copy the data from the flash to the buffer
    //Serial.println("DONE reading");Serial.flush();
  }

  uint8_t hw_eeprom_read(uint32_t adr) {  
    uint8_t val = eeprom_buffered_read_byte(adr); //read from buffer
    //Serial.printf("hw_eeprom_read(%d)=%d\n",adr,val);Serial.flush();
    return val;
  }

  void hw_eeprom_write(uint32_t adr, uint8_t val) {
    //Serial.printf("hw_eeprom_write(%d,%d)\n",adr,val);Serial.flush();
    eeprom_buffered_write_byte(adr, val); //write to buffer
  }

  void hw_eeprom_commit() {
    //Serial.println("START writing to flash");Serial.flush();
    eeprom_buffer_flush(); //Copy the data from the buffer to the flash
    //Serial.println("DONE writing");Serial.flush();
  } 
#endif

//======================================================================================================================//
//                    hw_setup()
//======================================================================================================================//

//#define HW_USE_FREERTOS //STM FreeRTOS not supported (yet), leave commented out

void hw_setup() 
{ 
  Serial.println("USE_HW_STM32");
  
  //Serial RX Inverters
  pinMode(HW_PIN_RCIN_INVERTER, OUTPUT);
  digitalWrite(HW_PIN_RCIN_INVERTER, LOW); //not inverted
  pinMode(HW_PIN_GPS_INVERTER, OUTPUT);
  digitalWrite(HW_PIN_GPS_INVERTER, LOW); //not inverted

  //I2C
  i2c->setSDA(HW_PIN_I2C_SDA);
  i2c->setSCL(HW_PIN_I2C_SCL);
  i2c->setClock(1000000);
  i2c->begin();

  //SPI 
  spi->setMISO(HW_PIN_SPI_MISO);
  spi->setSCLK(HW_PIN_SPI_SCLK);
  spi->setMOSI(HW_PIN_SPI_MOSI);
  //spi->setSSEL(HW_PIN_IMU_CS); //don't set CS here, it is done in the driver to be compatible with other hardware platforms
  spi->begin();
}

void hw_reboot() {
  NVIC_SystemReset();
}
