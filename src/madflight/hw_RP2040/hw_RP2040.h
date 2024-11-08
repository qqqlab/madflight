/*########################################################################################################################
This file contains all necessary functions and code for RP2040 hardware platforms

This file defines:
  HW_PIN_xxx -> The pin assignments
  *rcin_Serial -> Serial port for RCIN
  *spi -> SPI port
  *i2c -> I2C port
  HW_WIRETYPE -> the class to use for I2C
  hw_Setup() -> function to init the hardware
  HW_xxx and hw_xxx -> all other hardware platform specific stuff
########################################################################################################################*/

#define HW_ARDUINO_STR "Arduino-Pico v" ARDUINO_PICO_VERSION_STR 

//======================================================================================================================//
//                    DEFAULT BOARD
//======================================================================================================================//
#ifndef HW_BOARD_NAME
  #include <madflight_board_default_RP2040.h>
#endif

//======================================================================================================================//
//                    IMU
//======================================================================================================================//
#ifdef PICO_RP2350
  #define IMU_EXEC IMU_EXEC_IRQ
#else
  #define IMU_EXEC IMU_EXEC_FREERTOS_OTHERCORE //use FreeRTOS on second core (Note: RP2040 IMU_EXEC_IRQ blocks the system)
  #define IMU_FREERTOS_TASK_PRIORITY 7 //IMU Interrupt task priority, higher number is higher priority. Max priority on RP2040 is 7
  #define HW_RP2040_USE_FREERTOS //enable FreeRTOS
#endif

//======================================================================================================================//
//                    FREERTOS
//======================================================================================================================//
#include <FreeRTOS.h>
#include <semphr.h>
#define FREERTOS_DEFAULT_STACK_SIZE 512 //stack size in 32bit words

//======================================================================================================================//
//                    hw_setup()
//======================================================================================================================//

//overclocking, supposedly works up to 270 MHz on RP2040.
//#ifndef HW_RP2040_SYS_CLK_KHZ
//  #define HW_RP2040_SYS_CLK_KHZ 200000
//#endif

const int HW_PIN_OUT[] = HW_PIN_OUT_LIST;

//-------------------------------------
//Include Libraries
//-------------------------------------
#include <pico/stdlib.h>               //set_sys_clock_khz()
#include <Wire.h>                      //I2C communication
#include <SPI.h>                       //SPI communication
#include "madflight/hw_RP2040/RP2040_PWM.h"  //Servo and onshot
#include "madflight/hw_RP2040/RP2040_SerialIRQ.h"  //Replacement high performance serial driver

//-------------------------------------
//Bus Setup
//-------------------------------------

#if defined(HW_PIN_GPS_TX) && defined(HW_PIN_GPS_RX)
  // GPS Serial - uncomment one: SerialUART, SerialIRQ or SerialPIO and use uart0 or uart1
  uint8_t gps_txbuf[256];
  uint8_t gps_rxbuf[256];
  SerialIRQ gps_Serial(uart1, HW_PIN_GPS_TX, gps_txbuf, sizeof(gps_txbuf), HW_PIN_GPS_RX, gps_rxbuf, sizeof(gps_rxbuf)); //SerialIRQ uses buffered tx and rx in addition to the 32 byte uart hardware fifo
  //SerialUART gps_Serial = SerialUART(uart1, HW_PIN_GPS_TX, HW_PIN_GPS_RX); //SerialUART default Arduino impementation (had some problems with this)
  //SerialPIO gps_Serial = SerialPIO(HW_PIN_GPS_TX, HW_PIN_GPS_RX, 32); //PIO uarts, any pin allowed (not tested)
#else
  //DUMMY
  SerialUART gps_Serial = SerialUART(uart1, -1, -1);
#endif

#if defined(HW_PIN_RCIN_TX) && defined(HW_PIN_RCIN_RX)
  // RC Input Serial - uncomment one: SerialUART, SerialIRQ or SerialPIO and use uart0 or uart1
  uint8_t rcin_txbuf[256];
  uint8_t rcin_rxbuf[1024];
  SerialIRQ *rcin_Serial = new SerialIRQ(uart0, HW_PIN_RCIN_TX, rcin_txbuf, sizeof(rcin_txbuf), HW_PIN_RCIN_RX, rcin_rxbuf, sizeof(rcin_rxbuf)); //SerialIRQ uses buffered tx and rx in addition to the 32 byte uart hardware fifo
  //SerialUART rcin_Serial = SerialUART(uart0, HW_PIN_RCIN_TX, HW_PIN_RCIN_RX); //SerialUART default Arduino impementation (had some problems with this)
  //SerialPIO *rcin_Serial = new SerialPIO(HW_PIN_RCIN_TX, HW_PIN_RCIN_RX, 32); //PIO uarts, any pin allowed (not tested)
#else
  //DUMMY
  SerialUART *rcin_Serial = new SerialUART(uart0, -1, -1);
#endif

typedef TwoWire HW_WIRETYPE; //define the class to use for I2C
HW_WIRETYPE *i2c = &Wire; //&Wire or &Wire1

SPIClassRP2040 *spi = new SPIClassRP2040(spi0, HW_PIN_SPI_MISO, HW_PIN_IMU_CS, HW_PIN_SPI_SCLK, HW_PIN_SPI_MOSI); //spi0 or spi1
SPIClassRP2040 *bb_spi = new SPIClassRP2040(spi1, HW_PIN_SPI2_MISO, HW_PIN_BB_CS, HW_PIN_SPI2_SCLK, HW_PIN_SPI2_MOSI); //spi0 or spi1

//prototype
void hw_eeprom_begin();

void hw_setup() 
{ 
  //print hw info
  Serial.print("HW_RP2040 ");
  #ifdef HW_RP2040_USE_FREERTOS
    Serial.print("HW_RP2040_USE_FREERTOS ");
  #endif
  #ifdef HW_RP2040_SYS_CLK_KHZ
    set_sys_clock_khz(HW_RP2040_SYS_CLK_KHZ, true);
    Serial.printf(" HW_RP2040_SYS_CLK_KHZ %d",HW_RP2040_SYS_CLK_KHZ);
  #endif
  Serial.println();

  //I2C
  i2c->setSDA(HW_PIN_I2C_SDA);
  i2c->setSCL(HW_PIN_I2C_SCL);
  i2c->setClock(1000000);
  i2c->begin();

  //SPI
  spi->begin();
  bb_spi->begin();

  hw_eeprom_begin();

  //IMU
  pinMode(HW_PIN_IMU_EXTI, INPUT); //needed for RP2350, should not hurt for RP2040
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
//  MISC
//======================================================================================================================//

void hw_reboot() {

  //does not always work with FreeRTOS running
  watchdog_enable(10, false); //uint32_t delay_ms, bool pause_on_debug
  while(1);

  //does not always work with FreeRTOS running
  //watchdog_reboot(0, 0, 10); //uint32_t pc, uint32_t sp, uint32_t delay_ms
  //while(1);

  //alternate method - does not work...
  //#define AIRCR_Register (*((volatile uint32_t*)(PPB_BASE + 0x0ED0C)))
  //AIRCR_Register = 0x5FA0004;
}

inline uint32_t hw_get_core_num() {
  return get_core_num();
}