/*########################################################################################################################
This file contains all necessary functions and code for RP2040 hardware platforms

This file defines:
  HW_PIN_xxx -> The pin assignments
  *rcin_Serial -> Serial port for RCIN as MF_Serial object
  *gps_Serial -> Serial port for GPS as MF_Serial object
  *mf_i2c -> I2C port as MF_I2C object
  *spi -> SPI port
  hal_setup() -> function to init the hardware
  HAL_xxx and hal_xxx -> all other hardware platform specific stuff
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
#endif

//======================================================================================================================//
//                    FREERTOS
//======================================================================================================================//
#include <FreeRTOS.h>
#include <semphr.h>
#define FREERTOS_DEFAULT_STACK_SIZE 512 //stack size in 32bit words

//======================================================================================================================//
//                    hal_setup()
//======================================================================================================================//

const int HW_PIN_OUT[] = HW_PIN_OUT_LIST;

//-------------------------------------
//Include Libraries
//-------------------------------------
#include <pico/stdlib.h>               //set_sys_clock_khz()
#include <Wire.h>                      //I2C communication
#include <SPI.h>                       //SPI communication
#include "RP2040_PWM.h"  //Servo and onshot
#include "RP2040_SerialIRQ.h"  //Replacement high performance serial driver
#include "../../common/MF_Serial.h"

//-------------------------------------
//Bus Setup
//-------------------------------------

SPIClassRP2040 *spi = new SPIClassRP2040(spi0, HW_PIN_SPI_MISO, HW_PIN_IMU_CS, HW_PIN_SPI_SCLK, HW_PIN_SPI_MOSI); //spi0 or spi1
SPIClassRP2040 *bb_spi = new SPIClassRP2040(spi1, HW_PIN_SPI2_MISO, HW_PIN_BB_CS, HW_PIN_SPI2_SCLK, HW_PIN_SPI2_MOSI); //spi0 or spi1

//prototype
void hal_eeprom_begin();

uint8_t rcin_txbuf[256];
uint8_t rcin_rxbuf[256];
uint8_t gps_txbuf[256];
uint8_t gps_rxbuf[256];

void hal_setup() {
  //print hw info
  Serial.print("HW_RP2040 ");
  //overclocking, supposedly works up to 270 MHz on RP2040
  #ifdef HW_RP2040_SYS_CLK_KHZ
    set_sys_clock_khz(HW_RP2040_SYS_CLK_KHZ, true);
    Serial.printf(" HW_RP2040_SYS_CLK_KHZ %d", HW_RP2040_SYS_CLK_KHZ);
  #endif
  Serial.println();

  //rcin_Serial - global serial port for RCIN
  #if defined(HW_PIN_RCIN_TX) && defined(HW_PIN_RCIN_RX)
    //uncomment one: SerialIRQ, SerialUART or SerialPIO and use uart0 or uart1
    auto *rcin_ser = new SerialIRQ(uart0, HW_PIN_RCIN_TX, rcin_txbuf, sizeof(rcin_txbuf), HW_PIN_RCIN_RX, rcin_rxbuf, sizeof(rcin_rxbuf));
    //auto *rcin_ser = new SerialIRQ(uart0, HW_PIN_RCIN_TX, HW_PIN_RCIN_RX, 256, 256); //TODO
    //auto *rcin_ser = new SerialDMA(uart0, HW_PIN_RCIN_TX, HW_PIN_RCIN_RX, 256, 256); //TODO
    //auto *rcin_ser = new SerialUART(uart0, HW_PIN_RCIN_TX, HW_PIN_RCIN_RX); //SerialUART default Arduino impementation (had some problems with this)
    //auto *rcin_ser = new SerialPIO(HW_PIN_RCIN_TX, HW_PIN_RCIN_RX, 32); //PIO uarts, any pin allowed (not tested, but expect same as SerialUART)
    rcin_Serial = new MF_SerialPtrWrapper<decltype(rcin_ser)>( rcin_ser );
  #endif

  //gps_Serial - global serial port for GPS
  #if defined(HW_PIN_GPS_TX) && defined(HW_PIN_GPS_RX)
    //uncomment one: SerialIRQ, SerialUART or SerialPIO and use uart0 or uart1
    auto *gps_ser = new SerialIRQ(uart1, HW_PIN_GPS_TX, gps_txbuf, sizeof(gps_txbuf), HW_PIN_GPS_RX, gps_rxbuf, sizeof(gps_rxbuf));
    //auto *gps_ser = new SerialIRQ(uart1, HW_PIN_GPS_TX, HW_PIN_GPS_RX, 256, 256); //TODO
    //auto *gps_ser = new SerialDMA(uart1, HW_PIN_GPS_TX, HW_PIN_GPS_RX, 256, 256); //TODO
    //auto *gps_ser = new SerialUART(uart1, HW_PIN_GPS_TX, HW_PIN_GPS_RX); //SerialUART default Arduino impementation (had some problems with this)
    //auto *gps_ser = new SerialPIO(HW_PIN_GPS_TX, HW_PIN_GPS_RX, 32); //PIO uarts, any pin allowed (not tested, but expect same as SerialUART)
    gps_Serial = new MF_SerialPtrWrapper<decltype(gps_ser)>( gps_ser );
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
  spi->begin();
  bb_spi->begin();

  hal_eeprom_begin();

  //IMU
  pinMode(HW_PIN_IMU_EXTI, INPUT); //needed for RP2350, should not hurt for RP2040
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

inline uint32_t hal_get_core_num() {
  return get_core_num();
}

int hal_get_pin_number(String val) {
  return val.toInt();
}
