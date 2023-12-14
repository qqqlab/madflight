/*########################################################################################################################
This file contains all necessary functions and code for specific hardware platforms to avoid cluttering the main code

This file defines:
  HW_PIN_xxx -> The pin assignments
  *rcin_Serial -> Serial port for RCIN
  *spi -> SPI port
  *i2c -> I2C port
  HW_WIRETYPE -> the class to use for I2C
  hw_Setup() -> function to init the hardware
  HW_xxx and hw_xxx -> all other hardware platform specifi stuff
########################################################################################################################*/

//======================================================================================================================//
//                    HARDWARE DEFINITION for Raspberry Pi Pico (W) board                                               //
//======================================================================================================================//
//RP2040 - not all pin combinations are allowed, see datasheet
//This pin layout is optimized for Raspberry Pi Pico board: UART, PWM on side; I2C, SPI, PPM on the other side

#include <Wire.h>                      //I2C communication
#include <SPI.h>                       //SPI communication
#include "src/HW_RP2040/RP2040_PWM.h"  //Servo and onshot
#include <pico/stdlib.h>               //set_sys_clock_khz()

//NOTE: DON'T USE SAME PIN TWICE. All pins here get configured, even if they are not used. Set pin to -1 to disable.

//LED:
const int HW_PIN_LED      = 25; //internal on Raspberry Pi Pico

//Serial debug on USB Serial port (no GPIO pins)

//RC Receiver:
const int HW_PIN_RCIN_RX  = 1; //uart0: 1(default), 5, 13, 17   uart1: 5, 9(default) , this pin is also used as PPM input
const int HW_PIN_RCIN_TX  = 0; //uart0: 0(default), 4, 12, 16   uart1: 4, 8(default)
//uncomment one line only
SerialUART *rcin_Serial = new SerialUART(uart0, HW_PIN_RCIN_TX, HW_PIN_RCIN_RX); //uart0 or uart1
//SerialPIO *rcin_Serial = new SerialPIO(HW_PIN_RCIN_TX, HW_PIN_RCIN_RX, 32); //PIO uarts, any pin allowed

//IMU:
const int HW_PIN_IMU_INT  = 22; //only used when USE_IMU_INTERRUPT is defined
#define HW_RTOS_IMUTASK_PRIORITY 7 //IMU Interrupt task priority, higher number is higher priority. Max priority on RP2040 is 7

//I2C:
const int HW_PIN_I2C_SDA  = 20; //Wire: 0, 4(default), 8, 12, 16, 20   Wire1: 2, 6, 10, 14, 18, 26(default)
const int HW_PIN_I2C_SCL  = 21; //Wire: 1, 5(default), 9, 13, 17, 21   Wire1: 3, 7, 11, 15, 19, 27(default)
typedef TwoWire HW_WIRETYPE; //define the class to use for I2C
HW_WIRETYPE *i2c = &Wire; //&Wire or &Wire1

//SPI:
const int HW_PIN_SPI_MISO = 16; //spi0: 0, 4, 16(default)   spi1:  8, 12(default)
const int HW_PIN_SPI_CS   = 17; //spi0: 1, 5, 17(default)   spi1:  9, 13(default)
const int HW_PIN_SPI_SCLK = 18; //spi0: 2, 6, 18(default)   spi1: 10, 14(default)
const int HW_PIN_SPI_MOSI = 19; //spi0: 3, 7, 19(default)   spi1: 11, 15(default)
SPIClassRP2040 *spi = new SPIClassRP2040(spi0, HW_PIN_SPI_MISO, HW_PIN_SPI_CS, HW_PIN_SPI_SCLK, HW_PIN_SPI_MOSI); //spi0 or spi1

//Outputs:
#define HW_OUT_COUNT 14
const int8_t HW_PIN_OUT[HW_OUT_COUNT] = {2,3,4,5,6,7,8,9,10,11,12,13,14,15};

void hw_setup() 
{ 
  //Uncomment for overclocking, supposedly works up to 270 MHz.
  //set_sys_clock_khz(200000, true); 

  Serial.println("USE_HW_RP2040");
  
  //I2C
  i2c->setSDA(HW_PIN_I2C_SDA);
  i2c->setSCL(HW_PIN_I2C_SCL);
  i2c->setClock(1000000); //Note: this is 2.5 times the MPU6050/MPU9150 spec sheet 400 kHz max...
  i2c->begin();

  //SPI 
  spi->begin();
}
