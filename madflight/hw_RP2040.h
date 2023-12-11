/*########################################################################################################################
This file contains all necessary functions and code for specific hardware platforms to avoid cluttering the main code

This file defines:
  xxx_PIN -> The pin assignments
  *rcin_Serial -> Serial port for RCIN
  *spi -> SPI port
  *i2c -> I2C port
  HW_WIRETYPE -> the class to use for I2C
  hw_Setup() -> function to init the hardware
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

//LED:
const int led_PIN      = 25; //internal on Raspberry Pi Pico

//Serial debug on USB Serial port (no GPIO pins)

//RC Receiver:
const int rcin_PPM_PIN = 1; //can be same as rcin_RX_PIN
const int rcin_RX_PIN  = 1; //uart0: 1(default), 5, 13, 17   uart1: 5, 9(default)
const int rcin_TX_PIN  = 0; //uart0: 0(default), 4, 12, 16   uart1: 4, 8(default)
//uncomment one line only
SerialUART *rcin_Serial = new SerialUART(uart0, rcin_TX_PIN, rcin_RX_PIN); //uart0 or uart1
//SerialPIO *rcin_Serial = new SerialPIO(rcin_TX_PIN, rcin_RX_PIN, 32); //PIO uarts, any pin allowed

//IMU:
const int imu_INT_PIN  = 22;

//I2C:
const int i2c_SDA_PIN  = 20; //Wire: 0, 4(default), 8, 12, 16, 20   Wire1: 2, 6, 10, 14, 18, 26(default)
const int i2c_SCL_PIN  = 21; //Wire: 1, 5(default), 9, 13, 17, 21   Wire1: 3, 7, 11, 15, 19, 27(default)
typedef TwoWire HW_WIRETYPE; //define the class to use for I2C
HW_WIRETYPE *i2c = &Wire; //&Wire or &Wire1

//SPI:
const int spi_MISO_PIN = 16; //spi0: 0, 4, 16(default)   spi1:  8, 12(default)
const int spi_CS_PIN   = 17; //spi0: 1, 5, 17(default)   spi1:  9, 13(default)
const int spi_SCLK_PIN = 18; //spi0: 2, 6, 18(default)   spi1: 10, 14(default)
const int spi_MOSI_PIN = 19; //spi0: 3, 7, 19(default)   spi1: 11, 15(default)
SPIClassRP2040 *spi = new SPIClassRP2040(spi0, spi_MISO_PIN, spi_CS_PIN, spi_SCLK_PIN, spi_MOSI_PIN); //spi0 or spi1

//Outputs:
#define hw_OUT_COUNT 14
const int8_t out_PIN[hw_OUT_COUNT] = {2,3,4,5,6,7,8,9,10,11,12,13,14,15};

void hw_setup() 
{ 
  //Uncomment for overclocking,supposedly works up to 270 MHz. Use with care! 
  //set_sys_clock_khz(200000, true); 

  Serial.println("USE_HW_RP2040");
  
  //I2C
  i2c->setSDA(i2c_SDA_PIN);
  i2c->setSCL(i2c_SCL_PIN);
  i2c->setClock(1000000); //Note: this is 2.5 times the MPU6050/MPU9150 spec sheet 400 kHz max...
  i2c->begin();

  //SPI
  spi->begin();
}
