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

//======================================================================================================================//
//                    DEFAULT BOARD (used if no board set in madflight.ino)                                             //
//======================================================================================================================//
#ifndef HW_BOARD_NAME
#define HW_BOARD_NAME "DEFAULT RP2040 BOARD - Raspberry Pi Pico (W)" //This pin layout is optimized for Raspberry Pi Pico board: UART, PWM on side; I2C, SPI, PPM on the other side
#define HW_MCU "RP2040" //RP2040 - not all pin combinations are allowed, see datasheet

//-------------------------------------
// IMU SENSOR
//-------------------------------------
//Uncomment only one USE_IMU_xxx
//#define USE_IMU_SPI_MPU9250  //same as MPU6500 plus magnetometer
#define USE_IMU_SPI_MPU6500
//#define USE_IMU_SPI_MPU6000
//#define USE_IMU_I2C_MPU9250  //same as MPU6500 plus magnetometer
//#define USE_IMU_I2C_MPU9150  //same as MPU6050 plus magnetometer
//#define USE_IMU_I2C_MPU6500
//#define USE_IMU_I2C_MPU6050
//#define USE_IMU_I2C_MPU6000

#define IMU_I2C_ADR 0 //Set I2C address. If unknown, see output of print_i2c_scan()

//Uncomment only one sensor orientation. The label is yaw / roll (in that order) needed to rotate the sensor from it's normal position to it's mounted position.
//if not sure what is needed: try each setting until roll-right gives positive ahrs_roll, pitch-up gives positive ahrs_pitch, and yaw-right gives increasing ahrs_yaw
#define IMU_ROTATE_CW0
//#define IMU_ROTATE_CW90
//#define IMU_ROTATE_CW180
//#define IMU_ROTATE_CW270
//#define IMU_ROTATE_CW0FLIP
//#define IMU_ROTATE_CW90FLIP
//#define IMU_ROTATE_CW180FLIP
//#define IMU_ROTATE_CW270FLIP

//-------------------------------------
// BAROMETER SENSOR
//-------------------------------------
//Uncomment only one USE_BARO_xxx
//#define USE_BARO_BMP280
//#define USE_BARO_MS5611
#define USE_BARO_NONE

#define BARO_I2C_ADR 0 //set barometer I2C address, or 0 for default. If unknown, see output of print_i2c_scan()

//-------------------------------------
// EXTERNAL MAGNETOMETER SENSOR
//-------------------------------------
//Uncomment only one USE_MAG_xxx
//#define USE_MAG_QMC5883L
#define USE_MAG_NONE

#define MAG_I2C_ADR 0 //set magnetometer I2C address, or 0 for default. If unknown, see output of print_i2c_scan()

//-------------------------------------
// PIN DEFINITIONS
//-------------------------------------
//NOTE: DON'T USE SAME PIN TWICE. All pins here get configured, even if they are not used. Set pin to -1 to disable.

//LED:
const int HW_PIN_LED      = 25; //internal on Raspberry Pi Pico
const int HW_LED_ON       =  1; //0:low is on, 1:high is on

//IMU SPI:
const int HW_PIN_SPI_MISO = 16; //spi0: 0, 4, 16(default)   spi1:  8, 12(default)
const int HW_PIN_SPI_MOSI = 19; //spi0: 3, 7, 19(default)   spi1: 11, 15(default)
const int HW_PIN_SPI_SCLK = 18; //spi0: 2, 6, 18(default)   spi1: 10, 14(default)
const int HW_PIN_IMU_CS   = 17; //spi0: 1, 5, 17(default)   spi1:  9, 13(default)
const int HW_PIN_IMU_EXTI = 22; //only used when USE_IMU_INTERRUPT is defined

//I2C:
const int HW_PIN_I2C_SDA  = 20; //Wire: 0, 4(default), 8, 12, 16, 20   Wire1: 2, 6, 10, 14, 18, 26(default)
const int HW_PIN_I2C_SCL  = 21; //Wire: 1, 5(default), 9, 13, 17, 21   Wire1: 3, 7, 11, 15, 19, 27(default)

//Outputs:
const int HW_OUT_COUNT    = 12;
const int HW_PIN_OUT[HW_OUT_COUNT] = {2,3,4,5,6,7,10,11,12,13,14,15};

//Serial debug on USB Serial port (no GPIO pins)

//RC Receiver:
const int HW_PIN_RCIN_RX  = 1; //uart0: 1(default), 5, 13, 17   uart1: 5, 9(default) , this pin is also used as PPM input
const int HW_PIN_RCIN_TX  = 0; //uart0: 0(default), 4, 12, 16   uart1: 4, 8(default)

//GPS:
const int HW_PIN_GPS_RX   = 9; //uart0: 1(default), 5, 13, 17   uart1: 5, 9(default)
const int HW_PIN_GPS_TX   = 8; //uart0: 0(default), 4, 12, 16   uart1: 4, 8(default)

//Battery ADC (uncomment at least one pin to enable the battery monitor)
#define HW_PIN_BAT_V 28; //pin A2
//#define HW_PIN_BAT_I -1

//BlackBox SPI:
const int HW_PIN_SPI2_MISO = -1;
const int HW_PIN_SPI2_MOSI = -1;
const int HW_PIN_SPI2_SCLK = -1;
const int HW_PIN_BB_CS   = -1;

//-------------------------------------
//Include Libraries
//-------------------------------------
#include <Wire.h>                      //I2C communication
#include <SPI.h>                       //SPI communication
#include "src/hw_RP2040/RP2040_PWM.h"  //Servo and onshot
#include <pico/stdlib.h>               //set_sys_clock_khz()

//-------------------------------------
//Bus Setup
//-------------------------------------
//uncomment one line only
SerialUART gps_Serial = SerialUART(uart1, HW_PIN_GPS_TX, HW_PIN_GPS_RX); //uart0 or uart1
//SerialPIO gps_Serial = SerialPIO(HW_PIN_GPS_TX, HW_PIN_GPS_RX, 32); //PIO uarts, any pin allowed

//uncomment one line only
SerialUART *rcin_Serial = new SerialUART(uart0, HW_PIN_RCIN_TX, HW_PIN_RCIN_RX); //uart0 or uart1
//SerialPIO *rcin_Serial = new SerialPIO(HW_PIN_RCIN_TX, HW_PIN_RCIN_RX, 32); //PIO uarts, any pin allowed

typedef TwoWire HW_WIRETYPE; //define the class to use for I2C
HW_WIRETYPE *i2c = &Wire; //&Wire or &Wire1

SPIClassRP2040 *spi = new SPIClassRP2040(spi0, HW_PIN_SPI_MISO, HW_PIN_IMU_CS, HW_PIN_SPI_SCLK, HW_PIN_SPI_MOSI); //spi0 or spi1
SPIClassRP2040 *bb_spi = new SPIClassRP2040(spi1, HW_PIN_SPI2_MISO, HW_PIN_BB_CS, HW_PIN_SPI2_SCLK, HW_PIN_SPI2_MOSI); //spi0 or spi1

#endif //#ifndef HW_BOARD_NAME

//======================================================================================================================//
//                    hw_setup()
//======================================================================================================================//

#define HW_USE_FREERTOS //RP2040 optionally uses FreeRTOS
#define HW_RTOS_IMUTASK_PRIORITY 7 //IMU Interrupt task priority, higher number is higher priority. Max priority on RP2040 is 7

#ifdef HW_USE_FREERTOS
  #include <FreeRTOS.h>  //FreeRTOS
  #include <semphr.h>    //FreeRTOS
#endif

void hw_setup() 
{ 
  //Uncomment for overclocking, supposedly works up to 270 MHz.
  //set_sys_clock_khz(200000, true); 

  Serial.println("USE_HW_RP2040");
  
  //I2C
  i2c->setSDA(HW_PIN_I2C_SDA);
  i2c->setSCL(HW_PIN_I2C_SCL);
  i2c->setClock(1000000);
  i2c->begin();

  //SPI 
  spi->begin();
  bb_spi->begin();
}
