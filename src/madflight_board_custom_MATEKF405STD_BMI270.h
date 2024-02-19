/*==============================================================================
2023-12-31 Tested a "Clone MatekF405STD" flight controller with these settings

Based on betaflight: CLNE-MATEKF405STD_CLONE

//copy this line to madflight.ino to use this flight controller (or copy/paste the whole file)
#include "boards/madflight/MATEKF405STD_BMI270.h"
==============================================================================*/

#define HW_BOARD_NAME "MADFLIGHT-MATEKF405STD_BMI270"
#define HW_MCU "STM32F405"

//#define USE_IMU_SPI_MPU6000
//#define USE_IMU_SPI_MPU6500
//#define USE_IMU_SPI_ICM20689
#define USE_IMU_SPI_BMI270
//#define USE_BARO_BMP280
//#define USE_MAX7456
//#define USE_SDCARD

//Sensor specific setup
#define IMU_ROTATE_CW180
#define BARO_I2C_ADR 0
#define MAG_I2C_ADR 0

//LED:
const int HW_PIN_LED      = PB9;
const int HW_LED_ON       = 0; //0:low is on, 1:high is on

//IMU SPI: (SPI1)
const int HW_PIN_SPI_MISO = PA6;
const int HW_PIN_SPI_MOSI = PA7;
const int HW_PIN_SPI_SCLK = PA5;
const int HW_PIN_IMU_CS   = PC2;
const int HW_PIN_IMU_EXTI = PC3;

//BARO/MAG I2C: (I2C1)
const int HW_PIN_I2C_SDA  = PB6;
const int HW_PIN_I2C_SCL  = PB7;

//Outputs:
const int HW_OUT_COUNT    = 6;
const int HW_PIN_OUT[]    = {PC6,PC7,PC8,PC9,PA15,PA8};

//RC Receiver: (SERIAL1)
const int HW_PIN_RCIN_RX  = PA10;
const int HW_PIN_RCIN_TX  = PA9;
const int HW_PIN_RCIN_INVERTER = -1;

//GPS: (SERIAL2)
const int HW_PIN_GPS_RX   = PA3;
const int HW_PIN_GPS_TX   = PA2;
const int HW_PIN_GPS_INVERTER = -1;

//Battery ADC voltage and current inputs:
const int HW_PIN_BAT_V    = PC5;
const int HW_PIN_BAT_I    = PC4;

//Include Libraries
#include <Wire.h>                      //I2C communication
#include <SPI.h>                       //SPI communication
#include "src/hw_STM32/STM32_PWM.h"    //Servo and oneshot

//Bus Setup
HardwareSerial *rcin_Serial = new HardwareSerial(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);
HardwareSerial gps_Serial(HW_PIN_GPS_RX, HW_PIN_GPS_TX);
typedef TwoWire HW_WIRETYPE; //define the class to use for I2C
HW_WIRETYPE *i2c = &Wire; //&Wire or &Wire1
SPIClass *spi = &SPI;

//Serial
#define HW_SERIAL_COUNT 5
#define HW_PIN_SERIAL { {1,PA9,PA10,-1}, {2,PA2,PA3,-1}, {3,PC10,PC11,-1}, {4,PA0,PA1,-1}, {5,PC12,PD2,-1} } // {INDEX,TX,RX,INVERTER}

//SPI
#define HW_SPI_COUNT 3
#define HW_PIN_SPI { {1,PA5,PA6,PA7}, {2,PB13,PB14,PB15}, {3,PB3,PB4,PB5} } // {INDEX,SCK,MISO,MOSI}

//I2C
#define HW_I2C_COUNT 1
#define HW_PIN_I2C { {1,PB6,PB7} } // {INDEX,SCL,SDA}

//Motors:
#define HW_MOTOR_COUNT 6
#define HW_MOTOR_OUT {PC6,PC7,PC8,PC9,PA15,PA8}

//other pins
#define HW_PIN_BEEPER PC13
#define HW_PIN_PPM PA3
#define HW_PIN_LED_STRIP PB6
#define HW_PIN_ADC_RSSI PB1
#define HW_PIN_SDCARD_CS PC1
#define HW_PIN_OSD_CS PB10
#define HW_PIN_USB_DETECT PB12

//set statements
#define HW_SET_BEEPER_INVERSION ON
#define HW_SET_BEEPER_OD OFF
#define HW_SET_SDCARD_SPI_BUS 3
#define HW_SET_MAX7456_SPI_BUS 2
#define HW_SET_GYRO_1_SPIBUS 1
