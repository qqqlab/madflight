/*==============================================================================
2023-12-29 Flown a "DYS F4 Pro V2" quad with these settings

Board name: DYSF4PROV2
Manufacturer ID: DYST

copy this line to madflight.ino to use this flight controller:
#include "boards/madflight/DYST-DYSF4PRO_V2.h"


Top Connector
-------------
PC3           PB4      PB9 PB14
RSSI  9V  5V  BUZ+ 3V3 SDA PPM
 S5  GND  GND BUZ- GND SCL S6
PA1                        PA8


Left Connector
--------------
PA10 RX1 - TX1 PA9
     GND - 5V
 PC6 TX6 - RX6 PC7


Right Connector
---------------

     GND - BAT+
PB11 RX3 - TX3 PB10
 PC1 CUR - LED PA1
  VidOut - VidIn
      5V - 5V
     GND - GND


set BAT_CAL_V 0.00057630
set BAT_CAL_I 0.0011806
cwrite

==============================================================================*/

#define HW_BOARD_NAME "MADFLIGHT-DYST-DYSF4PRO_V2"
#define HW_MCU "STM32F405"

#undef BB_USE
#undef IMU_USE
#undef BAT_USE
#undef IMU_ALIGN

#define BB_USE   BB_USE_FLASH
#define IMU_USE  IMU_USE_SPI_MPU6000
#define BAT_USE  BAT_USE_ADC

//unused bf defines
//#define USE_MAX7456
//#define USE_FLASH_W25Q128FV

//Sensor specific setup
#define IMU_ALIGN  IMU_ALIGN_CW180
//#define BARO_I2C_ADR  0
//#define MAG_I2C_ADR  0

//Blue LED:
//const int HW_PIN_LED           = PB5;
//const int HW_LED_ON            = 0; //0:low is on, 1:high is on

//Green LED:
const int HW_PIN_LED           = PC10; // == SPI3_SCK for MAX7456, FlASH
const int HW_LED_ON            = 1; //0:low is on, 1:high is on

//IMU SPI: (SPI1)
const int HW_PIN_SPI_INDEX     = 1;
const int HW_PIN_SPI_SCLK      = PA5;
const int HW_PIN_SPI_MISO      = PA6;
const int HW_PIN_SPI_MOSI      = PA7;
const int HW_PIN_IMU_CS        = PA4;
const int HW_PIN_IMU_EXTI      = PC4;

//BARO/MAG I2C: (I2C2) Note: pins SCL/SDA are swapped in board documentation
const int HW_PIN_I2C_SDA       = PB9;
const int HW_PIN_I2C_SCL       = PB8;

//Outputs:
const int HW_OUT_COUNT         = 6;
const int HW_PIN_OUT[]         = {PB0,PB1,PA3,PA2,PA1,PA8};

//RC Receiver: (SERIAL1)
const int HW_PIN_RCIN_RX       = PA10;
const int HW_PIN_RCIN_TX       = PA9;
const int HW_PIN_RCIN_INVERTER = PC0;

//GPS: (SERIAL3) Note: pins RX3/TX3 are swapped in board documentation
const int HW_PIN_GPS_RX       = PB11;
const int HW_PIN_GPS_TX       = PB10;
const int HW_PIN_GPS_INVERTER = -1;

//Battery ADC voltage and current inputs:
const int HW_PIN_BAT_V        = PC2;
const int HW_PIN_BAT_I        = PC1;

//Black Box
const int HW_PIN_SPI2_INDEX   = 3;
const int HW_PIN_SPI2_SCLK    = PC10;
const int HW_PIN_SPI2_MISO    = PC11;
const int HW_PIN_SPI2_MOSI    = PC12;
const int HW_PIN_BB_CS        = PB3;

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
SPIClass *bb_spi = new SPIClass(HW_PIN_SPI2_MOSI, HW_PIN_SPI2_MISO, HW_PIN_SPI2_SCLK); //do not define HW_PIN_BB_CS here

//Serial
#define HW_SERIAL_COUNT 3
#define HW_PIN_SERIAL { {1,PA9,PA10,PC0}, {3,PB10,PB11,-1}, {6,PC6,PC7,-1} } // {INDEX,TX,RX,INVERTER}

//SPI spi1=imu, spi3=flash,osd
#define HW_SPI_COUNT 2
#define HW_PIN_SPI { {1,PA5,PA6,PA7}, {3,PC10,PC11,PC12} } // {INDEX,SCK,MISO,MOSI}

//I2C
#define HW_I2C_COUNT 0
#define HW_PIN_I2C {  } // {INDEX,SCL,SDA}

//Motors:
#define HW_MOTOR_COUNT 6
#define HW_MOTOR_OUT {PB0,PB1,PA3,PA2,PA1,PA8}

//other pins
#define HW_PIN_BEEPER      PB4
#define HW_PIN_PPM         PB14
#define HW_PIN_ADC_RSSI    PC3
#define HW_PIN_LED_STRIP   PA1
#define HW_PIN_FLASH_CS    PB3
#define HW_PIN_OSD_CS     PA15
