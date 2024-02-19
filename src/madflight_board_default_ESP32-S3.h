//This pin layout is optimized for Espressif ESP32 DevKitC 38 pin board, use "ESP32 Dev Module" as board in Arduino IDE

#define HW_BOARD_NAME "DEFAULT ESP32-S3 BOARD - Espressif ESP32-S3-DevKitC-1 - 44 pin" 
#define HW_MCU "ESP32-S3" //ESP32-S3 - Most pins can be assigned freely

//NOTE: DON'T USE SAME PIN TWICE. All pins here get configured, even if they are not used. Set pin to -1 to disable.

//Serial Debug
//19,20=USB, 43,44=CP2102
//USB CDC On Boot "Enabled": Serial->USB, Serial0->UART, "Disabled": Serial->UART, Serial1->undefined

//LED:
const int HW_PIN_LED      = 38; //Note: ESP32S3 DevKitC has RGB LED on this pin
const int HW_LED_ON       =  1; //0:low is on, 1:high is on

//IMU SPI:
const int HW_PIN_SPI_MOSI = 11;
const int HW_PIN_SPI_MISO = 12;
const int HW_PIN_SPI_SCLK = 13;
const int HW_PIN_IMU_CS   = 10;
const int HW_PIN_IMU_EXTI = 14;

//BARO/MAG I2C:
const int HW_PIN_I2C_SDA  =  8;
const int HW_PIN_I2C_SCL  =  9;

//Outputs:
const int HW_OUT_COUNT    =  6;
const int HW_PIN_OUT[HW_OUT_COUNT] = {4,5,6,7,15,16}; 

//RC Receiver:
const int HW_PIN_RCIN_RX  = 18; //also used as PPM input
const int HW_PIN_RCIN_TX  = 17;

//GPS:
const int HW_PIN_GPS_RX   =  3;
const int HW_PIN_GPS_TX   = 46;

//Battery ADC
const int HW_PIN_BAT_V    = -1;
const int HW_PIN_BAT_I    = -1;

//BlackBox SPI:
const int HW_PIN_SPI2_MISO = -1;
const int HW_PIN_SPI2_MOSI = -1;
const int HW_PIN_SPI2_SCLK = -1;
const int HW_PIN_BB_CS     = -1;


/*--------------------------------------------------------------------------------------------------
  IMPORTANT
  
  ESP32 Wire has a bug in I2C which causes the bus to hang for 1 second after a failed read, which can 
  happen a couple times per minute. This makes Wire I2C for IMU not a real option... 
  See --> https://github.com/espressif/esp-idf/issues/4999

  Uncomment USE_ESP32_SOFTWIRE to use software I2C, but this does not work well with all sensors...
  
  So, until a better I2C solution is available: use an SPI IMU sensor on ESP32!!!!
----------------------------------------------------------------------------------------------------*/  
//#define USE_ESP32_SOFTWIRE //uncomment to use SoftWire instead of Wire

//-------------------------------------
//Include Libraries
//-------------------------------------
#ifdef USE_ESP32_SOFTWIRE
  #include "madflight/HW_ESP32/ESP32_SoftWire.h"
#else
  #include <Wire.h>
#endif
#include <SPI.h>                         //SPI communication
#include "madflight/hw_ESP32/ESP32_PWM.h"      //Servo and onshot

//-------------------------------------
//Bus Setup
//-------------------------------------
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define VSPI FSPI
#endif  
HardwareSerial *rcin_Serial = &Serial1; //&Serial1 or &Serial2 (&Serial is used for debugging)
HardwareSerial &gps_Serial = Serial2; //Serial1 or Serial2 (Serial is used for debugging)
SPIClass spi1 = SPIClass(HSPI); // VSPI or HSPI(default) - used for IMU
SPIClass spi2 = SPIClass(VSPI);  // VSPI(default) or HSPI - used for BB and other functions

#ifdef USE_ESP32_SOFTWIRE
  typedef SoftWire HW_WIRETYPE; //typedef to force IMU to use SoftWire
  typedef SoftWire TwoWire; //typedef to force BARO to use SoftWire
  HW_WIRETYPE *i2c = new HW_WIRETYPE();  //create a ESP32_SoftWire instance
#else
  typedef TwoWire HW_WIRETYPE; //typedef HW_WIRETYPE with the class to use for I2C
  HW_WIRETYPE *i2c = &Wire; //&Wire or &Wire1
#endif

SPIClass *spi = &spi1;
SPIClass *bb_spi = &spi2;
