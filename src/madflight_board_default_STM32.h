//This pin layout for the Black Pill board is based on MATEK F411SE (MTKS-MATEKF411SE betaflight target)

#define HW_BOARD_NAME "DEFAULT STM32 BOARD - Black Pill STM32F411CEUx" 
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
#include "madflight/hw_STM32/STM32_PWM.h"    //Servo and onshot

//Bus Setup
HardwareSerial *rcin_Serial = new HardwareSerial(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);
HardwareSerial gps_Serial(HW_PIN_GPS_RX, HW_PIN_GPS_TX);
typedef TwoWire HW_WIRETYPE; //define the class to use for I2C
HW_WIRETYPE *i2c = &Wire; //&Wire or &Wire1
SPIClass *spi = &SPI;
SPIClass *bb_spi = new SPIClass(HW_PIN_SPI2_MOSI, HW_PIN_SPI2_MISO, HW_PIN_SPI2_SCLK); //do not define HW_PIN_BB_CS here
