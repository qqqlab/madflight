
//Note: this implementation does not check if frequency overwrites the frequency of previously started PWM instances

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

//Select one:
//#define TARGET_BLACKPILL 
//#define TARGET_MATEKF411SE //similar to black pill except LED and IMU_INT
#define TARGET_OMNIBUSF4 //F405

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
//generic code
//======================================================================================================================//

//#define HW_USE_FREERTOS //STM FreeRTOS not supported (yet), leave commented out

#include <Wire.h>                      //I2C communication
#include <SPI.h>                       //SPI communication
#include "src/hw_STM32/STM32_PWM.h"  //Servo and onshot

//======================================================================================================================//
//                    HARDWARE DEFINITION Black Pill STM32F411CEUx
//======================================================================================================================//
#if defined TARGET_BLACKPILL
//STM32F411CEUx - not all pin combinations are allowed, see datasheet
//This pin layout is based on MATEK F411SE (MTKS-MATEKF411SE betaflight target)

//Arduino F411 defines: -DSTM32F4xx -DARDUINO=10607 -DARDUINO_GENERIC_F411CEUX -DARDUINO_ARCH_STM32 -DBOARD_NAME="GENERIC_F411CEUX" -DVARIANT_H="variant_generic.h" -DSTM32F411xE -DUSBCON -DUSBD_VID=0 -DUSBD_PID=0 -DHAL_PCD_MODULE_ENABLED -DUSBD_USE_CDC -DHAL_UART_MODULE_ENABLED

//Arduino F411: Serial and Serial1 both map TX1/RX1 on pin A9/A10. 
//Arduino F411: Serial debug on USB Serial port (USB is on on PA11,PA12, shared with USART6)
//TX1:PA9,PA15,PB6
//RX1:PA10,PB3,PB7
//TX2:PA2
//RX2:PA3
//TX6:PA11
//RX6:PA12

//NOTE: DON'T USE SAME PIN TWICE. All pins here get configured, even if they are not used. Set pin to -1 to disable.

//LED:
const int HW_PIN_LED      = PC13;
#define LED_ON 0 //low = on

//Battery voltage divider:
const int HW_PIN_BAT_ADC  = PB0;
const int HW_PIN_BAT_CURR = PB1;

//GPS:
const int HW_PIN_GPS_RX   = PA3; //RX2
const int HW_PIN_GPS_TX   = PA2; //TX2
HardwareSerial gps_Serial(HW_PIN_GPS_RX, HW_PIN_GPS_TX);

//RC Receiver:
const int HW_PIN_RCIN_RX  = PB3; //this pin is also used as PPM input. RX1
const int HW_PIN_RCIN_TX  = PA15; //TX1
HardwareSerial *rcin_Serial = new HardwareSerial(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);

//IMU:
const int HW_PIN_IMU_INT  = PB10; //only used when USE_IMU_INTERRUPT is defined.

//I2C:
const int HW_PIN_I2C_SDA  = PB6;
const int HW_PIN_I2C_SCL  = PB7;
typedef TwoWire HW_WIRETYPE; //define the class to use for I2C
HW_WIRETYPE *i2c = &Wire; //&Wire or &Wire1

//SPI:
const int HW_PIN_SPI_MISO = PA6;
const int HW_PIN_SPI_CS   = PA4;
const int HW_PIN_SPI_SCLK = PA5;
const int HW_PIN_SPI_MOSI = PA7;
SPIClass *spi = &SPI;

//Outputs:
#define HW_OUT_COUNT 6
const int16_t HW_PIN_OUT[HW_OUT_COUNT] = {PB2,PB5,PA8,PA9,PA10,PB8}; //MATEKF411SE: resource MOTOR x yyy

//======================================================================================================================//
//                    HARDWARE DEFINITION for MATEKF411SE STM32F411CEUx
//======================================================================================================================//
#elif defined TARGET_MATEKF411SE
//STM32F411CEUx - not all pin combinations are allowed, see datasheet
//This pin layout is based on MATEK F411SE (MTKS-MATEKF411SE betaflight target)

//Arduino F411 defines: -DSTM32F4xx -DARDUINO=10607 -DARDUINO_GENERIC_F411CEUX -DARDUINO_ARCH_STM32 -DBOARD_NAME="GENERIC_F411CEUX" -DVARIANT_H="variant_generic.h" -DSTM32F411xE -DUSBCON -DUSBD_VID=0 -DUSBD_PID=0 -DHAL_PCD_MODULE_ENABLED -DUSBD_USE_CDC -DHAL_UART_MODULE_ENABLED

//Arduino F411: Serial and Serial1 both map TX1/RX1 on pin A9/A10. 
//Arduino F411: Serial debug on USB Serial port (USB is on on PA11,PA12, shared with USART6)
//TX1:PA9,PA15,PB6
//RX1:PA10,PB3,PB7
//TX2:PA2
//RX2:PA3
//TX6:PA11
//RX6:PA12

//NOTE: DON'T USE SAME PIN TWICE. All pins here get configured, even if they are not used. Set pin to -1 to disable.

//LED:
const int HW_PIN_LED      = PC13; //black pill: LED PC13, MATEKF411SE: resource LED_STRIP 1 B10
#define LED_ON 0 //low = on

//Battery voltage divider:
const int HW_PIN_BAT_ADC = PB0; //MATEKF411SE: resource ADC_BATT 1 B00
const int HW_PIN_BAT_CURR = PB1; //MATEKF411SE: resource ADC_CURR 1 B01

//GPS:
const int HW_PIN_GPS_RX   = PA3; //RX2, MATEKF411SE: resource SERIAL_RX 2 A03
const int HW_PIN_GPS_TX   = PA2; //TX2, MATEKF411SE: resource SERIAL_TX 2 A02
HardwareSerial gps_Serial(HW_PIN_GPS_RX, HW_PIN_GPS_TX);

//RC Receiver:
const int HW_PIN_RCIN_RX  = PB3; //this pin is also used as PPM input. RX1, MATEKF411SE: resource SERIAL_RX 1 B03
const int HW_PIN_RCIN_TX  = PA15; //TX1, MATEKF411SE: resource SERIAL_TX 1 A15
HardwareSerial *rcin_Serial = new HardwareSerial(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);

//IMU:
const int HW_PIN_IMU_INT  = PA14; //only used when USE_IMU_INTERRUPT is defined. MATEKF411SE: resource GYRO_EXTI 1 A14

//I2C:
const int HW_PIN_I2C_SDA  = PB6; //MATEKF411SE: resource I2C_SDA 1 B06
const int HW_PIN_I2C_SCL  = PB7; //MATEKF411SE: resource I2C_SCL 1 B07
typedef TwoWire HW_WIRETYPE; //define the class to use for I2C
HW_WIRETYPE *i2c = &Wire; //&Wire or &Wire1

//SPI:
const int HW_PIN_SPI_MISO = PA6; //MATEKF411SE: resource SPI_MISO 1 A06
const int HW_PIN_SPI_CS   = PA4; //MATEKF411SE: resource GYRO_CS 1 A04
const int HW_PIN_SPI_SCLK = PA5; //MATEKF411SE: resource SPI_SCK 1 A05
const int HW_PIN_SPI_MOSI = PA7; //MATEKF411SE: resource SPI_MOSI 1 A07
SPIClass *spi = &SPI;

//Outputs:
#define HW_OUT_COUNT 6
const int16_t HW_PIN_OUT[HW_OUT_COUNT] = {PB2,PB5,PA8,PA9,PA10,PB8}; //MATEKF411SE: resource MOTOR x yyy

//======================================================================================================================//
//                    HARDWARE DEFINITION for OMNIBUSF4 STM32F405
//======================================================================================================================//
#elif defined TARGET_OMNIBUSF4
//STM32F405RGTx - not all pin combinations are allowed, see datasheet
//This pin layout is based on AIRB OMNIBUSF4 (AIRB-OMNIBUSF4 betaflight target)

//Arduino F411 defines: -DSTM32F4xx -DARDUINO=10607 -DARDUINO_GENERIC_F405RGTX -DARDUINO_ARCH_STM32 -DBOARD_NAME="GENERIC_F405RGTX" -DVARIANT_H="variant_generic.h" -DSTM32F405xx -DUSBCON -DUSBD_VID=0 -DUSBD_PID=0 -DHAL_PCD_MODULE_ENABLED -DUSBD_USE_CDC -DHAL_UART_MODULE_ENABLED

//#ifndef ARDUINO_GENERIC_F405RGTX
//#ifndef STM32F405xx
// #error "Wrong mcu. Select Tools->Board part number: Generic F405RGTx"
//#endif

//LED:
//const int HW_PIN_LED      = PB5; //blue LED
//#define LED_ON 0 //low = blue on
const int HW_PIN_LED      = PC10; //green LED
#define LED_ON 1 //high = green on

//Battery voltage divider:
const int HW_PIN_BAT_ADC  = PC2;
const int HW_PIN_BAT_CURR = PC1;

//GPS:
const int HW_PIN_GPS_RX   = PB11; //RX3
const int HW_PIN_GPS_TX   = PB10; //TX3

//RC Receiver:
const int HW_PIN_RCIN_PPM = PB14;
const int HW_PIN_RCIN_RX  = PA10; //this pin is also used as PPM input. RX1
const int HW_PIN_RCIN_TX  = PA9; //TX1

//IMU:
const int HW_PIN_IMU_INT  = PC4; //only used when USE_IMU_INTERRUPT is defined.

//I2C:
const int HW_PIN_I2C_SDA  = PB9;
const int HW_PIN_I2C_SCL  = PB8;

//SPI:
const int HW_PIN_SPI_MISO = PA6;
const int HW_PIN_SPI_CS   = PA4;
const int HW_PIN_SPI_SCLK = PA5;
const int HW_PIN_SPI_MOSI = PA7;

//Outputs:
#define HW_OUT_COUNT 6
const int16_t HW_PIN_OUT[HW_OUT_COUNT] = {PB0,PB1,PA3,PA2,PA1,PA8};

//======================================================================================================================//
// Other targets
//======================================================================================================================//
#else
  #error "TARGET_XXX not defined or unknown target."
#endif

//======================================================================================================================//
//generic
//======================================================================================================================//
HardwareSerial gps_Serial(HW_PIN_GPS_RX, HW_PIN_GPS_TX);
HardwareSerial *rcin_Serial = new HardwareSerial(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);
typedef TwoWire HW_WIRETYPE; //define the class to use for I2C
HW_WIRETYPE *i2c = &Wire; //&Wire or &Wire1
SPIClass *spi = &SPI;

void hw_setup() 
{ 
  Serial.println("USE_HW_STM32");
  Serial.println("BOARD_NAME=" BOARD_NAME);  
  
  //I2C
  i2c->setSDA(HW_PIN_I2C_SDA);
  i2c->setSCL(HW_PIN_I2C_SCL);
  i2c->setClock(1000000); //Note: this is 2.5 times the MPU6050/MPU9150 spec sheet 400 kHz max...
  i2c->begin();

  //SPI 
  spi->setMISO(HW_PIN_SPI_MISO);
  spi->setSCLK(HW_PIN_SPI_SCLK);
  spi->setMOSI(HW_PIN_SPI_MOSI);
  //spi->setSSEL(HW_PIN_SPI_CS); //don't set CS here, it is done in the driver to be compatible with other hardware platforms
  spi->begin();
}


