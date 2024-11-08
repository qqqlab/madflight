//This pin layout is optimized for Raspberry Pi Pico board: UART, PWM on side; I2C, SPI, PPM on the other side

#define HW_BOARD_NAME "RP2040-Zero"
#define HW_MCU "RP2040" //RP2040 - not all pin combinations are allowed, see datasheet

//-------------------------------------
// PIN DEFINITIONS
//-------------------------------------

//LED:
//#define HW_PIN_LED                //16 is internal RGB LED on RP2040-Zero
#define HW_LED_ON                 1 //0:low is on, 1:high is on

//IMU SPI:
#define HW_PIN_SPI_MOSI          12 //spi0: 0, 4, 16(default)   spi1:  8, 12(default)
#define HW_PIN_SPI_MISO          11 //spi0: 3, 7, 19(default)   spi1: 11, 15(default)
#define HW_PIN_SPI_SCLK          10 //spi0: 2, 6, 18(default)   spi1: 10, 14(default)
#define HW_PIN_IMU_CS            13 //spi0: 1, 5, 17(default)   spi1:  9, 13(default)
#define HW_PIN_IMU_EXTI          15

//BARO/MAG I2C:
#define HW_PIN_I2C_SDA           26 //Wire: 0, 4(default), 8, 12, 16, 20   Wire1: 2, 6, 10, 14, 18, 26(default)
#define HW_PIN_I2C_SCL           27 //Wire: 1, 5(default), 9, 13, 17, 21   Wire1: 3, 7, 11, 15, 19, 27(default)

//Outputs:
#define HW_OUT_COUNT             6
#define HW_PIN_OUT_LIST {2,3,4,5,6,7}

//Serial debug on USB Serial port (no GPIO pins)

//RC Receiver:
#define HW_PIN_RCIN_RX            1 //uart0: 1(default), 5, 13, 17   uart1: 5, 9(default) , this pin is also used as PPM input
#define HW_PIN_RCIN_TX            0 //uart0: 0(default), 4, 12, 16   uart1: 4, 8(default)

//GPS:
#define HW_PIN_GPS_RX             9 //uart0: 1(default), 5, 13, 17   uart1: 5, 9(default)
#define HW_PIN_GPS_TX             8 //uart0: 0(default), 4, 12, 16   uart1: 4, 8(default)

//Battery ADC
#define HW_PIN_BAT_V             28 //pin A2
//#define HW_PIN_BAT_I

//BlackBox SPI:
//#define HW_PIN_SPI2_MISO
//#define HW_PIN_SPI2_MOSI
//#define HW_PIN_SPI2_SCLK
//#define HW_PIN_BB_CS

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

// GPS Serial - uncomment one: SerialUART, SerialIRQ or SerialPIO and use uart0 or uart1
uint8_t gps_txbuf[256];
uint8_t gps_rxbuf[256];
SerialIRQ gps_Serial(uart1, HW_PIN_GPS_TX, gps_txbuf, sizeof(gps_txbuf), HW_PIN_GPS_RX, gps_rxbuf, sizeof(gps_rxbuf)); //SerialIRQ uses buffered tx and rx in addition to the 32 byte uart hardware fifo
//SerialUART gps_Serial = SerialUART(uart1, HW_PIN_GPS_TX, HW_PIN_GPS_RX); //SerialUART default Arduino impementation (had some problems with this)
//SerialPIO gps_Serial = SerialPIO(HW_PIN_GPS_TX, HW_PIN_GPS_RX, 32); //PIO uarts, any pin allowed (not tested)

// RC Input Serial - uncomment one: SerialUART, SerialIRQ or SerialPIO and use uart0 or uart1
uint8_t rcin_txbuf[256];
uint8_t rcin_rxbuf[1024];
SerialIRQ *rcin_Serial = new SerialIRQ(uart0, HW_PIN_RCIN_TX, rcin_txbuf, sizeof(rcin_txbuf), HW_PIN_RCIN_RX, rcin_rxbuf, sizeof(rcin_rxbuf)); //SerialIRQ uses buffered tx and rx in addition to the 32 byte uart hardware fifo
//SerialUART rcin_Serial = SerialUART(uart0, HW_PIN_RCIN_TX, HW_PIN_RCIN_RX); //SerialUART default Arduino impementation (had some problems with this)
//SerialPIO *rcin_Serial = new SerialPIO(HW_PIN_RCIN_TX, HW_PIN_RCIN_RX, 32); //PIO uarts, any pin allowed (not tested)

typedef TwoWire HW_WIRETYPE; //define the class to use for I2C
HW_WIRETYPE *i2c = &Wire1; //&Wire or &Wire1

SPIClassRP2040 *spi = new SPIClassRP2040(spi1, HW_PIN_SPI_MISO, HW_PIN_IMU_CS, HW_PIN_SPI_SCLK, HW_PIN_SPI_MOSI); //spi0 or spi1
SPIClassRP2040 *bb_spi = new SPIClassRP2040(spi0, HW_PIN_SPI2_MISO, HW_PIN_BB_CS, HW_PIN_SPI2_SCLK, HW_PIN_SPI2_MOSI); //spi0 or spi1
