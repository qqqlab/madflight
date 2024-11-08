//This pin layout is optimized for Raspberry Pi Pico/Pico2 board: UART, PWM on side; I2C, SPI, PPM on the other side
//see https://madflight.com for details

#define HW_BOARD_NAME "DEFAULT RP2350/RP2040 BOARD"
#define HW_MCU "RP2350/RP2040" //RP2350/RP2040 - not all pin combinations are allowed, see datasheet

//NOTE: edit hw_RP2040.h to change uart/spi/i2c ports

//LED:
#define HW_PIN_LED               25 //internal on Raspberry Pi Pico
#define HW_LED_ON                 1 //0:low is on, 1:high is on

//IMU SPI on spi0:
#define HW_PIN_SPI_MISO          16 //spi0: 0, 4, 16(default)
#define HW_PIN_SPI_MOSI          19 //spi0: 3, 7, 19(default)
#define HW_PIN_SPI_SCLK          18 //spi0: 2, 6, 18(default)
#define HW_PIN_IMU_CS            17 //spi0: 1, 5, 17(default)
#define HW_PIN_IMU_EXTI          22

//BARO/MAG I2C on i2c0:
#define HW_PIN_I2C_SDA           20 //i2c0: 0, 4(default), 8, 12, 16, 20   i2c1: 2, 6, 10, 14, 18, 26(default)
#define HW_PIN_I2C_SCL           21 //i2c0: 1, 5(default), 9, 13, 17, 21   i2c1: 3, 7, 11, 15, 19, 27(default)

//Outputs:
#define HW_OUT_COUNT              8
#define HW_PIN_OUT_LIST {2,3,4,5,6,7,10,11}

//Serial debug on USB Serial port (no GPIO pins)

//RC Receiver on uart0:
#define HW_PIN_RCIN_RX            1 //uart0: 1(default), 5, 13, 17  - this pin is also used as PPM input
#define HW_PIN_RCIN_TX            0 //uart0: 0(default), 4, 12, 16

//GPS on uart1:
#define HW_PIN_GPS_RX             9 //uart1: 5, 9(default)
#define HW_PIN_GPS_TX             8 //uart1: 4, 8(default)

//Battery ADC
#define HW_PIN_BAT_V             28 //pin A2
//#define HW_PIN_BAT_I

//BlackBox SPI on spi1:
#define HW_PIN_SPI2_MISO         12 //spi1:  8, 12(default)
#define HW_PIN_SPI2_MOSI         15 //spi1: 11, 15(default)
#define HW_PIN_SPI2_SCLK         14 //spi1: 10, 14(default)
#define HW_PIN_BB_CS             13 //spi1:  9, 13(default)
