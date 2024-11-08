//This pin layout is optimized for Espressif ESP32 DevKitC 38 pin board, use "ESP32 Dev Module" as board in Arduino IDE
//see https://madflight.com for details

#define HW_BOARD_NAME "DEFAULT ESP32 BOARD" 
#define HW_MCU "ESP32" //ESP32 - Most pins can be assigned freely

//LED:
#define HW_PIN_LED                2 //Note: ESP32 DevKitC has no on-board LED
#define HW_LED_ON                 1 //0:low is on, 1:high is on

//IMU SPI:
#define HW_PIN_SPI_MOSI          21 //   defaults: VSPI 23, HSPI 13
#define HW_PIN_SPI_MISO          36 //VP defaults: VSPI 19, HSPI 12
#define HW_PIN_SPI_SCLK          19 //   defaults: VSPI 18, HSPI 14
#define HW_PIN_IMU_CS            18 //   defaults: VSPI  5, HSPI 15
#define HW_PIN_IMU_EXTI          39 // VN silkscreen label

//BARO/MAG I2C:
#define HW_PIN_I2C_SDA           23 //default: Wire 21
#define HW_PIN_I2C_SCL           22 //default: Wire 22

//Outputs:

#define HW_OUT_COUNT             11
#define HW_PIN_OUT_LIST {33,25,26,27,14,12,13,15,0,4,16} //for ESP32 it is recommended to use only pins 2,4,12-19,21-23,25-27,32-33 for motors/servos

//Serial Debug on tx0 (pin 1), rx0 (pin 3) connected to serial->USB converter

//RC Receiver:
#define HW_PIN_RCIN_RX           35 //also used as PPM input
#define HW_PIN_RCIN_TX           32

//GPS:
#define HW_PIN_GPS_RX            17
#define HW_PIN_GPS_TX             5

//Battery ADC
#define HW_PIN_BAT_V             34
//#define HW_PIN_BAT_I

//BlackBox SPI:
//#define HW_PIN_SPI2_MISO
//#define HW_PIN_SPI2_MOSI
//#define HW_PIN_SPI2_SCLK
//#define HW_PIN_BB_CS

//BlackBox SDCARD via MMC interface: (can only use specific pins on ESP32, and has strapping restrictions, read the docs first!)
//#define HW_PIN_SDMMC_DATA
//#define HW_PIN_SDMMC_CLK
//#define HW_PIN_SDMMC_CMD
