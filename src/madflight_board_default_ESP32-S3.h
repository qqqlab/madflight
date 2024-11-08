//This pin layout is optimized for Espressif ESP32-S3-DevKitC-1 (44 pin) board, use "ESP32-S3 Dev Module" as board in Arduino IDE
//see https://madflight.com for details

#define HW_BOARD_NAME "DEFAULT ESP32-S3 BOARD" 
#define HW_MCU "ESP32-S3" //ESP32-S3 - Most pins can be assigned freely

//LED:
#define HW_PIN_LED                2
#define HW_LED_ON                 1 //0:low is on, 1:high is on

//IMU SPI:
#define HW_PIN_SPI_MOSI          11
#define HW_PIN_SPI_MISO          12
#define HW_PIN_SPI_SCLK          13
#define HW_PIN_IMU_CS            10
#define HW_PIN_IMU_EXTI          14

//BARO/MAG I2C:
#define HW_PIN_I2C_SDA            8
#define HW_PIN_I2C_SCL            9

//Outputs:

#define HW_OUT_COUNT             6
#define HW_PIN_OUT_LIST {4,5,6,7,15,16}

//Serial Debug on tx0 (pin 1), rx0 (pin 3) connected to serial->USB converter

//RC Receiver:
#define HW_PIN_RCIN_RX           18 //also used as PPM input
#define HW_PIN_RCIN_TX           17

//GPS:
#define HW_PIN_GPS_RX             3
#define HW_PIN_GPS_TX            46

//Battery ADC
//#define HW_PIN_BAT_V
//#define HW_PIN_BAT_I

//BlackBox SPI:
//#define HW_PIN_SPI2_MISO
//#define HW_PIN_SPI2_MOSI
//#define HW_PIN_SPI2_SCLK
//#define HW_PIN_BB_CS

//BlackBox SDCARD via MMC interface: (can use any pin on ESP32-S3)
#define HW_PIN_SDMMC_DATA        40
#define HW_PIN_SDMMC_CLK         39
#define HW_PIN_SDMMC_CMD         38
