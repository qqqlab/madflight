//This pin layout for the Black Pill board is based on MATEK F411SE (MTKS-MATEKF411SE betaflight target)
//see https://madflight.com for details

#define HW_BOARD_NAME "DEFAULT STM32 BOARD" 
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

//LED:
#define HW_PIN_LED             PC13
#define HW_LED_ON                 0 //0:low is on, 1:high is on

//IMU SPI:
#define HW_PIN_SPI_MOSI         PA6
#define HW_PIN_SPI_MISO         PA7
#define HW_PIN_SPI_SCLK         PA5
#define HW_PIN_IMU_CS           PA4
#define HW_PIN_IMU_EXTI        PB10

//BARO/MAG I2C:
#define HW_PIN_I2C_SDA          PB6
#define HW_PIN_I2C_SCL          PB7

//Outputs:
#define HW_OUT_COUNT              6
#define HW_PIN_OUT_LIST {PB2,PB5,PA8,PA9,PA10,PB8}

//Serial Debug on tx0 (pin 1), rx0 (pin 3) connected to serial->USB converter

//RC Receiver: (SERIAL1)
#define HW_PIN_RCIN_RX           PA3 //also used as PPM input
#define HW_PIN_RCIN_TX           PA2
#define HW_PIN_RCIN_INVERTER      -1

//GPS: (SERIAL3)
#define HW_PIN_GPS_RX            PB3
#define HW_PIN_GPS_TX           PA15
#define HW_PIN_GPS_INVERTER       -1

//Battery ADC
#define HW_PIN_BAT_V            PB0
#define HW_PIN_BAT_I            PB1

//BlackBox SPI:
//#define HW_PIN_SPI2_MISO
//#define HW_PIN_SPI2_MOSI
//#define HW_PIN_SPI2_SCLK
//#define HW_PIN_BB_CS
