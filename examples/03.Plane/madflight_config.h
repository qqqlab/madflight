//========================================================================================================================//
//                                               HARDWARE CONFIG                                                          //
//========================================================================================================================//

//--- RC RECEIVER
#define RCIN_USE  RCIN_USE_CRSF // Select one: RCIN_USE_MAVLINK, RCIN_USE_CRSF, RCIN_USE_SBUS, RCIN_USE_DSM, RCIN_USE_PPM, RCIN_USE_PWM
#define RCIN_NUM_CHANNELS 8 // Number of channels
#define RCIN_STICK_DEADBAND 0 // Deadband for centering sticksD: pwm center-deadband to pwm center+deadband will be interpreted as central stick. Set to 15 for PPM or 0 for jitter-free serial protocol receivers.

//--- IMU SENSOR
// IMU_ALIGN is the sensor orientation. The label is yaw / roll (in that order) needed to rotate the sensor from it's normal position to it's mounted position.
// If not sure what is needed: use CLI 'pahrs' and try each setting until roll-right gives positive ahrs.roll, pitch-up gives positive ahrs.pitch, and yaw-right gives positive ahrs.yaw
#define IMU_USE  IMU_USE_SPI_MPU6500 // Select one: IMU_USE_SPI_MPU6500, IMU_USE_SPI_MPU9250, IMU_USE_SPI_MPU6000, IMU_USE_SPI_BMI270, IMU_USE_I2C_MPU9250, IMU_USE_I2C_MPU9150, IMU_USE_I2C_MPU6500, IMU_USE_I2C_MPU6050, IMU_USE_I2C_MPU6000
#define IMU_ALIGN  IMU_ALIGN_CW90 // Select one: IMU_ALIGN_CW0, IMU_ALIGN_CW90, IMU_ALIGN_CW180, IMU_ALIGN_CW270, IMU_ALIGN_CW0FLIP, IMU_ALIGN_CW90FLIP, IMU_ALIGN_CW180FLIP, IMU_ALIGN_CW270FLIP
#define IMU_I2C_ADR  0x69 // IMU I2C address. If unknown, use CLI 'i2c'

//-- AHRS sensor fusion 
#define AHRS_USE AHRS_USE_MAHONY // Select one: AHRS_USE_MAHONY, AHRS_USE_MAHONY_BF, AHRS_USE_MADGWICK, AHRS_USE_VQF

//--- GPS
#define GPS_BAUD 115200

//--- BAROMETER SENSOR
#define BARO_USE  BARO_USE_NONE // Select one: BARO_USE_BMP390, BARO_USE_BMP388, BARO_USE_BMP280, BARO_USE_MS5611, BARO_USE_NONE
//#define BARO_I2C_ADR  0x76 // Set barometer I2C address, leave commented for default address. If unknown, use CLI 'i2c'

//--- EXTERNAL MAGNETOMETER SENSOR
#define MAG_USE  MAG_USE_NONE // Select one: MAG_USE_QMC5883L, MAG_USE_NONE
//#define MAG_I2C_ADR  0x77 // Set magnetometer I2C address, leave commented for default address. If unknown, use CLI 'i2c'

//--- BATTERY MONITOR
#define BAT_USE  BAT_USE_NONE // Select one: BAT_USE_INA226, BAT_USE_INA228, BAT_USE_ADC, BAT_USE_NONE

//--- BLACKBOX LOGGER
#define BB_USE  BB_USE_NONE // Select one: BB_USE_SD, BB_USE_SDMMC, BB_USE_NONE

//========================================================================================================================//
//                                               MISC CONFIG                                                              //
//========================================================================================================================//

#define IMU_SAMPLE_RATE 1000 //imu sample rate in Hz (default 1000) NOTE: not all IMU drivers support a different rate
#define BARO_SAMPLE_RATE 100 //baro sample rate in Hz (default 100)

//Low Pass Filter cutoff frequency in Hz. Do not touch unless you know what you are doing.
#define IMU_ACC_LP_HZ 70        //Accelerometer  (default 70Hz)
#define IMU_GYR_LP_HZ 60        //Gyro           (default 60Hz)
#define IMU_MAG_LP_HZ 1e10      //Magnetometer   (default 1e10Hz, i.e. no filtering)

//========================================================================================================================//
//                                               PINS CONFIG                                                              //
//========================================================================================================================//
//
// You have 3 options to setup the pins (gpio numbers) for the flight controller:
//
//   1) Default - Leave this section as is and see https://madflight.com for default pinout diagrams for the supported
//      processor families. Default pinouts are defined in the board header files library/src/madflight_board_default_XXX.h
// 
//   2) Header - #include the BetaFlight flight controller you want to use. See library/madflight/src for all available 
//      boards. For example: #include <madflight_board_betaflight_MTKS-MATEKH743.h>
// 
//   3) Custom - Remove /* below to enable the CUSTOM PINS section, and define own pinout.
//
//========================================================================================================================//

/* <-- remove this to enable custom pins
//========================================================================================================================//
//                                               CUSTOM PINS CONFIG                                                       //
//========================================================================================================================//

#define HW_BOARD_NAME "My Custom Board" //REQUIRED: Give your board a name - without a name the default pinout is loaded!!!

//Replace 'pp' with the gpio number you want to use, or comment out the #define if the pin is not used
//NOTE: Not all pins can be freely configured. Read the processor datasheet, or use the default pinout.

//LED:
#define HW_PIN_LED        pp
#define HW_LED_ON          0 //0:low is on, 1:high is on

//IMU SPI:
#define HW_PIN_SPI_MISO   pp
#define HW_PIN_SPI_MOSI   pp
#define HW_PIN_SPI_SCLK   pp
#define HW_PIN_IMU_CS     pp
#define HW_PIN_IMU_EXTI   pp //REQUIRED: IMU external interrupt pin (required for SPI and I2C sensors)

//I2C for BARO, MAG, BAT sensors (and for IMU if not using SPI IMU)
#define HW_PIN_I2C_SDA    pp
#define HW_PIN_I2C_SCL    pp

//Motor/Servo Outputs:
#define HW_OUT_COUNT      4 //number of outputs
#define HW_PIN_OUT_LIST   {pp,pp,pp,pp} //list of output pins, enter exactly HW_OUT_COUNT pins.

//Serial debug on USB Serial port (no GPIO pins)

//RC Receiver:
#define HW_PIN_RCIN_RX    pp
#define HW_PIN_RCIN_TX    pp
#define HW_PIN_RCIN_INVERTER pp //only used for STM32 targets

//GPS:
#define HW_PIN_GPS_RX     pp 
#define HW_PIN_GPS_TX     pp
#define HW_PIN_GPS_INVERTER pp //only used for STM32 targets

//Battery ADC
#define HW_PIN_BAT_V      pp
#define HW_PIN_BAT_I      pp

//Black Box SPI (for sdcard or external flash chip):
#define HW_PIN_SPI2_MISO  pp
#define HW_PIN_SPI2_MOSI  pp
#define HW_PIN_SPI2_SCLK  pp
#define HW_PIN_BB_CS      pp

//Black Box SDCARD via MMC interface:
#define HW_PIN_SDMMC_DATA pp
#define HW_PIN_SDMMC_CLK  pp
#define HW_PIN_SDMMC_CMD  pp
//*/

//RP2040 specific options
//#define HW_RP2040_SYS_CLK_KHZ 200000 //overclocking

//ESP32 specific options
//#define USE_ESP32_SOFTWIRE //use bitbang I2C (not hardware I2C) See https://github.com/espressif/esp-idf/issues/499

