/*==============================================================================
Generated on: 2024-10-23 18:06:29.311168
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: SPRACINGH7EXTREME
Manufacturer ID: SPRO

//copy this line to madflight.ino to use this flight controller (or copy/paste the whole file)
#include <madflight_board_betaflight_SPRO-SPRACINGH7EXTREME.h>
==============================================================================*/

#define HW_BOARD_NAME "BETAFLIGHT-SPRO-SPRACINGH7EXTREME"
#define HW_MCU "STM32H750"

//Defines from betaflight. Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out.
#define TARGET_BOARD_IDENTIFIER  "SP7E"
#define USBD_PRODUCT_STRING  "SPRacingH7EXTREME"
#define FC_VMA_ADDRESS  0x97CE0000
#define EEPROM_SIZE  8192
#define USE_SPRACING_PERSISTENT_RTC_WORKAROUND
#define USE_BUTTONS
#define BUTTON_A_PIN  PE4
#define BUTTON_A_PIN_INVERTED
#define BUTTON_B_PIN  PE4
#define BUTTON_B_PIN_INVERTED
#define USE_QUADSPI
#define USE_QUADSPI_DEVICE_1
#define QUADSPI1_SCK_PIN  PB2
#define QUADSPI1_BK1_IO0_PIN  PD11
#define QUADSPI1_BK1_IO1_PIN  PD12
#define QUADSPI1_BK1_IO2_PIN  PE2
#define QUADSPI1_BK1_IO3_PIN  PD13
#define QUADSPI1_BK1_CS_PIN  PB10
#define QUADSPI1_BK2_IO0_PIN  PE7
#define QUADSPI1_BK2_IO1_PIN  PE8
#define QUADSPI1_BK2_IO2_PIN  PE9
#define QUADSPI1_BK2_IO3_PIN  PE10
#define QUADSPI1_BK2_CS_PIN  NONE
#define QUADSPI1_MODE  QUADSPI_MODE_BK1_ONLY
#define QUADSPI1_CS_FLAGS  (QUADSPI_BK1_CS_HARDWARE
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
#define FLASH_QUADSPI_INSTANCE  QUADSPI
#define CONFIG_IN_EXTERNAL_FLASH
#define SDCARD_DETECT_PIN  PD10
#define SDCARD_DETECT_INVERTED
#define SDIO_DEVICE  SDIODEV_1
#define SDIO_USE_4BIT  1
#define SDIO_CK_PIN  PC12
#define SDIO_CMD_PIN  PD2
#define SDIO_D0_PIN  PC8
#define SDIO_D1_PIN  PC9
#define SDIO_D2_PIN  PC10
#define SDIO_D3_PIN  PC11
#define USE_SPI
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN  PD3
#define SPI2_MISO_PIN  PC2
#define SPI2_MOSI_PIN  PC3
#define SPI2_NSS_PIN  PB12
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN  PB3
#define SPI3_MISO_PIN  PB4
#define SPI3_MOSI_PIN  PD6
#define SPI3_NSS_PIN  PA15
#define USE_SPI_DEVICE_4
#define SPI4_SCK_PIN  PE12
#define SPI4_MISO_PIN  PE13
#define SPI4_MOSI_PIN  PE14
#define SPI4_NSS_PIN  PE11
#define USE_USB_ID
#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL_PIN  PB8
#define I2C1_SDA_PIN  PB9
#define I2C_DEVICE  (I2CDEV_1)
#define ENSURE_MPU_DATA_READY_IS_LOW
#define USE_PID_AUDIO
#define VTX_RTC6705_OPTIONAL
#define ADC1_DMA_OPT  8
#define ADC3_DMA_OPT  9
#define IMU_USE  IMU_USE_SPI_MPU6500
#define BARO_USE  BARO_USE_BMP388
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define BB_USE  BB_USE_FLASH
#define BB_FLASH_TYPE  "W25N01G"
#define BB_USE  BB_USE_SDCARD
#define USE_CAMERA_CONTROL
#define OSD_USE  OSD_USE_MAX7456

//Sensor specific setup
#define IMU_ALIGN  IMU_ALIGN_CW180
#define BARO_I2C_ADR  0
#define MAG_I2C_ADR  0

//LED:
#define HW_PIN_LED       PE3
#define HW_LED_ON        1 //0:low is on, 1:high is on

//IMU SPI: (SPI3)
#define HW_PIN_SPI_MISO  -1
#define HW_PIN_SPI_MOSI  -1
#define HW_PIN_SPI_SCLK  -1
#define HW_PIN_IMU_CS    PA15
#define HW_PIN_IMU_EXTI  PD4

//BARO/MAG I2C: (I2C1)
#define HW_PIN_I2C_SDA   PB8
#define HW_PIN_I2C_SCL   PB9

//Outputs:
#define HW_OUT_COUNT     8
#define HW_PIN_OUT_LIST  {PA0,PA1,PA2,PA3,PB6,PB7,PC6,PC7}

//RC Receiver: (SERIAL1)
#define HW_PIN_RCIN_RX   PB15
#define HW_PIN_RCIN_TX   PB14
#define HW_PIN_RCIN_INVERTER  -1

//GPS: (SERIAL2)
#define HW_PIN_GPS_RX    PNONE
#define HW_PIN_GPS_TX    PD5
#define HW_PIN_GPS_INVERTER  -1

//Battery ADC voltage and current inputs:
#define HW_PIN_BAT_V     PC1
#define HW_PIN_BAT_I     PC0

//-------------------------------------

//Serial
#define HW_SERIAL_COUNT 8
#define HW_PIN_SERIAL { {1,PB14,PB15,-1}, {2,PD5,PNONE,-1}, {3,PD8,PD9,-1}, {4,PD1,PD0,-1}, {5,PB13,PB5,-1}, {6,PC6,PC7,-1}, {7,PNONE,PNONE,-1}, {8,PE1,PE0,-1} } // {INDEX,TX,RX,INVERTER}

//SPI
#define HW_SPI_COUNT 0
#define HW_PIN_SPI {  } // {INDEX,SCK,MISO,MOSI}

//I2C
#define HW_I2C_COUNT 4
#define HW_PIN_I2C { {1,PB8,PB9}, {2,PNONE,PNONE}, {3,PNONE,PNONE}, {4,PNONE,PNONE} } // {INDEX,SCL,SDA}

//other pins
#define HW_PIN_BEEPER PD7
#define HW_PIN_PPM PB15
#define HW_PIN_PWM PB15
#define HW_PIN_LED_STRIP PA8
//#define HW_PIN_LED PE3
#define HW_PIN_TRANSPONDER PB11
#define HW_PIN_CAMERA_CONTROL PE5
#define HW_PIN_ADC_BATT PC1
#define HW_PIN_ADC_RSSI PC4
#define HW_PIN_ADC_CURR PC0
#define HW_PIN_ADC_EXT PC5
#define HW_PIN_SDCARD_DETECT PD10
#define HW_PIN_SDIO_CK PC12
#define HW_PIN_SDIO_CMD PD2
#define HW_PIN_SDIO_D0 PC8
#define HW_PIN_SDIO_D1 PC9
#define HW_PIN_SDIO_D2 PC10
#define HW_PIN_SDIO_D3 PC11
#define HW_PIN_OSD_CS PE11
#define HW_PIN_GYRO_EXTI PD4
#define HW_PIN_GYRO_EXTI_2 PE15
#define HW_PIN_GYRO_CS PA15
#define HW_PIN_GYRO_CS_2 PB12
#define HW_PIN_VTX_POWER PB1
#define HW_PIN_VTX_CS PB0
#define HW_PIN_VTX_DATA PA6
#define HW_PIN_VTX_CLK PA7

//set statements
#define HW_SET_MAG_BUSTYPE I2C
#define HW_SET_MAG_I2C_DEVICE 1
#define HW_SET_BARO_BUSTYPE I2C
#define HW_SET_BARO_I2C_DEVICE 1
#define HW_SET_BLACKBOX_DEVICE SDCARD
#define HW_SET_GYRO_TO_USE BOTH
#define HW_SET_GYRO_1_BUSTYPE SPI
#define HW_SET_GYRO_1_SPIBUS 3
#define HW_SET_GYRO_1_SENSOR_ALIGN CW180
#define HW_SET_GYRO_2_BUSTYPE SPI
#define HW_SET_GYRO_2_SPIBUS 2
#define HW_SET_GYRO_2_SENSOR_ALIGN CUSTOM
#define HW_SET_GYRO_2_ALIGN_ROLL 0
#define HW_SET_GYRO_2_ALIGN_PITCH 0
#define HW_SET_GYRO_2_ALIGN_YAW 2250
#define HW_SET_MAX7456_SPI_BUS 4


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32H750 (SP7E) 4.4.0 Dec 26 2022 / 12:28:11 (7671c7eb01) MSP API: 1.45

#define TARGET_BOARD_IDENTIFIER "SP7E"
#define USBD_PRODUCT_STRING "SPRacingH7EXTREME"

#
# Settings required to boot
#
# Note: hardware for all possible config storage subsystems needs to be known and initialised BEFORE loading
# a config from it can work. (SPI/QSPI/SDCARD).
#

#define FC_VMA_ADDRESS    0x97CE0000

#define EEPROM_SIZE 8192

#define USE_SPRACING_PERSISTENT_RTC_WORKAROUND

#define USE_BUTTONS
#define BUTTON_A_PIN            PE4
#define BUTTON_A_PIN_INVERTED
#define BUTTON_B_PIN            PE4
#define BUTTON_B_PIN_INVERTED

#define USE_QUADSPI
#define USE_QUADSPI_DEVICE_1
#define QUADSPI1_SCK_PIN PB2
#define QUADSPI1_BK1_IO0_PIN PD11
#define QUADSPI1_BK1_IO1_PIN PD12
#define QUADSPI1_BK1_IO2_PIN PE2
#define QUADSPI1_BK1_IO3_PIN PD13
#define QUADSPI1_BK1_CS_PIN PB10
#define QUADSPI1_BK2_IO0_PIN PE7
#define QUADSPI1_BK2_IO1_PIN PE8
#define QUADSPI1_BK2_IO2_PIN PE9
#define QUADSPI1_BK2_IO3_PIN PE10
#define QUADSPI1_BK2_CS_PIN NONE
#define QUADSPI1_MODE QUADSPI_MODE_BK1_ONLY
#define QUADSPI1_CS_FLAGS (QUADSPI_BK1_CS_HARDWARE | QUADSPI_BK2_CS_NONE | QUADSPI_CS_MODE_LINKED)

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define FLASH_QUADSPI_INSTANCE    QUADSPI

#define CONFIG_IN_EXTERNAL_FLASH

#define SDCARD_DETECT_PIN PD10
#define SDCARD_DETECT_INVERTED
#define SDIO_DEVICE             SDIODEV_1
#define SDIO_USE_4BIT           1
#define SDIO_CK_PIN             PC12
#define SDIO_CMD_PIN            PD2
#define SDIO_D0_PIN             PC8
#define SDIO_D1_PIN             PC9
#define SDIO_D2_PIN             PC10
#define SDIO_D3_PIN             PC11

#define USE_SPI

#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PD3
#define SPI2_MISO_PIN           PC2
#define SPI2_MOSI_PIN           PC3
#define SPI2_NSS_PIN            PB12

#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PD6
#define SPI3_NSS_PIN            PA15

#define USE_SPI_DEVICE_4
#define SPI4_SCK_PIN            PE12
#define SPI4_MISO_PIN           PE13
#define SPI4_MOSI_PIN           PE14
#define SPI4_NSS_PIN            PE11

#
# Settings to enable/configure remaining hardware
#

#define USE_USB_ID

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL_PIN PB8
#define I2C1_SDA_PIN PB9
#define I2C_DEVICE (I2CDEV_1)


#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_PID_AUDIO

#define VTX_RTC6705_OPTIONAL

#define ADC1_DMA_OPT 8
#define ADC3_DMA_OPT 9


#
# Settings that are currently defined in target/common_pre.h for non-cloud builds that probably shouldn't be.
# There are here to illustrate that they SHOULD be included in THIS target when they are removed by default.
#

#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_BARO
#define USE_BARO_BMP388
#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_FLASH
#define USE_FLASH_W25N01G
#define USE_SDCARD
#define USE_CAMERA_CONTROL
#define USE_MAX7456

#
# Settings usually defined in common_pre.h for non-cloud-builds
# 

board_name SPRACINGH7EXTREME
manufacturer_id SPRO

resource BEEPER 1 D07
resource MOTOR 1 A00
resource MOTOR 2 A01
resource MOTOR 3 A02
resource MOTOR 4 A03
resource MOTOR 5 B06
resource MOTOR 6 B07
resource MOTOR 7 C06
resource MOTOR 8 C07
resource PPM 1 B15
resource PWM 1 B15
resource LED_STRIP 1 A08

resource SERIAL_TX 1 B14
resource SERIAL_TX 2 D05
resource SERIAL_TX 3 D08
resource SERIAL_TX 4 D01
resource SERIAL_TX 5 B13
resource SERIAL_TX 6 C06
resource SERIAL_TX 7 NONE
resource SERIAL_TX 8 E01
resource SERIAL_RX 1 B15
resource SERIAL_RX 2 NONE
resource SERIAL_RX 3 D09
resource SERIAL_RX 4 D00
resource SERIAL_RX 5 B05
resource SERIAL_RX 6 C07
resource SERIAL_RX 7 NONE
resource SERIAL_RX 8 E00

resource I2C_SCL 1 B08
resource I2C_SDA 1 B09
resource I2C_SCL 2 NONE
resource I2C_SDA 2 NONE
resource I2C_SCL 3 NONE
resource I2C_SDA 3 NONE
resource I2C_SCL 4 NONE
resource I2C_SDA 4 NONE

resource LED 1 E03
resource TRANSPONDER 1 B11

resource CAMERA_CONTROL 1 E05

resource ADC_BATT 1 C01
resource ADC_RSSI 1 C04
resource ADC_CURR 1 C00
resource ADC_EXT 1 C05

resource SDCARD_DETECT 1 D10
resource SDIO_CK 1 C12
resource SDIO_CMD 1 D02
resource SDIO_D0 1 C08
resource SDIO_D1 1 C09
resource SDIO_D2 1 C10
resource SDIO_D3 1 C11
resource OSD_CS 1 E11

resource GYRO_EXTI 1 D04
resource GYRO_EXTI 2 E15
resource GYRO_CS 1 A15
resource GYRO_CS 2 B12
resource VTX_POWER 1 B01
resource VTX_CS 1 B00
resource VTX_DATA 1 A06
resource VTX_CLK 1 A07

feature RSSI_ADC
feature LED_STRIP
feature OSD
feature TRANSPONDER

set mag_bustype = I2C
set mag_i2c_device = 1

set baro_bustype = I2C
set baro_i2c_device = 1

set blackbox_device = SDCARD

set gyro_to_use = BOTH
set gyro_1_bustype = SPI
set gyro_1_spibus = 3
set gyro_1_sensor_align = CW180
set gyro_2_bustype = SPI
set gyro_2_spibus = 2
set gyro_2_sensor_align = CUSTOM
set gyro_2_align_roll = 0
set gyro_2_align_pitch = 0
set gyro_2_align_yaw = 2250
set max7456_spi_bus = 4

*/
