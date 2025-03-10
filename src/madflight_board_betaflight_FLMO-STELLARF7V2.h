/*==============================================================================
Generated on: 2024-10-23 18:06:29.047406
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: STELLARF7V2
Manufacturer ID: FLMO

//copy this line to madflight.ino to use this flight controller (or copy/paste the whole file)
#include <madflight_board_betaflight_FLMO-STELLARF7V2.h>
==============================================================================*/

#define HW_BOARD_NAME "BETAFLIGHT-FLMO-STELLARF7V2"
#define HW_MCU "STM32F7X2"

//Defines from betaflight. Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out.
#define IMU_USE  IMU_USE_SPI_MPU6000
#define IMU_USE  IMU_USE_SPI_ICM20602
#define BARO_USE  BARO_USE_BMP280
#define BB_USE  BB_USE_FLASH
#define BB_FLASH_TYPE  "M25P16"
#define OSD_USE  OSD_USE_MAX7456

//Sensor specific setup
#define IMU_ALIGN  IMU_ALIGN_CW180
#define BARO_I2C_ADR  0
#define MAG_I2C_ADR  0

//LED:
#define HW_PIN_LED       PB2
#define HW_LED_ON        1 //0:low is on, 1:high is on

//IMU SPI: (SPI1)
#define HW_PIN_SPI_MISO  PA6
#define HW_PIN_SPI_MOSI  PA7
#define HW_PIN_SPI_SCLK  PA5
#define HW_PIN_IMU_CS    PA4
#define HW_PIN_IMU_EXTI  PC4

//BARO/MAG I2C: (I2C1)
#define HW_PIN_I2C_SDA   PB8
#define HW_PIN_I2C_SCL   PB9

//Outputs:
#define HW_OUT_COUNT     4
#define HW_PIN_OUT_LIST  {PC9,PC8,PC7,PC6}

//RC Receiver: (SERIAL1)
#define HW_PIN_RCIN_RX   PB7
#define HW_PIN_RCIN_TX   PB6
#define HW_PIN_RCIN_INVERTER  -1

//GPS: (SERIAL2)
#define HW_PIN_GPS_RX    PA3
#define HW_PIN_GPS_TX    PA2
#define HW_PIN_GPS_INVERTER  -1

//Battery ADC voltage and current inputs:
#define HW_PIN_BAT_V     PC1
#define HW_PIN_BAT_I     PC0

//-------------------------------------

//Serial
#define HW_SERIAL_COUNT 5
#define HW_PIN_SERIAL { {1,PB6,PB7,-1}, {2,PA2,PA3,-1}, {3,PC10,PC11,-1}, {4,PA0,PA1,-1}, {5,PC12,PD2,-1} } // {INDEX,TX,RX,INVERTER}

//SPI
#define HW_SPI_COUNT 3
#define HW_PIN_SPI { {1,PA5,PA6,PA7}, {2,PB13,PB14,PB15}, {3,PB3,PB4,PB5} } // {INDEX,SCK,MISO,MOSI}

//I2C
#define HW_I2C_COUNT 1
#define HW_PIN_I2C { {1,PB8,PB9} } // {INDEX,SCL,SDA}

//other pins
#define HW_PIN_SERVO PB11
#define HW_PIN_SERVO_2 PB10
#define HW_PIN_PPM PB7
//#define HW_PIN_LED PB2
#define HW_PIN_ADC_BATT PC1
#define HW_PIN_ADC_CURR PC0
#define HW_PIN_PINIO PC14
#define HW_PIN_PINIO_2 PA15
#define HW_PIN_PINIO_3 PC15
#define HW_PIN_FLASH_CS PC13
#define HW_PIN_OSD_CS PB12
#define HW_PIN_GYRO_EXTI PC4
#define HW_PIN_GYRO_CS PA4
#define HW_PIN_USB_DETECT PA9
#define HW_PIN_BEEPER PC3

//set statements
#define HW_SET_SERIALRX_PROVIDER CRSF
#define HW_SET_MAG_BUSTYPE I2C
#define HW_SET_MAG_I2C_DEVICE 1
#define HW_SET_BARO_BUSTYPE I2C
#define HW_SET_BARO_I2C_DEVICE 1
#define HW_SET_ADC_DEVICE 1
#define HW_SET_BLACKBOX_DEVICE SPIFLASH
#define HW_SET_DSHOT_BURST ON
#define HW_SET_MOTOR_PWM_PROTOCOL DSHOT300
#define HW_SET_CURRENT_METER ADC
#define HW_SET_BATTERY_METER ADC
#define HW_SET_VBAT_SCALE 110
#define HW_SET_IBATA_SCALE 182
#define HW_SET_BEEPER_INVERSION ON
#define HW_SET_BEEPER_OD OFF
#define HW_SET_OSD_DISPLAYPORT_DEVICE MAX7456
#define HW_SET_MAX7456_SPI_BUS 2
#define HW_SET_FLASH_SPI_BUS 3
#define HW_SET_GYRO_1_BUSTYPE SPI
#define HW_SET_GYRO_1_SPIBUS 1
#define HW_SET_GYRO_1_SENSOR_ALIGN CW180
#define HW_SET_GYRO_1_ALIGN_PITCH 1800
#define HW_SET_PINIO_CONFIG 1,1,1,1
#define HW_SET_PINIO_BOX 40,41,42,255
#define HW_SET_SMALL_ANGLE 180
#define HW_SET_BOX_USER_1_NAME VTX
#define HW_SET_BOX_USER_2_NAME CAMERA


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F7X2 (S7X2) 4.4.2 Aug 14 2023 / 20:54:15 (23d066d08) MSP API: 1.45

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_ICM20602
#define USE_GYRO_SPI_ICM20602
#define USE_ACCGYRO_BMI270
#define USE_BARO
#define USE_BARO_BMP280
#define USE_FLASH
#define USE_FLASH_M25P16
#define USE_MAX7456

board_name STELLARF7V2
manufacturer_id FLMO

# resources
resource MOTOR 1 C09
resource MOTOR 2 C08
resource MOTOR 3 C07
resource MOTOR 4 C06
resource SERVO 1 B11
resource SERVO 2 B10
resource PPM 1 B07
resource SERIAL_TX 1 B06
resource SERIAL_RX 1 B07
resource SERIAL_TX 2 A02
resource SERIAL_RX 2 A03
resource SERIAL_TX 3 C10
resource SERIAL_RX 3 C11
resource SERIAL_TX 4 A00
resource SERIAL_RX 4 A01
resource SERIAL_TX 5 C12
resource SERIAL_RX 5 D02
resource I2C_SCL 1 B08
resource I2C_SDA 1 B09
resource LED 1 B02
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 B03
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 B04
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 B05
resource ADC_BATT 1 C01
resource ADC_CURR 1 C00
resource PINIO 1 C14
resource PINIO 2 A15
resource PINIO 3 C15
resource FLASH_CS 1 C13
resource OSD_CS 1 B12
resource GYRO_EXTI 1 C04
resource GYRO_CS 1 A04
resource USB_DETECT 1 A09
resource BEEPER 1 C03

# timer
timer B07 AF2
# pin B07: TIM4 CH2 (AF2)
timer C06 AF3
# pin C06: TIM8 CH1 (AF3)
timer C07 AF3
# pin C07: TIM8 CH2 (AF3)
timer C08 AF3
# pin C08: TIM8 CH3 (AF3)
timer C09 AF3
# pin C09: TIM8 CH4 (AF3)
timer B10 AF1
# pin B10: TIM2 CH3 (AF1)
timer B11 AF1
# pin B11: TIM2 CH4 (AF1)

# dma
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin C06 1
# pin C06: DMA2 Stream 2 Channel 7
dma pin C07 1
# pin C07: DMA2 Stream 3 Channel 7
dma pin C08 1
# pin C08: DMA2 Stream 4 Channel 7
dma pin C09 0
# pin C09: DMA2 Stream 7 Channel 7
dma pin B10 0
# pin B10: DMA1 Stream 1 Channel 3
dma pin B11 0
# pin B11: DMA1 Stream 7 Channel 3

# feature
feature OSD
feature RX_SERIAL
feature TELEMETRY

# serial
serial 0 64 115200 57600 0 115200
serial 1 1 115200 57600 0 115200
serial 2 2048 115200 57600 0 115200
serial 4 2 115200 57600 0 115200

# beacon
beacon RX_LOST
beacon RX_SET

# master
set serialrx_provider = CRSF
set mag_bustype = I2C
set mag_i2c_device = 1
set baro_bustype = I2C
set baro_i2c_device = 1
set adc_device = 1
set blackbox_device = SPIFLASH
set dshot_burst = ON
set motor_pwm_protocol = DSHOT300
set current_meter = ADC
set battery_meter = ADC
set vbat_scale = 110
set ibata_scale = 182
set beeper_inversion = ON
set beeper_od = OFF
set osd_displayport_device = MAX7456
set max7456_spi_bus = 2
set flash_spi_bus = 3
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW180
set gyro_1_align_pitch = 1800
set pinio_config = 1,1,1,1
set pinio_box = 40,41,42,255
set small_angle = 180
set box_user_1_name = VTX POWER
set box_user_2_name = CAMERA SWITCH

*/
