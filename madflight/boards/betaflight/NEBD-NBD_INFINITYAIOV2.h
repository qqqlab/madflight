/*==============================================================================
Generated on: 2023-12-29 16:14:55.449139
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: NBD_INFINITYAIOV2
Manufacturer ID: NEBD

//copy this line to madflight.ino to use this flight controller (or copy/paste the whole file)
#include "boards/betaflight/NEBD-NBD_INFINITYAIOV2.h"
==============================================================================*/

#define HW_BOARD_NAME "BETAFLIGHT-NEBD-NBD_INFINITYAIOV2"
#define HW_MCU "STM32F745"

//Defines from betaflight. Note: madflight will pick the first IMU that is matched in imu.h, this might not be the IMU that is actually on the board. Comment the offending IMU out.
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

//Sensor specific setup
#define IMU_ROTATE_CW180
#define BARO_I2C_ADR 0
#define MAG_I2C_ADR 0

//LED:
const int HW_PIN_LED      = PC0;
const int HW_LED_ON       = 1; //0:low is on, 1:high is on

//IMU SPI: (SPI4)
const int HW_PIN_SPI_MISO = PE13;
const int HW_PIN_SPI_MOSI = PE14;
const int HW_PIN_SPI_SCLK = PE12;
const int HW_PIN_IMU_CS   = PE11;
const int HW_PIN_IMU_EXTI = PB1;

//BARO/MAG I2C: (I2C1)
const int HW_PIN_I2C_SDA  = PB8;
const int HW_PIN_I2C_SCL  = PB9;

//Outputs:
const int HW_OUT_COUNT    = 4;
const int HW_PIN_OUT[]    = {PC6,PC7,PC8,PC9};

//RC Receiver: (SERIAL2)
const int HW_PIN_RCIN_RX  = PA3;
const int HW_PIN_RCIN_TX  = PA2;
const int HW_PIN_RCIN_INVERTER = -1;

//GPS: (SERIAL3)
const int HW_PIN_GPS_RX   = PB11;
const int HW_PIN_GPS_TX   = PB10;
const int HW_PIN_GPS_INVERTER = -1;

//Battery ADC voltage and current inputs:
const int HW_PIN_BAT_V    = PC1;
const int HW_PIN_BAT_I    = PC2;

//Include Libraries
#include <Wire.h>                      //I2C communication
#include <SPI.h>                       //SPI communication
#include "src/hw_STM32/STM32_PWM.h"    //Servo and oneshot

//Bus Setup
HardwareSerial *rcin_Serial = new HardwareSerial(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);
HardwareSerial gps_Serial(HW_PIN_GPS_RX, HW_PIN_GPS_TX);
typedef TwoWire HW_WIRETYPE; //define the class to use for I2C
HW_WIRETYPE *i2c = &Wire; //&Wire or &Wire1
SPIClass *spi = &SPI;

//Serial
#define HW_SERIAL_COUNT 6
#define HW_PIN_SERIAL { {1,-1,PB7,-1}, {2,PA2,PA3,-1}, {3,PB10,PB11,-1}, {5,-1,PD2,-1}, {7,PE8,-1,-1}, {8,PE1,PE0,-1} } // {INDEX,TX,RX,INVERTER}

//SPI
#define HW_SPI_COUNT 3
#define HW_PIN_SPI { {1,PA5,PA6,PA7}, {3,PB3,PB4,PD6}, {4,PE12,PE13,PE14} } // {INDEX,SCK,MISO,MOSI}

//I2C
#define HW_I2C_COUNT 1
#define HW_PIN_I2C { {1,PB8,PB9} } // {INDEX,SCL,SDA}

//Motors:
#define HW_MOTOR_COUNT 4
#define HW_MOTOR_OUT {PC6,PC7,PC8,PC9}

//other pins
#define HW_PIN_BEEPER PD13
#define HW_PIN_LED_STRIP PA9
//#define HW_PIN_LED PC0
#define HW_PIN_ADC_BATT PC1
#define HW_PIN_ADC_CURR PC2
#define HW_PIN_FLASH_CS PB0
#define HW_PIN_OSD_CS PA15
#define HW_PIN_GYRO_EXTI PB1
#define HW_PIN_GYRO_CS PE11

//set statements
#define HW_SET_MAG_BUSTYPE I2C
#define HW_SET_MAG_I2C_DEVICE 1
#define HW_SET_SERIALRX_PROVIDER SBUS
#define HW_SET_BLACKBOX_DEVICE SPIFLASH
#define HW_SET_DSHOT_BURST ON
#define HW_SET_DSHOT_BIDIR ON
#define HW_SET_MOTOR_PWM_PROTOCOL DSHOT600
#define HW_SET_CURRENT_METER ADC
#define HW_SET_BATTERY_METER ADC
#define HW_SET_IBATA_SCALE 230
#define HW_SET_BEEPER_INVERSION ON
#define HW_SET_BEEPER_OD OFF
#define HW_SET_BEEPER_FREQUENCY 5400
#define HW_SET_MAX7456_SPI_BUS 3
#define HW_SET_FLASH_SPI_BUS 1
#define HW_SET_GYRO_1_BUSTYPE SPI
#define HW_SET_GYRO_1_SPIBUS 4
#define HW_SET_GYRO_1_SENSOR_ALIGN CW180
#define HW_SET_GYRO_1_ALIGN_YAW 1800


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F745 (S745) 4.2.11 Nov  9 2021 / 20:29:04 (948ba6339) MSP API: 1.43

#define USE_ACC
#define USE_GYRO
#define USE_ACCGYRO_BMI270
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

board_name NBD_INFINITYAIOV2
manufacturer_id NEBD

# resources
resource BEEPER 1 D13
resource MOTOR 1 C06
resource MOTOR 2 C07
resource MOTOR 3 C08
resource MOTOR 4 C09
resource LED_STRIP 1 A09
resource SERIAL_TX 2 A02
resource SERIAL_TX 3 B10
resource SERIAL_TX 7 E08
resource SERIAL_TX 8 E01
resource SERIAL_RX 1 B07
resource SERIAL_RX 2 A03
resource SERIAL_RX 3 B11
resource SERIAL_RX 5 D02
resource SERIAL_RX 8 E00
resource I2C_SCL 1 B08
resource I2C_SDA 1 B09
resource LED 1 C00
resource SPI_SCK 1 A05
resource SPI_SCK 3 B03
resource SPI_SCK 4 E12
resource SPI_MISO 1 A06
resource SPI_MISO 3 B04
resource SPI_MISO 4 E13
resource SPI_MOSI 1 A07
resource SPI_MOSI 3 D06
resource SPI_MOSI 4 E14
resource ADC_BATT 1 C01
resource ADC_CURR 1 C02
resource FLASH_CS 1 B00
resource OSD_CS 1 A15
resource GYRO_EXTI 1 B01
resource GYRO_CS 1 E11

# timer
timer C06 AF2
# pin C06: TIM3 CH1 (AF2)
timer C07 AF2
# pin C07: TIM3 CH2 (AF2)
timer C08 AF2
# pin C08: TIM3 CH3 (AF2)
timer C09 AF2
# pin C09: TIM3 CH4 (AF2)
timer A09 AF1
# pin A09: TIM1 CH2 (AF1)
timer D13 AF2
# pin D13: TIM4 CH2 (AF2)

# dma
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin C06 0
# pin C06: DMA1 Stream 4 Channel 5
dma pin C07 0
# pin C07: DMA1 Stream 5 Channel 5
dma pin C08 0
# pin C08: DMA1 Stream 7 Channel 5
dma pin C09 0
# pin C09: DMA1 Stream 2 Channel 5
dma pin A09 0
# pin A09: DMA2 Stream 6 Channel 0
dma pin D13 0
# pin D13: DMA1 Stream 3 Channel 2

# feature
feature RX_SERIAL
feature OSD

# serial
serial 0 64 115200 57600 0 115200

# master
set mag_bustype = I2C
set mag_i2c_device = 1
set serialrx_provider = SBUS
set blackbox_device = SPIFLASH
set dshot_burst = ON
set dshot_bidir = ON
set motor_pwm_protocol = DSHOT600
set current_meter = ADC
set battery_meter = ADC
set ibata_scale = 230
set beeper_inversion = ON
set beeper_od = OFF
set beeper_frequency = 5400
set max7456_spi_bus = 3
set flash_spi_bus = 1
set gyro_1_bustype = SPI
set gyro_1_spibus = 4
set gyro_1_sensor_align = CW180
set gyro_1_align_yaw = 1800

*/