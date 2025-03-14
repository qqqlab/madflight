/*==============================================================================
Generated on: 2024-10-23 18:06:29.317169
Generated by: betaflight_target_converter.py
Source: https://github.com/betaflight/unified-targets
Board name: STM32F411DISCOVERY
Manufacturer ID: STMI

//copy this line to madflight.ino to use this flight controller (or copy/paste the whole file)
#include <madflight_board_betaflight_STMI-STM32F411DISCOVERY.h>
==============================================================================*/

#define HW_BOARD_NAME "BETAFLIGHT-STMI-STM32F411DISCOVERY"
#define HW_MCU "STM32F411"

//Defines from betaflight. Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out.
#define IMU_USE  IMU_USE_L3GD20
#define MPU_I2C_INSTANCE  I2CDEV_1

//Sensor specific setup
#define IMU_ALIGN  IMU_ALIGN_CW180
#define BARO_I2C_ADR  0
#define MAG_I2C_ADR  0

//LED:
#define HW_PIN_LED       PD15
#define HW_LED_ON        1 //0:low is on, 1:high is on

//IMU SPI: (SPI1)
#define HW_PIN_SPI_MISO  PA6
#define HW_PIN_SPI_MOSI  PA7
#define HW_PIN_SPI_SCLK  PA5
#define HW_PIN_IMU_CS    PE3
#define HW_PIN_IMU_EXTI  PE1

//BARO/MAG I2C: (I2C1)
#define HW_PIN_I2C_SDA   PB6
#define HW_PIN_I2C_SCL   PB9

//Outputs:
#define HW_OUT_COUNT     6
#define HW_PIN_OUT_LIST  {PB1,PB0,PA2,PA3,PA10,PA8}

//RC Receiver: (SERIAL1)
#define HW_PIN_RCIN_RX   PA10
#define HW_PIN_RCIN_TX   PA15
#define HW_PIN_RCIN_INVERTER  -1

//GPS: (SERIAL2)
#define HW_PIN_GPS_RX    PA3
#define HW_PIN_GPS_TX    PA2
#define HW_PIN_GPS_INVERTER  -1

//Battery ADC voltage and current inputs:
#define HW_PIN_BAT_V     PC1
#define HW_PIN_BAT_I     PC2

//-------------------------------------

//Serial
#define HW_SERIAL_COUNT 3
#define HW_PIN_SERIAL { {1,PA15,PA10,-1}, {2,PA2,PA3,-1}, {6,PC6,PC7,-1} } // {INDEX,TX,RX,INVERTER}

//SPI
#define HW_SPI_COUNT 1
#define HW_PIN_SPI { {1,PA5,PA6,PA7} } // {INDEX,SCK,MISO,MOSI}

//I2C
#define HW_I2C_COUNT 1
#define HW_PIN_I2C { {1,PB6,PB9} } // {INDEX,SCL,SDA}

//other pins
#define HW_PIN_BEEPER PD12
#define HW_PIN_PPM PB8
#define HW_PIN_LED_STRIP PB8
//#define HW_PIN_LED PD15
#define HW_PIN_LED_2 PD13
#define HW_PIN_ESCSERIAL PB8
#define HW_PIN_ADC_BATT PC1
#define HW_PIN_ADC_CURR PC2
#define HW_PIN_GYRO_EXTI PE1
#define HW_PIN_GYRO_CS PE3
#define HW_PIN_USB_DETECT PA9

//set statements
#define HW_SET_MAG_BUSTYPE I2C
#define HW_SET_MAG_I2C_DEVICE 1
#define HW_SET_CURRENT_METER ADC
#define HW_SET_BATTERY_METER ADC
#define HW_SET_BEEPER_FREQUENCY 2000
#define HW_SET_SYSTEM_HSE_MHZ 8
#define HW_SET_GYRO_1_BUSTYPE SPI
#define HW_SET_GYRO_1_SPIBUS 1
#define HW_SET_GYRO_1_SENSOR_ALIGN CW180
#define HW_SET_GYRO_1_ALIGN_YAW 1800


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F411 (S411) 4.2.0 Feb  2 2020 / 16:19:09 (30bf9e809f) MSP API: 1.43

#define USE_ACC
#define USE_ACC_LSM303DLHC
#define USE_GYRO
#define USE_GYRO_L3GD20
#define MPU_I2C_INSTANCE I2CDEV_1

board_name STM32F411DISCOVERY
manufacturer_id STMI

# resources
resource BEEPER 1 D12
resource MOTOR 1 B01
resource MOTOR 2 B00
resource MOTOR 3 A02
resource MOTOR 4 A03
resource MOTOR 5 A10
resource MOTOR 6 A08
resource PPM 1 B08
resource LED_STRIP 1 B08
resource SERIAL_TX 1 A15
resource SERIAL_TX 2 A02
resource SERIAL_TX 6 C06
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 A03
resource SERIAL_RX 6 C07
resource I2C_SCL 1 B06
resource I2C_SDA 1 B09
resource LED 1 D15
resource LED 2 D13
resource SPI_SCK 1 A05
resource SPI_MISO 1 A06
resource SPI_MOSI 1 A07
resource ESCSERIAL 1 B08
resource ADC_BATT 1 C01
resource ADC_CURR 1 C02
resource GYRO_EXTI 1 E01
resource GYRO_CS 1 E03
resource USB_DETECT 1 A09

# timer
timer B08 AF2
# pin B08: TIM4 CH3 (AF2)
timer D12 AF2
# pin D12: TIM4 CH1 (AF2)
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer B00 AF2
# pin B00: TIM3 CH3 (AF2)
timer A02 AF1
# pin A02: TIM2 CH3 (AF1)
timer A03 AF1
# pin A03: TIM2 CH4 (AF1)
timer A10 AF1
# pin A10: TIM1 CH3 (AF1)
timer A08 AF1
# pin A08: TIM1 CH1 (AF1)

# dma
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin B08 0
# pin B08: DMA1 Stream 7 Channel 2
dma pin D12 0
# pin D12: DMA1 Stream 0 Channel 2
dma pin B01 0
# pin B01: DMA1 Stream 2 Channel 5
dma pin B00 0
# pin B00: DMA1 Stream 7 Channel 5
dma pin A02 0
# pin A02: DMA1 Stream 1 Channel 3
dma pin A03 1
# pin A03: DMA1 Stream 6 Channel 3
dma pin A10 1
# pin A10: DMA2 Stream 6 Channel 6
dma pin A08 1
# pin A08: DMA2 Stream 1 Channel 6

# master
set mag_bustype = I2C
set mag_i2c_device = 1
set current_meter = ADC
set battery_meter = ADC
set beeper_frequency = 2000
set system_hse_mhz = 8
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW180
set gyro_1_align_yaw = 1800

*/
