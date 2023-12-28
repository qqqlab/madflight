//==============================================================================
// Generated on: 2023-12-28 01:17:04.308448
// Generated by: _convert.py in this directory
// Source: https://github.com/betaflight/unified-targets
// Board name: NOX
// Manufacturer ID: AIRB
//==============================================================================
#define USE_IMU_SPI_MPU6000
#define USE_IMU_SPI_MPU6500
#define USE_BARO_SPI_BMP280
#define USE_FLASH_M25P16
#define USE_MAX7456

//LED:
const int HW_PIN_LED      = A04;
#define LED_ON 0 //low = on

//Battery voltage divider:
const int HW_PIN_BAT_ADC  = A05;
const int HW_PIN_BAT_CURR = -1;

//GPS: (SERIAL1)
const int HW_PIN_GPS_RX   = B07;
const int HW_PIN_GPS_TX   = B06;
HardwareSerial gps_Serial(HW_PIN_GPS_RX, HW_PIN_GPS_TX);

//RC Receiver: (SERIAL3)
const int HW_PIN_RCIN_RX  = -1;
const int HW_PIN_RCIN_TX  = -1;
HardwareSerial *rcin_Serial = new HardwareSerial(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);

//IMU:
const int HW_PIN_IMU_INT  = NONE;

//I2C: (I2C1)
const int HW_PIN_I2C_SDA  = -1;
const int HW_PIN_I2C_SCL  = -1;
typedef TwoWire HW_WIRETYPE; //define the class to use for I2C
HW_WIRETYPE *i2c = &Wire; //&Wire or &Wire1

//SPI: (SPI2)
const int HW_PIN_SPI_MISO = B14;
const int HW_PIN_SPI_CS   = B12;
const int HW_PIN_SPI_SCLK = B13;
const int HW_PIN_SPI_MOSI = B15;
SPIClass *spi = &SPI;

//Outputs:
#define HW_OUT_COUNT 4
const int8_t HW_PIN_OUT[HW_OUT_COUNT] = {PA1,PA7,PB8,PB1};


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F411 (S411) 4.1.0 Oct 16 2019 / 11:57:34 (c37a7c91a) MSP API: 1.42

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_MPU6500
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_MPU6500
#define USE_BARO
#define USE_BARO_SPI_BMP280
#define USE_FLASH
#define USE_FLASH_M25P16
#define USE_MAX7456

board_name NOX
manufacturer_id AIRB

# resources
resource BEEPER 1 C13
resource MOTOR 1 A01
resource MOTOR 2 A07
resource MOTOR 3 B08
resource MOTOR 4 B01
resource PPM 1 B10
resource LED_STRIP 1 A00
resource SERIAL_TX 1 B06
resource SERIAL_TX 2 A02
resource SERIAL_RX 1 B07
resource SERIAL_RX 2 A03
resource INVERTER 2 C14
resource LED 1 A04
resource SPI_SCK 1 B03
resource SPI_SCK 2 B13
resource SPI_MISO 1 B04
resource SPI_MISO 2 B14
resource SPI_MOSI 1 B05
resource SPI_MOSI 2 B15
resource ESCSERIAL 1 B10
resource ADC_BATT 1 A05
resource BARO_CS 1 A09
resource FLASH_CS 1 A15
resource OSD_CS 1 A10
# Disabling GYRO_EXTI since gyro and OSD are on same SPI bus
# resource GYRO_EXTI 1 A08
resource GYRO_EXTI 1 NONE
resource GYRO_CS 1 B12

# timer
timer B10 AF1
# pin B10: TIM2 CH3 (AF1)
timer A01 AF2
# pin A01: TIM5 CH2 (AF2)
timer A07 AF1
# pin A07: TIM1 CH1N (AF1)
timer B08 AF2
# pin B08: TIM4 CH3 (AF2)
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer A00 AF1
# pin A00: TIM2 CH1 (AF1)
timer A02 AF3
# pin A02: TIM9 CH1 (AF3)
timer A03 AF3
# pin A03: TIM9 CH2 (AF3)

# dma
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin B10 0
# pin B10: DMA1 Stream 1 Channel 3
dma pin A01 0
# pin A01: DMA1 Stream 4 Channel 6
dma pin A07 0
# pin A07: DMA2 Stream 6 Channel 0
dma pin B08 0
# pin B08: DMA1 Stream 7 Channel 2
dma pin B01 0
# pin B01: DMA1 Stream 2 Channel 5
dma pin A00 0
# pin A00: DMA1 Stream 5 Channel 3

# feature
feature RX_SERIAL
feature OSD
feature SOFTSERIAL
feature ESC_SENSOR

# serial
serial 0 2048 115200 57600 0 115200
serial 1 64 115200 57600 0 115200

# master
set baro_spi_device = 2
set serialrx_provider = SBUS
set blackbox_device = SPIFLASH
set min_throttle = 1070
set use_unsynced_pwm = OFF
set motor_pwm_protocol = ONESHOT125
set motor_pwm_rate = 480
set current_meter = ESC
set battery_meter = ESC
set beeper_inversion = ON
set beeper_od = OFF
set system_hse_mhz = 8
set max7456_spi_bus = 2
set flash_spi_bus = 1
set gyro_1_bustype = SPI
set gyro_1_spibus = 2

*/
