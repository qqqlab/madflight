//==============================================================================
// Generated on: 2023-12-28 01:17:04.481604
// Generated by: _convert.py in this directory
// Source: https://github.com/betaflight/unified-targets
// Board name: MLTEMPF4
// Manufacturer ID: MOLA
//==============================================================================
#define USE_IMU_SPI_MPU6000
#define USE_MAX7456
#define USE_SDCARD

//LED:
const int HW_PIN_LED      = C03;
#define LED_ON 0 //low = on

//Battery voltage divider:
const int HW_PIN_BAT_ADC  = C00;
const int HW_PIN_BAT_CURR = C01;

//GPS: (SERIAL1)
const int HW_PIN_GPS_RX   = A10;
const int HW_PIN_GPS_TX   = A09;
HardwareSerial gps_Serial(HW_PIN_GPS_RX, HW_PIN_GPS_TX);

//RC Receiver: (SERIAL3)
const int HW_PIN_RCIN_RX  = -1;
const int HW_PIN_RCIN_TX  = -1;
HardwareSerial *rcin_Serial = new HardwareSerial(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);

//IMU:
const int HW_PIN_IMU_INT  = C05;

//I2C: (I2C1)
const int HW_PIN_I2C_SDA  = -1;
const int HW_PIN_I2C_SCL  = -1;
typedef TwoWire HW_WIRETYPE; //define the class to use for I2C
HW_WIRETYPE *i2c = &Wire; //&Wire or &Wire1

//SPI: (SPI1)
const int HW_PIN_SPI_MISO = A06;
const int HW_PIN_SPI_CS   = A04;
const int HW_PIN_SPI_SCLK = A05;
const int HW_PIN_SPI_MOSI = A07;
SPIClass *spi = &SPI;

//Outputs:
#define HW_OUT_COUNT 4
const int8_t HW_PIN_OUT[HW_OUT_COUNT] = {PB5,PB0,PB3,PB1};


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F405 (S405) 4.1.0 Jul 27 2019 / 18:42:59 (f6870e418) MSP API: 1.42

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_MAX7456
#define USE_SDCARD

board_name MLTEMPF4
manufacturer_id MOLA

# resources
resource BEEPER 1 B04
resource MOTOR 1 B05
resource MOTOR 2 B00
resource MOTOR 3 B03
resource MOTOR 4 B01
resource LED_STRIP 1 A08
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 A02
resource SERIAL_TX 4 A00
resource SERIAL_TX 5 C12
resource SERIAL_TX 6 C06
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 A03
resource SERIAL_RX 4 A01
resource SERIAL_RX 5 D02
resource SERIAL_RX 6 C07
resource LED 1 C03
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 C10
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 C11
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 C12
resource ESCSERIAL 1 B08
resource ADC_BATT 1 C00
resource ADC_CURR 1 C01
resource SDCARD_CS 1 B12
resource SDCARD_DETECT 1 C13
resource OSD_CS 1 A15
resource GYRO_EXTI 1 C05
resource GYRO_CS 1 A04

# timer
timer B05 AF2
# pin B05: TIM3 CH2 (AF2)
timer B00 AF2
# pin B00: TIM3 CH3 (AF2)
timer B03 AF1
# pin B03: TIM2 CH2 (AF1)
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer A02 AF2
# pin A02: TIM5 CH3 (AF2)
timer A03 AF2
# pin A03: TIM5 CH4 (AF2)
timer A08 AF1
# pin A08: TIM1 CH1 (AF1)
timer B08 AF2
# pin B08: TIM4 CH3 (AF2)

# dma
dma SPI_TX 2 0
# SPI_TX 2: DMA1 Stream 4 Channel 0
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin B05 0
# pin B05: DMA1 Stream 5 Channel 5
dma pin B00 0
# pin B00: DMA1 Stream 7 Channel 5
dma pin B03 0
# pin B03: DMA1 Stream 6 Channel 3
dma pin B01 0
# pin B01: DMA1 Stream 2 Channel 5
dma pin A02 0
# pin A02: DMA1 Stream 0 Channel 6
dma pin A03 0
# pin A03: DMA1 Stream 1 Channel 6
dma pin A08 0
# pin A08: DMA2 Stream 6 Channel 0
dma pin B08 0
# pin B08: DMA1 Stream 7 Channel 2

# feature
feature OSD

# master
set blackbox_device = SDCARD
set current_meter = ADC
set battery_meter = ADC
set ibata_scale = 140
set beeper_inversion = ON
set beeper_od = OFF
set sdcard_detect_inverted = ON
set sdcard_mode = SPI
set sdcard_spi_bus = 2
set system_hse_mhz = 8
set max7456_spi_bus = 3
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW180
set gyro_1_align_yaw = 1800

*/
