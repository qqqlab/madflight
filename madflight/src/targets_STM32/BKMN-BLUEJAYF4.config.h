//==============================================================================
// Generated on: 2023-12-28 01:17:04.324056
// Generated by: _convert.py in this directory
// Source: https://github.com/betaflight/unified-targets
// Board name: BLUEJAYF4
// Manufacturer ID: BKMN
//==============================================================================
#define USE_IMU_SPI_MPU6500
#define USE_FLASH_W25P16

//LED:
const int HW_PIN_LED      = B06;
#define LED_ON 0 //low = on

//Battery voltage divider:
const int HW_PIN_BAT_ADC  = C03;
const int HW_PIN_BAT_CURR = C02;

//GPS: (SERIAL1)
const int HW_PIN_GPS_RX   = A10;
const int HW_PIN_GPS_TX   = A09;
HardwareSerial gps_Serial(HW_PIN_GPS_RX, HW_PIN_GPS_TX);

//RC Receiver: (SERIAL3)
const int HW_PIN_RCIN_RX  = B11;
const int HW_PIN_RCIN_TX  = B10;
HardwareSerial *rcin_Serial = new HardwareSerial(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);

//IMU:
const int HW_PIN_IMU_INT  = C05;

//I2C: (I2C1)
const int HW_PIN_I2C_SDA  = B08;
const int HW_PIN_I2C_SCL  = B09;
typedef TwoWire HW_WIRETYPE; //define the class to use for I2C
HW_WIRETYPE *i2c = &Wire; //&Wire or &Wire1

//SPI: (SPI1)
const int HW_PIN_SPI_MISO = A06;
const int HW_PIN_SPI_CS   = C04;
const int HW_PIN_SPI_SCLK = A05;
const int HW_PIN_SPI_MOSI = A07;
SPIClass *spi = &SPI;

//Outputs:
#define HW_OUT_COUNT 6
const int8_t HW_PIN_OUT[HW_OUT_COUNT] = {PA0,PA1,PA2,PA3,PB0,PB1};


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F405 (S405) 4.1.0 Oct 16 2019 / 11:57:16 (c37a7c91a) MSP API: 1.42
# manufacturer_id: BKMN   board_name: BLUEJAYF4   custom defaults: NO

#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_FLASH
#define USE_FLASH_W25P16

board_name BLUEJAYF4
manufacturer_id BKMN

# resources
resource BEEPER 1 C01
resource MOTOR 1 A00
resource MOTOR 2 A01
resource MOTOR 3 A02
resource MOTOR 4 A03
resource MOTOR 5 B00
resource MOTOR 6 B01
resource PPM 1 C07
resource LED_STRIP 1 B00
resource SERIAL_TX 1 A09
resource SERIAL_TX 3 B10
resource SERIAL_TX 6 C06
resource SERIAL_TX 11 B03
resource SERIAL_RX 1 A10
resource SERIAL_RX 3 B11
resource SERIAL_RX 6 C07
resource INVERTER 6 B15
resource I2C_SCL 1 B08
resource I2C_SDA 1 B09
resource LED 1 B06
resource LED 2 B05
resource LED 3 B04
resource SPI_SCK 1 A05
resource SPI_SCK 3 C10
resource SPI_MISO 1 A06
resource SPI_MISO 3 C11
resource SPI_MOSI 1 A07
resource SPI_MOSI 3 C12
resource ESCSERIAL 1 C07
resource ADC_BATT 1 C03
resource ADC_CURR 1 C02
resource SDCARD_CS 1 A15
resource SDCARD_DETECT 1 D02
resource FLASH_CS 1 B07
resource GYRO_EXTI 1 C05
resource GYRO_CS 1 C04

# timer
timer C07 AF2
# pin C07: TIM3 CH2 (AF2)
timer A00 AF2
# pin A00: TIM5 CH1 (AF2)
timer A01 AF2
# pin A01: TIM5 CH2 (AF2)
timer A02 AF2
# pin A02: TIM5 CH3 (AF2)
timer A03 AF2
# pin A03: TIM5 CH4 (AF2)
timer B00 AF1
# pin B00: TIM1 CH2N (AF1)
timer B01 AF3
# pin B01: TIM8 CH3N (AF3)
timer B03 AF1
# pin B03: TIM2 CH2 (AF1)

# dma
dma SPI_TX 3 0
# SPI_TX 3: DMA1 Stream 5 Channel 0
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin C07 0
# pin C07: DMA1 Stream 5 Channel 5
dma pin A00 0
# pin A00: DMA1 Stream 2 Channel 6
dma pin A01 0
# pin A01: DMA1 Stream 4 Channel 6
dma pin A02 0
# pin A02: DMA1 Stream 0 Channel 6
dma pin A03 1
# pin A03: DMA1 Stream 3 Channel 6
dma pin B00 0
# pin B00: DMA2 Stream 6 Channel 0
dma pin B01 0
# pin B01: DMA2 Stream 2 Channel 0
dma pin B03 0
# pin B03: DMA1 Stream 6 Channel 3

# feature
feature OSD

# master
set mag_bustype = I2C
set mag_i2c_device = 1
set baro_bustype = I2C
set baro_i2c_device = 1
set blackbox_device = SDCARD
set battery_meter = ADC
set beeper_inversion = ON
set beeper_od = OFF
set sdcard_detect_inverted = ON
set sdcard_mode = SPI
set sdcard_spi_bus = 3
set system_hse_mhz = 8
set dashboard_i2c_bus = 1
set flash_spi_bus = 3
set gyro_1_bustype = SPI
set gyro_1_spibus = 1

*/
