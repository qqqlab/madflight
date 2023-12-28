//==============================================================================
// Generated on: 2023-12-28 01:17:04.497225
// Generated by: _convert.py in this directory
// Source: https://github.com/betaflight/unified-targets
// Board name: BEEROTORF4
// Manufacturer ID: RCTI
//==============================================================================
#define USE_IMU_SPI_ICM20689
#define USE_MAX7456
#define USE_SDCARD

//LED:
const int HW_PIN_LED      = B04;
#define LED_ON 0 //low = on

//Battery voltage divider:
const int HW_PIN_BAT_ADC  = C00;
const int HW_PIN_BAT_CURR = C01;

//GPS: (SERIAL1)
const int HW_PIN_GPS_RX   = A10;
const int HW_PIN_GPS_TX   = A09;
HardwareSerial gps_Serial(HW_PIN_GPS_RX, HW_PIN_GPS_TX);

//RC Receiver: (SERIAL3)
const int HW_PIN_RCIN_RX  = B11;
const int HW_PIN_RCIN_TX  = B10;
HardwareSerial *rcin_Serial = new HardwareSerial(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);

//IMU:
const int HW_PIN_IMU_INT  = A08;

//I2C: (I2C1)
const int HW_PIN_I2C_SDA  = B06;
const int HW_PIN_I2C_SCL  = B07;
typedef TwoWire HW_WIRETYPE; //define the class to use for I2C
HW_WIRETYPE *i2c = &Wire; //&Wire or &Wire1

//SPI: (SPI1)
const int HW_PIN_SPI_MISO = A06;
const int HW_PIN_SPI_CS   = A04;
const int HW_PIN_SPI_SCLK = A05;
const int HW_PIN_SPI_MOSI = A07;
SPIClass *spi = &SPI;

//Outputs:
#define HW_OUT_COUNT 8
const int8_t HW_PIN_OUT[HW_OUT_COUNT] = {PB0,PB1,PA1,PA0,PC6,PC7,PB5,PB9};


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F405 (S405) 4.1.0 May 26 2019 / 13:44:05 (00969f3ba) MSP API: 1.42

#define USE_GYRO
#define USE_GYRO_SPI_ICM20689
#define USE_ACC
#define USE_ACC_SPI_ICM20689
#define USE_MAX7456
#define USE_SDCARD

board_name BEEROTORF4
manufacturer_id RCTI

# resources
resource BEEPER 1 B03
resource MOTOR 1 B00
resource MOTOR 2 B01
resource MOTOR 3 A01
resource MOTOR 4 A00
resource MOTOR 5 C06
resource MOTOR 6 C07
resource MOTOR 7 B05
resource MOTOR 8 B09
resource PPM 1 A03
resource LED_STRIP 1 B08
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 A02
resource SERIAL_TX 3 B10
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 A03
resource SERIAL_RX 3 B11
resource INVERTER 2 C15
resource INVERTER 3 C14
resource I2C_SCL 1 B06
resource I2C_SDA 1 B07
resource LED 1 B04
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 C10
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 C11
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 C12
resource ESCSERIAL 1 A03
resource ADC_BATT 1 C00
resource ADC_RSSI 1 C02
resource ADC_CURR 1 C01
resource SDCARD_CS 1 B12
resource SDCARD_DETECT 1 C03
resource OSD_CS 1 A15
resource GYRO_EXTI 1 A08
resource GYRO_CS 1 A04
resource USB_DETECT 1 C05

# timer
timer A03 AF3
# pin A03: TIM9 CH2 (AF3)
timer B00 AF1
# pin B00: TIM1 CH2N (AF1)
timer B01 AF3
# pin B01: TIM8 CH3N (AF3)
timer A01 AF1
# pin A01: TIM2 CH2 (AF1)
timer A00 AF2
# pin A00: TIM5 CH1 (AF2)
timer C06 AF2
# pin C06: TIM3 CH1 (AF2)
timer C07 AF3
# pin C07: TIM8 CH2 (AF3)
timer B05 AF2
# pin B05: TIM3 CH2 (AF2)
timer B09 AF2
# pin B09: TIM4 CH4 (AF2)
timer B08 AF2
# pin B08: TIM4 CH3 (AF2)

# dma
dma SPI_TX 2 0
# SPI_TX 2: DMA1 Stream 4 Channel 0
dma SPI_TX 3 0
# SPI_TX 3: DMA1 Stream 5 Channel 0
dma SPI_RX 3 0
# SPI_RX 3: DMA1 Stream 0 Channel 0
dma ADC 1 0
# ADC 1: DMA2 Stream 0 Channel 0
dma pin B00 0
# pin B00: DMA2 Stream 6 Channel 0
dma pin B01 1
# pin B01: DMA2 Stream 4 Channel 7
dma pin A01 0
# pin A01: DMA1 Stream 6 Channel 3
dma pin A00 0
# pin A00: DMA1 Stream 2 Channel 6
dma pin C06 0
# pin C06: DMA1 Stream 4 Channel 5
dma pin C07 0
# pin C07: DMA2 Stream 2 Channel 0
dma pin B05 0
# pin B05: DMA1 Stream 5 Channel 5
dma pin B08 0
# pin B08: DMA1 Stream 7 Channel 2

# feature
feature OSD

# master
set baro_bustype = I2C
set baro_i2c_device = 1
set blackbox_device = SDCARD
set dshot_burst = OFF
set current_meter = ADC
set battery_meter = ADC
set beeper_inversion = ON
set beeper_od = OFF
set sdcard_detect_inverted = ON
set sdcard_mode = SPI
set sdcard_spi_bus = 2
set system_hse_mhz = 8
set max7456_spi_bus = 3
set dashboard_i2c_bus = 1
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW270
set gyro_2_spibus = 1

*/
