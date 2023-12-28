//==============================================================================
// Generated on: 2023-12-28 01:17:04.481604
// Generated by: _convert.py in this directory
// Source: https://github.com/betaflight/unified-targets
// Board name: MATEKF722
// Manufacturer ID: MTKS
//==============================================================================
#define USE_IMU_SPI_ICM20689
#define USE_IMU_SPI_MPU6500
#define USE_BARO_BMP280
#define USE_MAX7456
#define USE_SDCARD

//LED:
const int HW_PIN_LED      = B09;
#define LED_ON 0 //low = on

//Battery voltage divider:
const int HW_PIN_BAT_ADC  = C00;
const int HW_PIN_BAT_CURR = C04;

//GPS: (SERIAL1)
const int HW_PIN_GPS_RX   = A10;
const int HW_PIN_GPS_TX   = A09;
HardwareSerial gps_Serial(HW_PIN_GPS_RX, HW_PIN_GPS_TX);

//RC Receiver: (SERIAL3)
const int HW_PIN_RCIN_RX  = C11;
const int HW_PIN_RCIN_TX  = C10;
HardwareSerial *rcin_Serial = new HardwareSerial(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);

//IMU:
const int HW_PIN_IMU_INT  = C03;

//I2C: (I2C1)
const int HW_PIN_I2C_SDA  = B06;
const int HW_PIN_I2C_SCL  = B07;
typedef TwoWire HW_WIRETYPE; //define the class to use for I2C
HW_WIRETYPE *i2c = &Wire; //&Wire or &Wire1

//SPI: (SPI1)
const int HW_PIN_SPI_MISO = A06;
const int HW_PIN_SPI_CS   = C02;
const int HW_PIN_SPI_SCLK = A05;
const int HW_PIN_SPI_MOSI = A07;
SPIClass *spi = &SPI;

//Outputs:
#define HW_OUT_COUNT 8
const int8_t HW_PIN_OUT[HW_OUT_COUNT] = {PC6,PC7,PC8,PC9,PB1,PA8,PB8,PA2};


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F7X2 (S7X2) 4.1.0 May 12 2019 / 23:09:44 (a8e9dd94e) MSP API: 1.42

#define USE_ACC
#define USE_ACC_SPI_ICM20689
#define USE_ACC_SPI_MPU6500
#define USE_GYRO
#define USE_GYRO_SPI_ICM20689
#define USE_GYRO_SPI_MPU6500
#define USE_BARO
#define USE_BARO_BMP280
#define USE_MAX7456
#define USE_SDCARD

board_name MATEKF722
manufacturer_id MTKS

# resources
resource BEEPER 1 C13
resource MOTOR 1 C06
resource MOTOR 2 C07
resource MOTOR 3 C08
resource MOTOR 4 C09
resource MOTOR 5 B01
resource MOTOR 6 A08
resource MOTOR 7 B08
resource MOTOR 8 A02
resource PPM 1 A03
resource PWM 1 A00
resource PWM 2 A01
resource LED_STRIP 1 A15
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 A02
resource SERIAL_TX 3 C10
resource SERIAL_TX 4 A00
resource SERIAL_TX 5 C12
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 A03
resource SERIAL_RX 3 C11
resource SERIAL_RX 4 A01
resource SERIAL_RX 5 D02
resource I2C_SCL 1 B06
resource I2C_SDA 1 B07
resource LED 1 B09
resource LED 2 A14
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 B03
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 B04
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 B05
resource ADC_BATT 1 C00
resource ADC_RSSI 1 B00
resource ADC_CURR 1 C04
resource SDCARD_CS 1 C01
resource OSD_CS 1 B10
resource GYRO_EXTI 1 C03
resource GYRO_CS 1 C02
resource USB_DETECT 1 B12

# timer
timer A03 AF3
# pin A03: TIM9 CH2 (AF3)
timer C06 AF3
# pin C06: TIM8 CH1 (AF3)
timer C07 AF3
# pin C07: TIM8 CH2 (AF3)
timer C08 AF3
# pin C08: TIM8 CH3 (AF3)
timer C09 AF3
# pin C09: TIM8 CH4 (AF3)
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer A08 AF1
# pin A08: TIM1 CH1 (AF1)
timer B08 AF2
# pin B08: TIM4 CH3 (AF2)
timer A02 AF2
# pin A02: TIM5 CH3 (AF2)
timer A00 AF2
# pin A00: TIM5 CH1 (AF2)
timer A01 AF2
# pin A01: TIM5 CH2 (AF2)
timer A15 AF1
# pin A15: TIM2 CH1 (AF1)

# dma
dma SPI_TX 3 1
# SPI_TX 3: DMA1 Stream 7 Channel 0
dma ADC 1 0
# ADC 1: DMA2 Stream 0 Channel 0
dma pin C06 1
# pin C06: DMA2 Stream 2 Channel 7
dma pin C07 1
# pin C07: DMA2 Stream 3 Channel 7
dma pin C08 1
# pin C08: DMA2 Stream 4 Channel 7
dma pin C09 0
# pin C09: DMA2 Stream 7 Channel 7
dma pin B01 0
# pin B01: DMA1 Stream 2 Channel 5
dma pin A08 0
# pin A08: DMA2 Stream 6 Channel 0
dma pin B08 0
# pin B08: DMA1 Stream 7 Channel 2
dma pin A02 0
# pin A02: DMA1 Stream 0 Channel 6
dma pin A00 0
# pin A00: DMA1 Stream 2 Channel 6
dma pin A01 0
# pin A01: DMA1 Stream 4 Channel 6
dma pin A15 0
# pin A15: DMA1 Stream 5 Channel 3

# feature
feature RX_SERIAL
feature TELEMETRY
feature OSD

# serial
serial 1 64 115200 57600 0 115200

# master
set serialrx_provider = SBUS
set mag_bustype = I2C
set mag_i2c_device = 1
set mag_hardware = NONE
set baro_bustype = I2C
set baro_i2c_device = 1
set blackbox_device = SDCARD
set dshot_burst = ON
set current_meter = ADC
set battery_meter = ADC
set ibata_scale = 179
set beeper_inversion = ON
set beeper_od = OFF
set sdcard_mode = SPI
set sdcard_spi_bus = 3
set max7456_spi_bus = 2
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW180
set gyro_2_spibus = 1

*/
