//==============================================================================
// Generated on: 2023-12-28 01:17:04.465967
// Generated by: _convert.py in this directory
// Source: https://github.com/betaflight/unified-targets
// Board name: JHEF411HD
// Manufacturer ID: JHEF
//==============================================================================
#define USE_IMU_SPI_MPU6000
#define USE_IMU_SPI_ICM42688P
#define USE_BARO_BMP280
#define USE_BARO_DPS310
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

//LED:
const int HW_PIN_LED      = C13;
#define LED_ON 0 //low = on

//Battery voltage divider:
const int HW_PIN_BAT_ADC  = B00;
const int HW_PIN_BAT_CURR = B01;

//GPS: (SERIAL1)
const int HW_PIN_GPS_RX   = A10;
const int HW_PIN_GPS_TX   = A09;
HardwareSerial gps_Serial(HW_PIN_GPS_RX, HW_PIN_GPS_TX);

//RC Receiver: (SERIAL3)
const int HW_PIN_RCIN_RX  = -1;
const int HW_PIN_RCIN_TX  = -1;
HardwareSerial *rcin_Serial = new HardwareSerial(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);

//IMU:
const int HW_PIN_IMU_INT  = A01;

//I2C: (I2C1)
const int HW_PIN_I2C_SDA  = B08;
const int HW_PIN_I2C_SCL  = B09;
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
const int8_t HW_PIN_OUT[HW_OUT_COUNT] = {PB4,PB5,PB6,PB7};


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F411 (S411) 4.2.0 Jun 14 2020 / 03:04:43 (8f2d21460) MSP API: 1.43

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_ICM42688P
#define USE_ACC_SPI_ICM42688P
#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_DPS310
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

board_name JHEF411HD
manufacturer_id JHEF

# resources
resource BEEPER 1 B02
resource MOTOR 1 B04
resource MOTOR 2 B05
resource MOTOR 3 B06
resource MOTOR 4 B07
resource LED_STRIP 1 A08
resource SERIAL_TX 1 A09
resource SERIAL_RX 1 A10
resource SERIAL_TX 2 A02
resource SERIAL_RX 2 A03
resource SERIAL_RX 11 B03
resource SERIAL_TX 11 A15
resource I2C_SCL 1 B08
resource I2C_SDA 1 B09
resource LED 1 C13
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource ADC_BATT 1 B00
resource ADC_CURR 1 B01
resource PINIO 1 B10
resource PINIO 2 C14
resource FLASH_CS 1 A00
resource OSD_CS 1 B12
resource GYRO_EXTI 1 A01
resource GYRO_CS 1 A04
resource USB_DETECT 1 C15

# timer
timer A03 AF3
# pin A03: TIM9 CH2 (AF3)
timer B04 AF2
# pin B04: TIM3 CH1 (AF2)
timer B05 AF2
# pin B05: TIM3 CH2 (AF2)
timer B06 AF2
# pin B06: TIM4 CH1 (AF2)
timer B07 AF2
# pin B07: TIM4 CH2 (AF2)
timer B03 AF1
# pin B03: TIM2 CH2 (AF1)
timer B10 AF1
# pin B10: TIM2 CH3 (AF1)
timer A00 AF2
# pin A00: TIM5 CH1 (AF2)
timer A02 AF2
# pin A02: TIM5 CH3 (AF2)
timer A08 AF1
# pin A08: TIM1 CH1 (AF1)
timer A15 AF1
# pin A15: TIM2 CH1 (AF1)

# dma
dma ADC 1 0
# ADC 1: DMA2 Stream 0 Channel 0
dma pin B04 0
# pin B04: DMA1 Stream 4 Channel 5
dma pin B05 0
# pin B05: DMA1 Stream 5 Channel 5
dma pin B06 0
# pin B06: DMA1 Stream 0 Channel 2
dma pin B07 0
# pin B07: DMA1 Stream 3 Channel 2
dma pin B03 0
# pin B03: DMA1 Stream 6 Channel 3
dma pin B10 0
# pin B10: DMA1 Stream 1 Channel 3
dma pin A00 0
# pin A00: DMA1 Stream 2 Channel 6
dma pin A02 0
# pin A02: DMA1 Stream 0 Channel 6
dma pin A08 0
# pin A08: DMA2 Stream 6 Channel 0
dma pin A15 0
# pin A15: DMA1 Stream 5 Channel 3

# feature
feature RX_SERIAL
feature TELEMETRY
feature SOFTSERIAL
feature OSD

# serial
serial 20 1 115200 57600 0 115200
serial 0 0 115200 57600 0 115200
serial 1 64 115200 57600 0 115200
serial 30 0 115200 57600 0 115200
serial 1 0 115200 57600 0 115200

# master
set baro_bustype = I2C
set baro_i2c_device = 1
set serialrx_provider = CRSF
set adc_device = 1
set blackbox_device = SPIFLASH
set current_meter = ADC
set battery_meter = ADC
set beeper_inversion = ON
set beeper_od = OFF
set mag_bustype = I2C
set mag_i2c_device = 1
set mag_hardware = NONE
set max7456_spi_bus = 2
set pinio_config = 1,1,1,1
set pinio_box = 40,41,255,255
set flash_spi_bus = 2
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW90

*/
