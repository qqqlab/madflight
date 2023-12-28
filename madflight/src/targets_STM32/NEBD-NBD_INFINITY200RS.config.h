//==============================================================================
// Generated on: 2023-12-28 01:17:04.481604
// Generated by: _convert.py in this directory
// Source: https://github.com/betaflight/unified-targets
// Board name: NBD_INFINITY200RS
// Manufacturer ID: NEBD
//==============================================================================
#define USE_IMU_SPI_MPU6000
#define USE_FLASH_W25N01G
#define USE_MAX7456

//LED:
const int HW_PIN_LED      = C00;
#define LED_ON 0 //low = on

//Battery voltage divider:
const int HW_PIN_BAT_ADC  = C01;
const int HW_PIN_BAT_CURR = C02;

//GPS: (SERIAL1)
const int HW_PIN_GPS_RX   = B07;
const int HW_PIN_GPS_TX   = B06;
HardwareSerial gps_Serial(HW_PIN_GPS_RX, HW_PIN_GPS_TX);

//RC Receiver: (SERIAL3)
const int HW_PIN_RCIN_RX  = -1;
const int HW_PIN_RCIN_TX  = B10;
HardwareSerial *rcin_Serial = new HardwareSerial(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);

//IMU:
const int HW_PIN_IMU_INT  = B01;

//I2C: (I2C1)
const int HW_PIN_I2C_SDA  = B08;
const int HW_PIN_I2C_SCL  = B09;
typedef TwoWire HW_WIRETYPE; //define the class to use for I2C
HW_WIRETYPE *i2c = &Wire; //&Wire or &Wire1

//SPI: (SPI4)
const int HW_PIN_SPI_MISO = E13;
const int HW_PIN_SPI_CS   = E11;
const int HW_PIN_SPI_SCLK = E12;
const int HW_PIN_SPI_MOSI = E14;
SPIClass *spi = &SPI;

//Outputs:
#define HW_OUT_COUNT 4
const int8_t HW_PIN_OUT[HW_OUT_COUNT] = {PC8,PC6,PC9,PC7};


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F745 (S745) 4.4.0 Apr 30 2023 / 08:22:46 (norevision) MSP API: 1.45

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACCGYRO_BMI270
#define USE_FLASH
#define USE_FLASH_W25N01G
#define USE_MAX7456

board_name NBD_INFINITY200RS
manufacturer_id NEBD

# resources
resource MOTOR 1 C08
resource MOTOR 2 C06
resource MOTOR 3 C09
resource MOTOR 4 C07
resource LED_STRIP 1 A09
resource SERIAL_TX 1 B06
resource SERIAL_TX 2 A02
resource SERIAL_TX 3 B10
resource SERIAL_TX 7 E08
resource SERIAL_TX 8 E01
resource SERIAL_RX 1 B07
resource SERIAL_RX 2 A03
resource SERIAL_RX 5 D02
resource SERIAL_RX 7 E07
resource SERIAL_RX 8 E00
resource I2C_SCL 1 B08
resource I2C_SDA 1 B09
resource LED 1 C00
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 B03
resource SPI_SCK 4 E12
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 B04
resource SPI_MISO 4 E13
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 D06
resource SPI_MOSI 4 E14
resource ADC_BATT 1 C01
resource ADC_CURR 1 C02
resource FLASH_CS 1 B00
resource OSD_CS 1 A15
resource GYRO_EXTI 1 B01
resource GYRO_EXTI 2 E09
resource GYRO_CS 1 E11
resource GYRO_CS 2 B12

# timer
timer C08 AF2
# pin C08: TIM3 CH3 (AF2)
timer C06 AF2
# pin C06: TIM3 CH1 (AF2)
timer C09 AF2
# pin C09: TIM3 CH4 (AF2)
timer C07 AF2
# pin C07: TIM3 CH2 (AF2)
timer A01 AF1
# pin A01: TIM2 CH2 (AF1)
timer A09 AF1
# pin A09: TIM1 CH2 (AF1)

# dma
dma ADC 1 0
# ADC 1: DMA2 Stream 0 Channel 0
dma pin C08 0
# pin C08: DMA1 Stream 7 Channel 5
dma pin C06 0
# pin C06: DMA1 Stream 4 Channel 5
dma pin C09 0
# pin C09: DMA1 Stream 2 Channel 5
dma pin C07 0
# pin C07: DMA1 Stream 5 Channel 5
dma pin A01 0
# pin A01: DMA1 Stream 6 Channel 3
dma pin A09 0
# pin A09: DMA2 Stream 6 Channel 0

# feature
feature LED_STRIP
feature OSD
feature ESC_SENSOR

# serial
serial 0 1 115200 57600 0 115200
serial 1 2048 115200 57600 0 115200
serial 4 1024 115200 57600 0 115200

# led
led 0 6,11::CTO:0
led 1 7,11::CTO:0
led 2 8,11::CTO:0
led 3 9,11::CTO:0

# master
set blackbox_device = SPIFLASH
set dshot_burst = ON
set current_meter = ADC
set battery_meter = ADC
set ibata_scale = 230
set ibata_offset = 10
set small_angle = 180
set max7456_spi_bus = 3
set flash_spi_bus = 1
set gyro_1_bustype = SPI
set gyro_1_spibus = 4
set gyro_1_sensor_align = CW90
set gyro_1_align_yaw = 900
set gyro_2_spibus = 2

*/
