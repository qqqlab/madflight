//==============================================================================
// Generated on: 2023-12-28 01:17:04.339680
// Generated by: _convert.py in this directory
// Source: https://github.com/betaflight/unified-targets
// Board name: FURYF4OSD
// Manufacturer ID: DIAT
//==============================================================================
#define USE_IMU_SPI_ICM20689
#define USE_IMU_SPI_MPU6500
#define USE_IMU_SPI_MPU6000
#define USE_IMU_SPI_ICM42688P
#define USE_BARO_BMP280
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

//LED:
const int HW_PIN_LED      = B05;
#define LED_ON 0 //low = on

//Battery voltage divider:
const int HW_PIN_BAT_ADC  = C01;
const int HW_PIN_BAT_CURR = C03;

//GPS: (SERIAL1)
const int HW_PIN_GPS_RX   = A10;
const int HW_PIN_GPS_TX   = A09;
HardwareSerial gps_Serial(HW_PIN_GPS_RX, HW_PIN_GPS_TX);

//RC Receiver: (SERIAL3)
const int HW_PIN_RCIN_RX  = B11;
const int HW_PIN_RCIN_TX  = B10;
HardwareSerial *rcin_Serial = new HardwareSerial(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);

//IMU:
const int HW_PIN_IMU_INT  = C04;

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
#define HW_OUT_COUNT 4
const int8_t HW_PIN_OUT[HW_OUT_COUNT] = {PA3,PB0,PB1,PA2};


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F405 (S405) 4.3.0 Apr 27 2021 / 18:42:02 (3ae7e91) MSP API: 1.44

#define USE_ACC
#define USE_ACC_SPI_ICM20689
#define USE_ACC_SPI_MPU6500
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_ICM42688P
#define USE_GYRO
#define USE_GYRO_SPI_ICM20689
#define USE_GYRO_SPI_MPU6500
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_ICM42688P
#define USE_BARO
#define USE_BARO_BMP280
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

board_name FURYF4OSD
manufacturer_id DIAT

# resources
resource BEEPER 1 A08
resource MOTOR 1 A03
resource MOTOR 2 B00
resource MOTOR 3 B01
resource MOTOR 4 A02
resource PPM 1 C09
resource LED_STRIP 1 A00
resource SERIAL_TX 1 A09
resource SERIAL_TX 3 B10
resource SERIAL_TX 6 C06
resource SERIAL_RX 1 A10
resource SERIAL_RX 3 B11
resource SERIAL_RX 6 C07
resource INVERTER 1 C00
resource I2C_SCL 1 B06
resource I2C_SDA 1 B07
resource LED 1 B05
resource LED 2 B04
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 C10
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 C11
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 C12
resource ESCSERIAL 1 C09
resource CAMERA_CONTROL 1 B09
resource ADC_BATT 1 C01
resource ADC_RSSI 1 C02
resource ADC_CURR 1 C03
resource FLASH_CS 1 B03
resource OSD_CS 1 B12
resource GYRO_EXTI 1 C04
resource GYRO_CS 1 A04
resource USB_DETECT 1 C05

# timer
timer C09 AF3
# pin C09: TIM8 CH4 (AF3)
timer A03 AF1
# pin A03: TIM2 CH4 (AF1)
timer B00 AF2
# pin B00: TIM3 CH3 (AF2)
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer A02 AF1
# pin A02: TIM2 CH3 (AF1)
timer A00 AF2
# pin A00: TIM5 CH1 (AF2)
timer B09 AF3
# pin B09: TIM11 CH1 (AF3)

# dma
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin C09 0
# pin C09: DMA2 Stream 7 Channel 7
dma pin A03 1
# pin A03: DMA1 Stream 6 Channel 3
dma pin B00 0
# pin B00: DMA1 Stream 7 Channel 5
dma pin B01 0
# pin B01: DMA1 Stream 2 Channel 5
dma pin A02 0
# pin A02: DMA1 Stream 1 Channel 3
dma pin A00 0
# pin A00: DMA1 Stream 2 Channel 6

# feature
feature RX_SERIAL
feature OSD

# serial
serial 2 8192 115200 57600 0 115200

# master
set serialrx_provider = SBUS
set blackbox_device = SPIFLASH
set motor_pwm_protocol = DSHOT600
set current_meter = ADC
set battery_meter = ADC
set beeper_inversion = ON
set beeper_od = OFF
set baro_bustype = I2C
set baro_i2c_device = 1
set mag_bustype = I2C
set mag_i2c_address = 1
set system_hse_mhz = 8
set max7456_spi_bus = 2
set dashboard_i2c_bus = 1
set flash_spi_bus = 3
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW180

*/
