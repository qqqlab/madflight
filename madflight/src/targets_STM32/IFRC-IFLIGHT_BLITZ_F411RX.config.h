//==============================================================================
// Generated on: 2023-12-28 01:17:04.449955
// Generated by: _convert.py in this directory
// Source: https://github.com/betaflight/unified-targets
// Board name: IFLIGHT_BLITZ_F411RX
// Manufacturer ID: IFRC
//==============================================================================
#define USE_MAX7456

//LED:
const int HW_PIN_LED      = C13;
#define LED_ON 0 //low = on

//Battery voltage divider:
const int HW_PIN_BAT_ADC  = B00;
const int HW_PIN_BAT_CURR = B01;

//GPS: (SERIAL1)
const int HW_PIN_GPS_RX   = -1;
const int HW_PIN_GPS_TX   = A09;
HardwareSerial gps_Serial(HW_PIN_GPS_RX, HW_PIN_GPS_TX);

//RC Receiver: (SERIAL3)
const int HW_PIN_RCIN_RX  = -1;
const int HW_PIN_RCIN_TX  = -1;
HardwareSerial *rcin_Serial = new HardwareSerial(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);

//IMU:
const int HW_PIN_IMU_INT  = B10;

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
const int8_t HW_PIN_OUT[HW_OUT_COUNT] = {PA0,PA1,PB6,PB7};


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F411 (S411) 4.2.11 Nov  9 2021 / 20:28:23 (948ba6339) MSP API: 1.43

#define USE_ACC
#define USE_GYRO
#define USE_ACCGYRO_BMI270
#define USE_MAX7456

board_name IFLIGHT_BLITZ_F411RX
manufacturer_id IFRC

# resources
resource MOTOR 1 A00
resource MOTOR 2 A01
resource MOTOR 3 B06
resource MOTOR 4 B07
resource PPM 1 A03
resource LED_STRIP 1 A10
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 A02
resource SERIAL_RX 2 A03
resource I2C_SCL 1 B08
resource I2C_SDA 1 B09
resource LED 1 C13
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 B03
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 B04
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 B05
resource ADC_BATT 1 B00
resource ADC_CURR 1 B01
resource OSD_CS 1 B12
resource RX_SPI_CS 1 A15
resource RX_SPI_EXTI 1 C14
resource RX_SPI_BIND 1 B02
resource RX_SPI_LED 1 C15
resource RX_SPI_CC2500_TX_EN 1 A08
resource GYRO_EXTI 1 B10
resource GYRO_CS 1 A04

# timer
timer A03 AF3
# pin A03: TIM9 CH2 (AF3)
timer A00 AF1
# pin A00: TIM2 CH1 (AF1)
timer A01 AF1
# pin A01: TIM2 CH2 (AF1)
timer B06 AF2
# pin B06: TIM4 CH1 (AF2)
timer B07 AF2
# pin B07: TIM4 CH2 (AF2)
timer A10 AF1
# pin A10: TIM1 CH3 (AF1)

# dma
dma ADC 1 0
# ADC 1: DMA2 Stream 0 Channel 0
dma pin A00 0
# pin A00: DMA1 Stream 5 Channel 3
dma pin A01 0
# pin A01: DMA1 Stream 6 Channel 3
dma pin B06 0
# pin B06: DMA1 Stream 0 Channel 2
dma pin B07 0
# pin B07: DMA1 Stream 3 Channel 2
dma pin A10 0
# pin A10: DMA2 Stream 6 Channel 0

# feature
feature OSD
feature RX_SPI

# serial
serial 1 64 115200 57600 0 115200

# master
set baro_bustype = I2C
set baro_i2c_device = 1
set serialrx_provider = SBUS
set rx_spi_bus = 3
set rx_spi_led_inversion = ON
set dshot_bitbang = OFF
set current_meter = ADC
set battery_meter = ADC
set ibata_scale = 180
set system_hse_mhz = 8
set max7456_spi_bus = 2
set cc2500_spi_chip_detect = OFF
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW180
set gyro_1_align_yaw = 1800

*/
