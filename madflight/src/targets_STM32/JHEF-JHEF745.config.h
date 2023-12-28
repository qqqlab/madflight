//==============================================================================
// Generated on: 2023-12-28 01:17:04.465967
// Generated by: _convert.py in this directory
// Source: https://github.com/betaflight/unified-targets
// Board name: JHEF745
// Manufacturer ID: JHEF
//==============================================================================
#define USE_IMU_SPI_MPU6000
#define USE_BARO_BMP280
#define USE_BARO_DPS310
#define USE_MAX7456
#define USE_FLASH_M25P16

//LED:
const int HW_PIN_LED      = A02;
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
const int HW_PIN_IMU_INT  = E01;

//I2C: (I2C1)
const int HW_PIN_I2C_SDA  = B06;
const int HW_PIN_I2C_SCL  = B07;
typedef TwoWire HW_WIRETYPE; //define the class to use for I2C
HW_WIRETYPE *i2c = &Wire; //&Wire or &Wire1

//SPI: (SPI4)
const int HW_PIN_SPI_MISO = E05;
const int HW_PIN_SPI_CS   = E04;
const int HW_PIN_SPI_SCLK = E02;
const int HW_PIN_SPI_MOSI = E06;
SPIClass *spi = &SPI;

//Outputs:
#define HW_OUT_COUNT 8
const int8_t HW_PIN_OUT[HW_OUT_COUNT] = {PB0,PB1,PE9,PE11,PC9,PA3,PB4,PB5};


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F745 (S745) 4.2.0 Jun 14 2020 / 03:05:04 (8f2d21460) MSP API: 1.43

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_DPS310
#define USE_MAX7456
#define USE_FLASH
#define USE_FLASH_M25P16

board_name JHEF745
manufacturer_id JHEF

# resources
resource BEEPER 1 D15
resource MOTOR 1 B00
resource MOTOR 2 B01
resource MOTOR 3 E09
resource MOTOR 4 E11
resource MOTOR 5 C09
resource MOTOR 6 A03
resource MOTOR 7 B04
resource MOTOR 8 B05
resource PPM 1 E13
resource LED_STRIP 1 D12
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 D05
resource SERIAL_TX 3 B10
resource SERIAL_TX 4 A00
resource SERIAL_TX 5 C12
resource SERIAL_TX 6 C06
resource SERIAL_TX 7 E08
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 D06
resource SERIAL_RX 3 B11
resource SERIAL_RX 4 A01
resource SERIAL_RX 5 D02
resource SERIAL_RX 6 C07
resource SERIAL_RX 7 E07
resource I2C_SCL 1 B06
resource I2C_SDA 1 B07
resource LED 1 A02
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 4 E02
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 4 E05
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 4 E06
resource CAMERA_CONTROL 1 B03
resource ADC_BATT 1 C03
resource ADC_RSSI 1 C05
resource ADC_CURR 1 C02
resource FLASH_CS 1 A04
resource OSD_CS 1 B12
resource GYRO_EXTI 1 E01
resource GYRO_CS 1 E04
resource USB_DETECT 1 A08
resource PINIO 1 C00

# timer
timer E13 AF1
# pin E13: TIM1 CH3 (AF1)
timer B00 AF2
# pin B00: TIM3 CH3 (AF2)
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer E09 AF1
# pin E09: TIM1 CH1 (AF1)
timer E11 AF1
# pin E11: TIM1 CH2 (AF1)
timer C09 AF3
# pin C09: TIM8 CH4 (AF3)
timer A03 AF2
# pin A03: TIM5 CH4 (AF2)
timer D12 AF2
# pin D12: TIM4 CH1 (AF2)
timer B03 AF1
# pin B03: TIM2 CH2 (AF1)

# dma
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin E13 1
# pin E13: DMA2 Stream 6 Channel 6
dma pin B00 0
# pin B00: DMA1 Stream 7 Channel 5
dma pin B01 0
# pin B01: DMA1 Stream 2 Channel 5
dma pin E09 2
# pin E09: DMA2 Stream 3 Channel 6
dma pin E11 1
# pin E11: DMA2 Stream 2 Channel 6
dma pin C09 0
# pin C09: DMA2 Stream 7 Channel 7
dma pin A03 0
# pin A03: DMA1 Stream 1 Channel 6
dma pin D12 0
# pin D12: DMA1 Stream 0 Channel 2
dma pin B03 0
# pin B03: DMA1 Stream 6 Channel 3

# feature
feature RX_SERIAL
feature OSD

# master
set mag_bustype = I2C
set mag_i2c_device = 1
set baro_bustype = I2C
set baro_i2c_device = 1
set serialrx_provider = SBUS
set blackbox_device = SPIFLASH
set current_meter = ADC
set battery_meter = ADC
set ibata_scale = 275
set beeper_inversion = ON
set beeper_od = OFF
set osd_vbat_pos = 14570
set osd_ah_sbar_pos = 14542
set osd_ah_pos = 14414
set max7456_spi_bus = 2
set dashboard_i2c_bus = 1
set flash_spi_bus = 1
set gyro_1_bustype = SPI
set gyro_1_spibus = 4
set gyro_1_sensor_align = CW270

*/
