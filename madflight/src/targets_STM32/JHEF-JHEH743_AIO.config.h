//==============================================================================
// Generated on: 2023-12-28 01:17:04.465967
// Generated by: _convert.py in this directory
// Source: https://github.com/betaflight/unified-targets
// Board name: JHEH743_AIO
// Manufacturer ID: JHEF
//==============================================================================
#define USE_IMU_SPI_MPU6000
#define USE_BARO_BMP280
#define USE_MAX7456
#define USE_FLASH_M25P16

//LED:
const int HW_PIN_LED      = C13;
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
const int HW_PIN_IMU_INT  = D00;

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
const int8_t HW_PIN_OUT[HW_OUT_COUNT] = {PE9,PB0,PE11,PB1};


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32H743 (SH47) 4.3.0 Sep  2 2021 / 06:34:37 (c23dd47f4) MSP API: 1.44

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_BARO
#define USE_BARO_BMP280
#define USE_MAX7456
#define USE_FLASH
#define USE_FLASH_M25P16

board_name JHEH743_AIO
manufacturer_id JHEF

# resources
resource BEEPER 1 D15
resource MOTOR 1 E09
resource MOTOR 2 B00
resource MOTOR 3 E11
resource MOTOR 4 B01
resource PPM 1 A03
resource LED_STRIP 1 D12
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 A02
resource SERIAL_TX 3 B10
resource SERIAL_TX 4 A00
resource SERIAL_TX 6 C06
resource SERIAL_TX 7 E08
resource SERIAL_TX 8 E01
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 A03
resource SERIAL_RX 3 B11
resource SERIAL_RX 4 A01
resource SERIAL_RX 6 C07
resource SERIAL_RX 7 E07
resource SERIAL_RX 8 E00
resource I2C_SCL 1 B08
resource I2C_SCL 2 B10
resource I2C_SDA 1 B09
resource I2C_SDA 2 B11
resource LED 1 C13
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 C10
resource SPI_SCK 4 E02
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 C11
resource SPI_MISO 4 E05
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 C12
resource SPI_MOSI 4 E06
resource CAMERA_CONTROL 1 C08
resource ADC_BATT 1 C03
resource ADC_RSSI 1 C05
resource ADC_CURR 1 C02
resource ADC_EXT 1 C01
resource FLASH_CS 1 A15
resource OSD_CS 1 E04
resource GYRO_EXTI 1 D00
resource GYRO_EXTI 2 D08
resource GYRO_CS 1 A04
resource GYRO_CS 2 B12

# timer
timer A08 AF1
# pin A08: TIM1 CH1 (AF1)
timer B03 AF1
# pin B03: TIM2 CH2 (AF1)
timer B00 AF2
# pin B00: TIM3 CH3 (AF2)
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer D12 AF2
# pin D12: TIM4 CH1 (AF2)
timer D13 AF2
# pin D13: TIM4 CH2 (AF2)
timer C08 AF3
# pin C08: TIM8 CH3 (AF3)
timer C09 AF3
# pin C09: TIM8 CH4 (AF3)
timer A00 AF2
# pin A00: TIM5 CH1 (AF2)
timer A01 AF2
# pin A01: TIM5 CH2 (AF2)
timer A02 AF2
# pin A02: TIM5 CH3 (AF2)
timer A03 AF1
# pin A03: TIM2 CH4 (AF1)
timer B10 AF1
# pin B10: TIM2 CH3 (AF1)
timer B11 AF1
# pin B11: TIM2 CH4 (AF1)
timer E09 AF1
# pin E09: TIM1 CH1 (AF1)
timer E11 AF1
# pin E11: TIM1 CH2 (AF1)


# dma
dma ADC 1 8
# ADC 1: DMA2 Stream 0 Request 9
dma ADC 3 9
# ADC 3: DMA2 Stream 1 Request 115
dma TIMUP 1 0
# TIMUP 1: DMA1 Stream 0 Request 15
dma TIMUP 2 0
# TIMUP 2: DMA1 Stream 0 Request 22
dma TIMUP 3 0
# TIMUP 3: DMA1 Stream 0 Request 27
dma TIMUP 4 0
# TIMUP 4: DMA1 Stream 0 Request 32
dma TIMUP 8 0
# TIMUP 8: DMA1 Stream 0 Request 51
dma pin A08 10
# pin A08: DMA2 Stream 2 Request 11
dma pin B03 0
# pin B03: DMA1 Stream 0 Request 19
dma pin B00 0
# pin B00: DMA1 Stream 0 Request 25
dma pin B01 1
# pin B01: DMA1 Stream 1 Request 26
dma pin D12 10
# pin D12: DMA2 Stream 2 Request 29
dma pin D13 5
# pin D13: DMA1 Stream 5 Request 30
dma pin C08 0
# pin C08: DMA1 Stream 0 Request 49
dma pin C09 7
# pin C09: DMA1 Stream 7 Request 50
dma pin A00 0
# pin A00: DMA1 Stream 0 Request 55
dma pin A01 0
# pin A01: DMA1 Stream 0 Request 56
dma pin A02 0
# pin A02: DMA1 Stream 0 Request 57
dma pin A03 0
# pin A03: DMA1 Stream 0 Request 21
dma pin B10 0
# pin B10: DMA1 Stream 0 Request 20
dma pin B11 0
# pin B11: DMA1 Stream 0 Request 21
dma pin E09 2
# pin E09: DMA1 Stream 2 Request 11
dma pin E11 3
# pin E11: DMA1 Stream 3 Request 12
# feature
feature RX_SERIAL
feature OSD

# master
set mag_bustype = I2C
set mag_i2c_device = 1
set mag_hardware = NONE
set baro_bustype = I2C
set baro_i2c_device = 1
set blackbox_device = SPIFLASH
set current_meter = ADC
set battery_meter = ADC
set ibata_scale = 120
set beeper_inversion = ON
set beeper_od = OFF
set max7456_spi_bus = 4
set flash_spi_bus = 3
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_2_spibus = 2
set motor_pwm_protocol = DSHOT600

*/
