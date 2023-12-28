//==============================================================================
// Generated on: 2023-12-28 01:17:04.308448
// Generated by: _convert.py in this directory
// Source: https://github.com/betaflight/unified-targets
// Board name: AG3XF4
// Manufacturer ID: AIRB
//==============================================================================
#define USE_IMU_SPI_MPU6500
#define USE_IMU_SPI_MPU6000
#define USE_BARO_BMP280
#define USE_MAX7456

//LED:
const int HW_PIN_LED      = C13;
#define LED_ON 0 //low = on

//Battery voltage divider:
const int HW_PIN_BAT_ADC  = C00;
const int HW_PIN_BAT_CURR = C01;

//GPS: (SERIAL1)
const int HW_PIN_GPS_RX   = A10;
const int HW_PIN_GPS_TX   = A09;
HardwareSerial gps_Serial(HW_PIN_GPS_RX, HW_PIN_GPS_TX);

//RC Receiver: (SERIAL3)
const int HW_PIN_RCIN_RX  = C11;
const int HW_PIN_RCIN_TX  = C10;
HardwareSerial *rcin_Serial = new HardwareSerial(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);

//IMU:
const int HW_PIN_IMU_INT  = -1;

//I2C: (I2C2)
const int HW_PIN_I2C_SDA  = B10;
const int HW_PIN_I2C_SCL  = B11;
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
const int8_t HW_PIN_OUT[HW_OUT_COUNT] = {PC8,PB0,PB1,PB7,PB8,PC9,PB14,PB15};


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F405 (S405) 4.2.0 Feb  2 2020 / 13:54:52 (09e8234117) MSP API: 1.43
#mcu STM32F405

#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_GYRO_SPI_MPU6000
#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_ACC_SPI_MPU6000
#define USE_BARO
#define USE_BARO_BMP280
#define USE_MAX7456

board_name AG3XF4
manufacturer_id AIRB

# resources
resource MOTOR 1 C08
resource MOTOR 2 B00
resource MOTOR 3 B01
resource MOTOR 4 B07
resource MOTOR 5 B08
resource MOTOR 6 C09
resource MOTOR 7 B14
resource MOTOR 8 B15
resource PPM 1 A08
resource LED_STRIP 1 A02
resource SERIAL_TX 1 A09
resource SERIAL_TX 3 C10
resource SERIAL_TX 5 C12
resource SERIAL_TX 6 C06
resource SERIAL_RX 1 A10
resource SERIAL_RX 3 C11
resource SERIAL_RX 4 A01
resource SERIAL_RX 5 D02
resource SERIAL_RX 6 C07
resource I2C_SCL 2 B10
resource I2C_SDA 2 B11
resource LED 1 C13
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 B03
resource SPI_MISO 1 A06
resource SPI_MISO 2 C02
resource SPI_MISO 3 B04
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 C03
resource SPI_MOSI 3 B05
resource ESCSERIAL 1 A08
resource CAMERA_CONTROL 1 A03
resource ADC_BATT 1 C00
resource ADC_RSSI 1 C04
resource ADC_CURR 1 C01
resource ADC_EXT 1 A00
resource BARO_CS 1 B09
resource FLASH_CS 1 B12
resource OSD_CS 1 A15
resource GYRO_CS 1 A04
resource GYRO_CS 2 C15

# timer
timer A08 AF1
# pin A08: TIM1 CH1 (AF1)
timer C08 AF3
# pin C08: TIM8 CH3 (AF3)
timer B00 AF2
# pin B00: TIM3 CH3 (AF2)
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer B07 AF2
# pin B07: TIM4 CH2 (AF2)
timer B08 AF2
# pin B08: TIM4 CH3 (AF2)
timer C09 AF3
# pin C09: TIM8 CH4 (AF3)
timer B14 AF9
# pin B14: TIM12 CH1 (AF9)
timer B15 AF9
# pin B15: TIM12 CH2 (AF9)
timer A09 AF1
# pin A09: TIM1 CH2 (AF1)
timer A10 AF1
# pin A10: TIM1 CH3 (AF1)
timer A02 AF2
# pin A02: TIM5 CH3 (AF2)
timer A03 AF3
# pin A03: TIM9 CH2 (AF3)
timer B10 AF1
# pin B10: TIM2 CH3 (AF1)
timer B11 AF1
# pin B11: TIM2 CH4 (AF1)

# dma
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin A08 0
# pin A08: DMA2 Stream 6 Channel 0
dma pin C08 0
# pin C08: DMA2 Stream 2 Channel 0
dma pin B00 0
# pin B00: DMA1 Stream 7 Channel 5
dma pin B01 0
# pin B01: DMA1 Stream 2 Channel 5
dma pin B07 0
# pin B07: DMA1 Stream 3 Channel 2
dma pin B08 0
# pin B08: DMA1 Stream 7 Channel 2
dma pin C09 0
# pin C09: DMA2 Stream 7 Channel 7
dma pin A09 1
# pin A09: DMA2 Stream 2 Channel 6
dma pin A10 0
# pin A10: DMA2 Stream 6 Channel 0
dma pin A02 0
# pin A02: DMA1 Stream 0 Channel 6
dma pin B10 0
# pin B10: DMA1 Stream 1 Channel 3
dma pin B11 0
# pin B11: DMA1 Stream 7 Channel 3

# feature
feature OSD

# master
set mag_bustype = I2C
set mag_i2c_device = 2
set baro_spi_device = 2
set blackbox_device = SPIFLASH
set dshot_burst = ON
set current_meter = ESC
set battery_meter = ADC
set beeper_inversion = ON
set beeper_od = OFF
set system_hse_mhz = 8
set max7456_spi_bus = 3
set dashboard_i2c_bus = 2
set flash_spi_bus = 2
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW270FLIP
set gyro_1_align_pitch = 1800
set gyro_1_align_yaw = 2700
set gyro_2_spibus = 1
set gyro_2_sensor_align = CW0FLIP
set gyro_2_align_pitch = 1800

*/
