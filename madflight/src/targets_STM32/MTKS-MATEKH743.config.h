//==============================================================================
// Generated on: 2023-12-28 01:17:04.481604
// Generated by: _convert.py in this directory
// Source: https://github.com/betaflight/unified-targets
// Board name: MATEKH743
// Manufacturer ID: MTKS
//==============================================================================
#define USE_IMU_SPI_MPU6500
#define USE_IMU_SPI_MPU6000
#define USE_IMU_SPI_ICM42688P
#define USE_IMU_SPI_ICM42605
#define USE_BARO_BMP280
#define USE_BARO_DPS310
#define USE_MAX7456
#define USE_SDCARD

//LED:
const int HW_PIN_LED      = E03;
#define LED_ON 0 //low = on

//Battery voltage divider:
const int HW_PIN_BAT_ADC  = C00;
const int HW_PIN_BAT_CURR = C01;

//GPS: (SERIAL1)
const int HW_PIN_GPS_RX   = A10;
const int HW_PIN_GPS_TX   = A09;
HardwareSerial gps_Serial(HW_PIN_GPS_RX, HW_PIN_GPS_TX);

//RC Receiver: (SERIAL3)
const int HW_PIN_RCIN_RX  = D09;
const int HW_PIN_RCIN_TX  = D08;
HardwareSerial *rcin_Serial = new HardwareSerial(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);

//IMU:
const int HW_PIN_IMU_INT  = B02;

//I2C: (I2C2)
const int HW_PIN_I2C_SDA  = B10;
const int HW_PIN_I2C_SCL  = B11;
typedef TwoWire HW_WIRETYPE; //define the class to use for I2C
HW_WIRETYPE *i2c = &Wire; //&Wire or &Wire1

//SPI: (SPI1)
const int HW_PIN_SPI_MISO = A06;
const int HW_PIN_SPI_CS   = C15;
const int HW_PIN_SPI_SCLK = A05;
const int HW_PIN_SPI_MOSI = D07;
SPIClass *spi = &SPI;

//Outputs:
#define HW_OUT_COUNT 8
const int8_t HW_PIN_OUT[HW_OUT_COUNT] = {PB0,PB1,PA0,PA1,PA2,PA3,PD12,PD13};


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32H743 (SH74) 4.3.0 Jan 16 2022 / 12:56:16 (ee740d1) MSP API: 1.44

#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_ACC_SPI_MPU6000
#define USE_GYRO_SPI_MPU6000
#define USE_ACC_SPI_ICM42688P
#define USE_GYRO_SPI_ICM42688P
#define USE_ACC_SPI_ICM42605
#define USE_GYRO_SPI_ICM42605
#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_DPS310
#define USE_MAX7456
#define USE_SDCARD

board_name MATEKH743
manufacturer_id MTKS

# resources
resource BEEPER 1 A15
resource MOTOR 1 B00
resource MOTOR 2 B01
resource MOTOR 3 A00
resource MOTOR 4 A01
resource MOTOR 5 A02
resource MOTOR 6 A03
resource MOTOR 7 D12
resource MOTOR 8 D13
resource SERVO 1 E05
resource SERVO 2 E06
resource PPM 1 C07
resource LED_STRIP 1 A08
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 D05
resource SERIAL_TX 3 D08
resource SERIAL_TX 4 B09
resource SERIAL_TX 6 C06
resource SERIAL_TX 7 E08
resource SERIAL_TX 8 E01
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 D06
resource SERIAL_RX 3 D09
resource SERIAL_RX 4 B08
resource SERIAL_RX 6 C07
resource SERIAL_RX 7 E07
resource SERIAL_RX 8 E00
resource I2C_SCL 1 B06
resource I2C_SCL 2 B10
resource I2C_SDA 1 B07
resource I2C_SDA 2 B11
resource LED 1 E03
resource LED 2 E04
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 B03
resource SPI_SCK 4 E12
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 B04
resource SPI_MISO 4 E13
resource SPI_MOSI 1 D07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 B05
resource SPI_MOSI 4 E14
resource ADC_BATT 1 C00
resource ADC_RSSI 1 C05
resource ADC_CURR 1 C01
resource ADC_EXT 1 C04
resource SDIO_CK 1 C12
resource SDIO_CMD 1 D02
resource SDIO_D0 1 C08
resource SDIO_D1 1 C09
resource SDIO_D2 1 C10
resource SDIO_D3 1 C11
resource PINIO 1 D10
resource PINIO 2 D11
resource OSD_CS 1 B12
resource GYRO_EXTI 1 B02
resource GYRO_EXTI 2 E15
resource GYRO_CS 1 C15
resource GYRO_CS 2 E11

# CS1/CS2 pads for SPI3 connection
# CS1 D04
# CS2 E02

# timer
timer B00 AF2
# pin B00: TIM3 CH3 (AF2)
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer A00 AF2
# pin A00: TIM5 CH1 (AF2)
timer A01 AF2
# pin A01: TIM5 CH2 (AF2)
timer A02 AF2
# pin A02: TIM5 CH3 (AF2)
timer A03 AF2
# pin A03: TIM5 CH4 (AF2)
timer D12 AF2
# pin D12: TIM4 CH1 (AF2)
timer D13 AF2
# pin D13: TIM4 CH2 (AF2)
timer D14 AF2
# pin D14: TIM4 CH3 (AF2)
timer D15 AF2
# pin D15: TIM4 CH4 (AF2)
timer E05 AF4
# pin E05: TIM15 CH1 (AF4)
timer E06 AF4
# pin E06: TIM15 CH2 (AF4)
timer A08 AF1
# pin A08: TIM1 CH1 (AF1)
timer A15 AF1
# pin A15: TIM2 CH1 (AF1)
timer C07 AF3
# pin C07: TIM8 CH2 (AF3)
timer C06 AF3
# pin C06: TIM8 CH1 (AF3)
timer B08 AF1
# pin B08: TIM16 CH1 (AF1)
timer B09 AF1
# pin B09: TIM17 CH1 (AF1)

# dma
dma ADC 1 8
# ADC 1: DMA2 Stream 0 Request 9
dma ADC 3 9
# ADC 3: DMA2 Stream 1 Request 115
dma TIMUP 1 0
# TIMUP 1: DMA1 Stream 0 Request 15
dma TIMUP 2 0
# TIMUP 2: DMA1 Stream 0 Request 22
dma TIMUP 3 2
# TIMUP 3: DMA1 Stream 2 Request 27
dma TIMUP 4 1
# TIMUP 4: DMA1 Stream 1 Request 32
dma TIMUP 5 0
# TIMUP 5: DMA1 Stream 0 Request 59
dma TIMUP 8 0
# TIMUP 8: DMA1 Stream 0 Request 51
dma pin B00 0
# pin B00: DMA1 Stream 0 Request 25
dma pin B01 1
# pin B01: DMA1 Stream 1 Request 26
dma pin A00 2
# pin A00: DMA1 Stream 2 Request 55
dma pin A01 3
# pin A01: DMA1 Stream 3 Request 56
dma pin A02 4
# pin A02: DMA1 Stream 4 Request 57
dma pin A03 5
# pin A03: DMA1 Stream 5 Request 58
dma pin D12 6
# pin D12: DMA1 Stream 6 Request 29
dma pin D13 7
# pin D13: DMA1 Stream 7 Request 30
dma pin D14 12
# pin D14: DMA2 Stream 4 Request 31
dma pin E05 0
# pin E05: DMA1 Stream 0 Request 105
dma pin A08 14
# pin A08: DMA2 Stream 6 Request 11
dma pin A15 0
# pin A15: DMA1 Stream 0 Request 18
dma pin C07 0
# pin C07: DMA1 Stream 0 Request 48
dma pin C06 0
# pin C06: DMA1 Stream 0 Request 47
dma pin B08 0
# pin B08: DMA1 Stream 0 Request 109
dma pin B09 0
# pin B09: DMA1 Stream 0 Request 111

# feature
feature RX_SERIAL
feature TELEMETRY
feature OSD

# serial
serial 5 64 115200 57600 0 115200

# master
set mag_bustype = I2C
set mag_i2c_device = 1
set baro_bustype = I2C
set baro_i2c_device = 2
set serialrx_provider = SBUS
set blackbox_device = SDCARD
set current_meter = ADC
set battery_meter = ADC
set ibata_scale = 250
set beeper_inversion = ON
set beeper_od = OFF
set beeper_frequency = 2500
set sdio_use_4bit_width = ON
set sdio_device = 1
set max7456_spi_bus = 2
set pinio_box = 40,41,255,255
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW0FLIP
set gyro_1_align_pitch = 1800
set gyro_2_spibus = 4
set gyro_2_sensor_align = CW0FLIP
set gyro_2_align_pitch = 1800

*/
