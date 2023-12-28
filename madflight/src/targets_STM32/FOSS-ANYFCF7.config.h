//==============================================================================
// Generated on: 2023-12-28 01:17:04.386964
// Generated by: _convert.py in this directory
// Source: https://github.com/betaflight/unified-targets
// Board name: ANYFCF7
// Manufacturer ID: FOSS
//==============================================================================
#define USE_IMU_SPI_MPU6000
#define USE_BARO_MS5611
#define USE_MAX7456
#define USE_SDCARD

//LED:
const int HW_PIN_LED      = B07;
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
const int HW_PIN_IMU_INT  = C04;

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
const int8_t HW_PIN_OUT[HW_OUT_COUNT] = {PB8,PA2,PA1,PA3,PB5,PA0,PB9,PE6};


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F745 (S745) 4.2.0 Feb  2 2020 / 16:59:28 (norevision) MSP API: 1.43

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_BARO
#define USE_BARO_MS5611
#define USE_MAX7456
#define USE_SDCARD

board_name ANYFCF7
manufacturer_id FOSS

# resources
resource BEEPER 1 B02
resource MOTOR 1 B08
resource MOTOR 2 A02
resource MOTOR 3 A01
resource MOTOR 4 A03
resource MOTOR 5 B05
resource MOTOR 6 A00
resource MOTOR 7 B09
resource MOTOR 8 E06
resource PPM 1 B14
resource PWM 1 B14
resource PWM 2 B15
resource PWM 3 C06
resource PWM 4 C07
resource PWM 5 C08
resource PWM 6 C09
resource LED_STRIP 1 B03
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 D05
resource SERIAL_TX 3 D08
resource SERIAL_TX 4 C10
resource SERIAL_TX 5 C12
resource SERIAL_TX 6 C06
resource SERIAL_TX 7 E08
resource SERIAL_TX 8 E01
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 D06
resource SERIAL_RX 3 D09
resource SERIAL_RX 4 C11
resource SERIAL_RX 5 D02
resource SERIAL_RX 6 C07
resource SERIAL_RX 7 E07
resource SERIAL_RX 8 E00
resource I2C_SCL 2 B10
resource I2C_SCL 4 D12
resource I2C_SDA 2 B11
resource I2C_SDA 4 D13
resource LED 1 B07
resource LED 2 B06
resource SPI_SCK 1 A05
resource SPI_SCK 3 C10
resource SPI_SCK 4 E12
resource SPI_MISO 1 A06
resource SPI_MISO 3 C11
resource SPI_MISO 4 E13
resource SPI_MOSI 1 A07
resource SPI_MOSI 3 C12
resource SPI_MOSI 4 E14
resource ADC_BATT 1 C00
resource ADC_RSSI 1 C02
resource ADC_CURR 1 C01
resource SDCARD_CS 1 E11
resource SDCARD_DETECT 1 D03
resource OSD_CS 1 D02
resource GYRO_EXTI 1 C04
resource GYRO_CS 1 A04
resource USB_DETECT 1 A08

# timer
timer B14 AF9
# pin B14: TIM12 CH1 (AF9)
timer B15 AF9
# pin B15: TIM12 CH2 (AF9)
timer C06 AF3
# pin C06: TIM8 CH1 (AF3)
timer C07 AF3
# pin C07: TIM8 CH2 (AF3)
timer C08 AF3
# pin C08: TIM8 CH3 (AF3)
timer C09 AF3
# pin C09: TIM8 CH4 (AF3)
timer B08 AF2
# pin B08: TIM4 CH3 (AF2)
timer A02 AF2
# pin A02: TIM5 CH3 (AF2)
timer A01 AF2
# pin A01: TIM5 CH2 (AF2)
timer A03 AF2
# pin A03: TIM5 CH4 (AF2)
timer B05 AF2
# pin B05: TIM3 CH2 (AF2)
timer A00 AF2
# pin A00: TIM5 CH1 (AF2)
timer B09 AF2
# pin B09: TIM4 CH4 (AF2)
timer E06 AF3
# pin E06: TIM9 CH2 (AF3)
timer B03 AF1
# pin B03: TIM2 CH2 (AF1)
timer B04 AF2
# pin B04: TIM3 CH1 (AF2)

# dma
dma SPI_TX 4 0
# SPI_TX 4: DMA2 Stream 1 Channel 4
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin C06 0
# pin C06: DMA2 Stream 2 Channel 0
dma pin C07 1
# pin C07: DMA2 Stream 3 Channel 7
dma pin C08 1
# pin C08: DMA2 Stream 4 Channel 7
dma pin C09 0
# pin C09: DMA2 Stream 7 Channel 7
dma pin B08 0
# pin B08: DMA1 Stream 7 Channel 2
dma pin A02 0
# pin A02: DMA1 Stream 0 Channel 6
dma pin A01 0
# pin A01: DMA1 Stream 4 Channel 6
dma pin A03 0
# pin A03: DMA1 Stream 1 Channel 6
dma pin B05 0
# pin B05: DMA1 Stream 5 Channel 5
dma pin A00 0
# pin A00: DMA1 Stream 2 Channel 6
dma pin B03 0
# pin B03: DMA1 Stream 6 Channel 3
dma pin B04 0
# pin B04: DMA1 Stream 4 Channel 5

# feature
feature OSD

# master
set mag_bustype = I2C
set mag_i2c_device = 2
set baro_bustype = I2C
set baro_i2c_device = 2
set blackbox_device = SDCARD
set beeper_inversion = ON
set beeper_od = OFF
set sdcard_detect_inverted = ON
set sdcard_mode = SPI
set sdcard_spi_bus = 4
set max7456_spi_bus = 3
set dashboard_i2c_bus = 2
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW270
set gyro_1_align_yaw = 2700

*/
