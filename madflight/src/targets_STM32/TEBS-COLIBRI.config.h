//==============================================================================
// Generated on: 2023-12-28 01:17:04.544113
// Generated by: _convert.py in this directory
// Source: https://github.com/betaflight/unified-targets
// Board name: COLIBRI
// Manufacturer ID: TEBS
//==============================================================================
#define USE_IMU_SPI_MPU6000
#define USE_BARO_MS5611
#define USE_FLASH_M25P16

//LED:
const int HW_PIN_LED      = C14;
#define LED_ON 0 //low = on

//Battery voltage divider:
const int HW_PIN_BAT_ADC  = -1;
const int HW_PIN_BAT_CURR = -1;

//GPS: (SERIAL1)
const int HW_PIN_GPS_RX   = B07;
const int HW_PIN_GPS_TX   = B06;
HardwareSerial gps_Serial(HW_PIN_GPS_RX, HW_PIN_GPS_TX);

//RC Receiver: (SERIAL3)
const int HW_PIN_RCIN_RX  = B11;
const int HW_PIN_RCIN_TX  = B10;
HardwareSerial *rcin_Serial = new HardwareSerial(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);

//IMU:
const int HW_PIN_IMU_INT  = C00;

//I2C: (I2C3)
const int HW_PIN_I2C_SDA  = A08;
const int HW_PIN_I2C_SCL  = C09;
typedef TwoWire HW_WIRETYPE; //define the class to use for I2C
HW_WIRETYPE *i2c = &Wire; //&Wire or &Wire1

//SPI: (SPI1)
const int HW_PIN_SPI_MISO = A06;
const int HW_PIN_SPI_CS   = C04;
const int HW_PIN_SPI_SCLK = A05;
const int HW_PIN_SPI_MOSI = A07;
SPIClass *spi = &SPI;

//Outputs:
#define HW_OUT_COUNT 8
const int8_t HW_PIN_OUT[HW_OUT_COUNT] = {PB0,PB4,PB1,PB15,PB5,PB14,PB8,PB9};


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F405 (S405) 4.2.0 Feb 11 2020 / 19:14:12 (norevision) MSP API: 1.43
#mcu STM32F405

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_BARO
#define USE_BARO_MS5611
#define USE_FLASH
#define USE_FLASH_M25P16

board_name COLIBRI
manufacturer_id TEBS

# resources
resource BEEPER 1 C05
resource MOTOR 1 B00
resource MOTOR 2 B04
resource MOTOR 3 B01
resource MOTOR 4 B15
resource MOTOR 5 B05
resource MOTOR 6 B14
resource MOTOR 7 B08
resource MOTOR 8 B09
resource PPM 1 A10
resource PWM 1 A10
resource PWM 2 C06
resource PWM 3 C07
resource PWM 4 C08
resource PWM 5 A15
resource PWM 6 B03
resource PWM 7 A00
resource PWM 8 A01
resource LED_STRIP 1 B07
resource SERIAL_TX 1 B06
resource SERIAL_TX 2 A02
resource SERIAL_TX 3 B10
resource SERIAL_RX 1 B07
resource SERIAL_RX 2 A03
resource SERIAL_RX 3 B11
resource INVERTER 2 B02
resource I2C_SCL 3 A08
resource I2C_SDA 3 C09
resource LED 1 C14
resource LED 2 C13
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_MISO 1 A06
resource SPI_MISO 2 C02
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 C03
resource ESCSERIAL 1 A10
resource COMPASS_EXTI 1 C01
resource FLASH_CS 1 B12
resource GYRO_EXTI 1 C00
resource GYRO_CS 1 C04
resource USB_DETECT 1 A09

# timer
timer A10 AF1
# pin A10: TIM1 CH3 (AF1)
timer C06 AF3
# pin C06: TIM8 CH1 (AF3)
timer C07 AF3
# pin C07: TIM8 CH2 (AF3)
timer C08 AF3
# pin C08: TIM8 CH3 (AF3)
timer A15 AF1
# pin A15: TIM2 CH1 (AF1)
timer B03 AF1
# pin B03: TIM2 CH2 (AF1)
timer A00 AF2
# pin A00: TIM5 CH1 (AF2)
timer A01 AF2
# pin A01: TIM5 CH2 (AF2)
timer B00 AF2
# pin B00: TIM3 CH3 (AF2)
timer B04 AF2
# pin B04: TIM3 CH1 (AF2)
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer B15 AF9
# pin B15: TIM12 CH2 (AF9)
timer B05 AF2
# pin B05: TIM3 CH2 (AF2)
timer B14 AF9
# pin B14: TIM12 CH1 (AF9)
timer B08 AF3
# pin B08: TIM10 CH1 (AF3)
timer B09 AF3
# pin B09: TIM11 CH1 (AF3)
timer B07 AF2
# pin B07: TIM4 CH2 (AF2)

# dma
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin A10 0
# pin A10: DMA2 Stream 6 Channel 0
dma pin C06 0
# pin C06: DMA2 Stream 2 Channel 0
dma pin C07 0
# pin C07: DMA2 Stream 2 Channel 0
dma pin C08 0
# pin C08: DMA2 Stream 2 Channel 0
dma pin A15 0
# pin A15: DMA1 Stream 5 Channel 3
dma pin B03 0
# pin B03: DMA1 Stream 6 Channel 3
dma pin A00 0
# pin A00: DMA1 Stream 2 Channel 6
dma pin A01 0
# pin A01: DMA1 Stream 4 Channel 6
dma pin B00 0
# pin B00: DMA1 Stream 7 Channel 5
dma pin B04 0
# pin B04: DMA1 Stream 4 Channel 5
dma pin B01 0
# pin B01: DMA1 Stream 2 Channel 5
dma pin B05 0
# pin B05: DMA1 Stream 5 Channel 5
dma pin B07 0
# pin B07: DMA1 Stream 3 Channel 2

# master
set mag_bustype = I2C
set mag_i2c_device = 3
set baro_bustype = I2C
set baro_i2c_device = 3
set blackbox_device = SPIFLASH
set system_hse_mhz = 16
set dashboard_i2c_bus = 3
set flash_spi_bus = 2
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW270FLIP
set gyro_1_align_pitch = 1800
set gyro_1_align_yaw = 2700

*/
