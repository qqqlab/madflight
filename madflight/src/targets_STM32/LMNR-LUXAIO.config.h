//==============================================================================
// Generated on: 2023-12-28 01:17:04.465967
// Generated by: _convert.py in this directory
// Source: https://github.com/betaflight/unified-targets
// Board name: LUXAIO
// Manufacturer ID: LMNR
//==============================================================================
#define USE_IMU_SPI_MPU6000
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

//LED:
const int HW_PIN_LED      = C14;
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
const int HW_PIN_IMU_INT  = A04;

//I2C: (I2C1)
const int HW_PIN_I2C_SDA  = B08;
const int HW_PIN_I2C_SCL  = B09;
typedef TwoWire HW_WIRETYPE; //define the class to use for I2C
HW_WIRETYPE *i2c = &Wire; //&Wire or &Wire1

//SPI: (SPI2)
const int HW_PIN_SPI_MISO = B14;
const int HW_PIN_SPI_CS   = B12;
const int HW_PIN_SPI_SCLK = B13;
const int HW_PIN_SPI_MOSI = B15;
SPIClass *spi = &SPI;

//Outputs:
#define HW_OUT_COUNT 4
const int8_t HW_PIN_OUT[HW_OUT_COUNT] = {PC9,PB7,PC8,PB6};


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F7X2 (S7X2) 4.2.9 Apr 27 2021 / 19:34:29 (e097f4ab7) MSP API: 1.43

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

board_name LUXAIO
manufacturer_id LMNR

# resources
resource BEEPER 1 B00
resource MOTOR 1 C09
resource MOTOR 2 B07
resource MOTOR 3 C08
resource MOTOR 4 B06
resource LED_STRIP 1 A15
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 A02
resource SERIAL_TX 3 C10
resource SERIAL_TX 4 A00
resource SERIAL_TX 5 C12
resource SERIAL_TX 6 C06
resource SERIAL_RX 1 A10
resource SERIAL_RX 3 C11
resource SERIAL_RX 4 A01
resource SERIAL_RX 6 C07
resource I2C_SCL 1 B08
resource I2C_SDA 1 B09
resource LED 1 C14
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 B03
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 B04
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 B05
resource CAMERA_CONTROL 1 A08
resource ADC_BATT 1 C00
resource ADC_CURR 1 C01
resource PINIO 2 B11
resource FLASH_CS 1 A03
resource OSD_CS 1 D02
resource GYRO_EXTI 1 A04
resource GYRO_CS 1 B12

# timer
timer A15 AF1
# pin A15: TIM2 CH1 (AF1)
timer A08 AF1
# pin A08: TIM1 CH1 (AF1)
timer C09 AF3
# pin C09: TIM8 CH4 (AF3)
timer B07 AF2
# pin B07: TIM4 CH2 (AF2)
timer C08 AF3
# pin C08: TIM8 CH3 (AF3)
timer B06 AF2
# pin B06: TIM4 CH1 (AF2)

# dma
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin A15 0
# pin A15: DMA1 Stream 5 Channel 3
dma pin A08 0
# pin A08: DMA2 Stream 6 Channel 0
dma pin C09 0
# pin C09: DMA2 Stream 7 Channel 7
dma pin B07 0
# pin B07: DMA1 Stream 3 Channel 2
dma pin C08 0
# pin C08: DMA2 Stream 2 Channel 0
dma pin B06 0
# pin B06: DMA1 Stream 0 Channel 2

# feature
feature TELEMETRY
feature OSD

# master
set baro_bustype = I2C
set baro_i2c_device = 1
set blackbox_device = SPIFLASH
set dshot_burst = ON
set current_meter = ADC
set battery_meter = ADC
set ibata_scale = 163
set beeper_inversion = ON
set beeper_od = OFF
set max7456_spi_bus = 3
set flash_spi_bus = 1
set gyro_1_bustype = SPI
set gyro_1_spibus = 2
set gyro_1_sensor_align = CW90FLIP
set gyro_1_align_pitch = 1800
set gyro_1_align_yaw = 900
set pinio_config = 1,129,1,1
set pinio_box = 255,40,255,255

*/
