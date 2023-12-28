//==============================================================================
// Generated on: 2023-12-28 01:17:04.402589
// Generated by: _convert.py in this directory
// Source: https://github.com/betaflight/unified-targets
// Board name: GEPRCF722
// Manufacturer ID: GEPR
//==============================================================================
#define USE_IMU_SPI_MPU6000
#define USE_IMU_SPI_ICM42688P
#define USE_BARO_BMP280
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

//LED:
const int HW_PIN_LED      = A13;
#define LED_ON 0 //low = on

//Battery voltage divider:
const int HW_PIN_BAT_ADC  = C00;
const int HW_PIN_BAT_CURR = C01;

//GPS: (SERIAL1)
const int HW_PIN_GPS_RX   = A10;
const int HW_PIN_GPS_TX   = A09;
HardwareSerial gps_Serial(HW_PIN_GPS_RX, HW_PIN_GPS_TX);

//RC Receiver: (SERIAL3)
const int HW_PIN_RCIN_RX  = B11;
const int HW_PIN_RCIN_TX  = B10;
HardwareSerial *rcin_Serial = new HardwareSerial(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);

//IMU:
const int HW_PIN_IMU_INT  = C03;

//I2C: (I2C1)
const int HW_PIN_I2C_SDA  = B08;
const int HW_PIN_I2C_SCL  = B09;
typedef TwoWire HW_WIRETYPE; //define the class to use for I2C
HW_WIRETYPE *i2c = &Wire; //&Wire or &Wire1

//SPI: (SPI3)
const int HW_PIN_SPI_MISO = C11;
const int HW_PIN_SPI_CS   = A15;
const int HW_PIN_SPI_SCLK = C10;
const int HW_PIN_SPI_MOSI = C12;
SPIClass *spi = &SPI;

//Outputs:
#define HW_OUT_COUNT 6
const int8_t HW_PIN_OUT[HW_OUT_COUNT] = {PB4,PB5,PB0,PB1,PB6,PB7};


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32F7X2 (S7X2) 4.1.7 May 28 2020 / 15:06:20 (9ba02a587) MSP API: 1.42

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_ICM42688P
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_ICM42688P
#define USE_BARO
#define USE_BARO_BMP280
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

board_name GEPRCF722
manufacturer_id GEPR

# resources
resource BEEPER 1 C13
resource MOTOR 1 B04
resource MOTOR 2 B05
resource MOTOR 3 B00
resource MOTOR 4 B01
resource MOTOR 5 B06
resource MOTOR 6 B07
resource PPM 1 A03
resource PWM 1 A02
resource LED_STRIP 1 A08
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 A02
resource SERIAL_TX 3 B10
resource SERIAL_TX 4 A00
resource SERIAL_TX 6 C06
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 A03
resource SERIAL_RX 3 B11
resource SERIAL_RX 4 A01
resource SERIAL_RX 6 C07
resource I2C_SCL 1 B08
resource I2C_SCL 2 B10
resource I2C_SDA 1 B09
resource I2C_SDA 2 B11
resource LED 1 A13
resource LED 2 A14
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 C10
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 C11
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 C12
resource ADC_BATT 1 C00
resource ADC_RSSI 1 C02
resource ADC_CURR 1 C01
resource ADC_EXT 1 A04
resource PINIO 1 C08
resource PINIO 2 C09
resource FLASH_CS 1 B12
resource OSD_CS 1 B02
resource GYRO_EXTI 1 C03
resource GYRO_EXTI 2 C04
resource GYRO_CS 1 A15
resource GYRO_CS 2 D02
resource USB_DETECT 1 C14

# timer
timer B04 AF2
# pin B04: TIM3 CH1 (AF2)
timer B05 AF2
# pin B05: TIM3 CH2 (AF2)
timer B00 AF2
# pin B00: TIM3 CH3 (AF2)
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer B06 AF2
# pin B06: TIM4 CH1 (AF2)
timer B07 AF2
# pin B07: TIM4 CH2 (AF2)
timer A08 AF1
# pin A08: TIM1 CH1 (AF1)
timer A03 AF3
# pin A03: TIM9 CH2 (AF3)
timer A02 AF3
# pin A02: TIM9 CH1 (AF3)

# dma
dma ADC 1 1
# ADC 1: DMA2 Stream 4 Channel 0
dma pin B04 0
# pin B04: DMA1 Stream 4 Channel 5
dma pin B05 0
# pin B05: DMA1 Stream 5 Channel 5
dma pin B00 0
# pin B00: DMA1 Stream 7 Channel 5
dma pin B01 0
# pin B01: DMA1 Stream 2 Channel 5
dma pin B06 0
# pin B06: DMA1 Stream 0 Channel 2
dma pin B07 0
# pin B07: DMA1 Stream 3 Channel 2
dma pin A08 2
# pin A08: DMA2 Stream 3 Channel 6

# feature
feature RX_SERIAL
feature OSD

# serial
serial 0 1 115200 57600 0 115200
serial 1 64 115200 57600 0 115200
serial 5 2 115200 57600 0 115200

# aux
aux 0 0 0 1700 2100 0 0
aux 1 41 3 1800 2100 0 0

# master
set mag_bustype = I2C
set mag_i2c_device = 1
set baro_bustype = I2C
set baro_i2c_device = 1
set serialrx_provider = SBUS
set blackbox_device = SPIFLASH
set dshot_burst = ON
set motor_pwm_protocol = DSHOT600
set current_meter = ADC
set battery_meter = ADC
set ibata_scale = 179
set beeper_inversion = ON
set beeper_od = OFF
set gps_provider = UBLOX
set max7456_spi_bus = 1
set pinio_box = 40,41,255,255
set flash_spi_bus = 2
set gyro_1_bustype = SPI
set gyro_1_spibus = 3
set gyro_1_sensor_align = CW180
set gyro_1_align_yaw = 1800
set gyro_2_spibus = 3

*/
