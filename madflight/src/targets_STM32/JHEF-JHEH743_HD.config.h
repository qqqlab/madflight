//==============================================================================
// Generated on: 2023-12-28 01:17:04.465967
// Generated by: _convert.py in this directory
// Source: https://github.com/betaflight/unified-targets
// Board name: JHEH743_HD
// Manufacturer ID: JHEF
//==============================================================================
#define USE_IMU_SPI_MPU6000
#define USE_IMU_SPI_ICM42688P
#define USE_BARO_DPS310
#define USE_FLASH_W25N01G
#define USE_MAX7456

//LED:
const int HW_PIN_LED      = E05;
#define LED_ON 0 //low = on

//Battery voltage divider:
const int HW_PIN_BAT_ADC  = C01;
const int HW_PIN_BAT_CURR = C03;

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
#define HW_OUT_COUNT 8
const int8_t HW_PIN_OUT[HW_OUT_COUNT] = {PA0,PA1,PA2,PA3,PB0,PB1,PC8,PC9};


/*
#==============================================================================
# BetaFlight Source file
#==============================================================================
# Betaflight / STM32H743 (SH47) 4.3.0 Sep  2 2021 / 06:34:37 (c23dd47f4) MSP API: 1.44

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACCGYRO_BMI270
#define USE_GYRO_SPI_ICM42688P
#define USE_ACC_SPI_ICM42688P       
#define USE_BARO
#define USE_BARO_DPS310
#define USE_FLASH
#define USE_FLASH_W25N01G
#define USE_MAX7456

board_name JHEH743_HD
manufacturer_id JHEF

# resources
resource BEEPER 1 E03
resource MOTOR 1 A00
resource MOTOR 2 A01
resource MOTOR 3 A02
resource MOTOR 4 A03
resource MOTOR 5 B00
resource MOTOR 6 B01
resource MOTOR 7 C08
resource MOTOR 8 C09
resource LED_STRIP 1 A08
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 D05
resource SERIAL_TX 3 D08
resource SERIAL_TX 4 D01
resource SERIAL_TX 5 C12
resource SERIAL_TX 6 C06
resource SERIAL_TX 7 E08
resource SERIAL_TX 8 E01
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 D06
resource SERIAL_RX 3 D09
resource SERIAL_RX 4 D00
resource SERIAL_RX 5 D02
resource SERIAL_RX 6 C07
resource SERIAL_RX 7 E07
resource SERIAL_RX 8 E00
resource I2C_SCL 1 B06
resource I2C_SCL 2 B10
resource I2C_SDA 1 B07
resource I2C_SDA 2 B11
resource LED 1 E05
resource LED 2 E04
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 C10
resource SPI_SCK 4 E12
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 C11
resource SPI_MISO 4 E13
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 B02
resource SPI_MOSI 4 E14
resource ADC_BATT 1 C01
resource ADC_CURR 1 C03
resource ADC_EXT 1 C00
resource PINIO 1 C02
resource PINIO 2 C05
resource FLASH_CS 1 A15
resource OSD_CS 1 B12
resource GYRO_EXTI 1 C04
resource GYRO_EXTI 2 E15
resource GYRO_CS 1 A04
resource GYRO_CS 2 E11
resource USB_DETECT 1 E06

# timer
timer A00 AF2
# pin A00: TIM5 CH1 (AF2)
timer A01 AF2
# pin A01: TIM5 CH2 (AF2)
timer A02 AF2
# pin A02: TIM5 CH3 (AF2)
timer A03 AF2
# pin A03: TIM5 CH4 (AF2)
timer B00 AF2
# pin B00: TIM3 CH3 (AF2)
timer B01 AF2
# pin B01: TIM3 CH4 (AF2)
timer C08 AF3
# pin C08: TIM8 CH3 (AF3)
timer C09 AF3
# pin C09: TIM8 CH4 (AF3)
timer A08 AF1
# pin A08: TIM1 CH1 (AF1)
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
dma TIMUP 3 2
# TIMUP 3: DMA1 Stream 2 Request 27
dma TIMUP 5 0
# TIMUP 5: DMA1 Stream 0 Request 59
dma TIMUP 8 4
# TIMUP 8: DMA1 Stream 4 Request 51
dma pin A00 0
# pin A00: DMA1 Stream 0 Request 55
dma pin A01 1
# pin A01: DMA1 Stream 1 Request 56
dma pin A02 2
# pin A02: DMA1 Stream 2 Request 57
dma pin A03 3
# pin A03: DMA1 Stream 3 Request 58
dma pin B00 4
# pin B00: DMA1 Stream 4 Request 25
dma pin B01 5
# pin B01: DMA1 Stream 5 Request 26
dma pin C08 6
# pin C08: DMA1 Stream 6 Request 49
dma pin C09 7
# pin C09: DMA1 Stream 7 Request 50
dma pin A08 14
# pin A08: DMA2 Stream 6 Request 11
dma pin B08 0
# pin B08: DMA1 Stream 0 Request 109
dma pin B09 0
# pin B09: DMA1 Stream 0 Request 111

# feature
feature -AIRMODE
feature MOTOR_STOP
feature LED_STRIP
feature OSD

# master
set gyro_to_use = BOTH
set mag_bustype = I2C
set mag_i2c_device = 1
set baro_bustype = I2C
set baro_i2c_device = 1
set blackbox_device = SPIFLASH
set serialrx_provider = CRSF
set current_meter = ADC
set battery_meter = ADC
set motor_pwm_protocol = DSHOT600
set ibata_scale =170
set vbat_scale = 110
set osd_vbat_pos = 2347
set osd_crosshairs_pos = 2253
set osd_ah_sbar_pos = 2254
set osd_ah_pos = 2126
set osd_current_pos = 2442
set osd_warnings_pos = 14697
set beeper_frequency = 0
set beeper_inversion = ON
set beeper_od = OFF
set max7456_spi_bus = 2
set flash_spi_bus = 3
set pinio_config = 129,129,1,1
set pinio_box = 0,40,255,255
set gyro_1_bustype = SPI
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW90
set gyro_1_align_yaw = 900
set gyro_2_bustype = SPI
set gyro_2_spibus = 4
set gyro_2_sensor_align = CW180
set gyro_2_align_yaw = 1800

*/
