- 6 pins 3x UART: Receiver, GPS, spare/debug
- 2 pins 1x I2C: Mag, Baro, Current sensor, IMU
- 4 pins 1x SPI: IMU
- 8 pins 8x PWM: Motor, Servo

TOTAL: 20 pins



| Board | ESP32 DevKitC | ESP32-S3 DevKitC | Raspberry Pi Pico (W) | Black Pill |
| --- | :-: | :-: | :-: | :-: |
Board Size | 55 * 28 mm | 69 * 26 mm | 51 * 21 mm | 53 * 21 mm
Board Weight | 6.9 g<br>(9.1 g with headers) | 8.4 g<br>(10.9 g with headers) | 3.0 g | 4.5 g
Board Pins | 38 pins | 44 pins | 40 pins | 40 pins
Available external GPIO pins | 21<br>+ 4 input only<br>+ 1 button: 0<br>Note: strap pin restictions | 25<br>+ 7 external pins but used:<br>Button: 0<br>OSPI: 35, 36, 37<br>RGB LED: 38<br>USB: 19, 20<br>Note: strap pin restictions | 26<br>Internal only:<br>Power save: 23<br>VBUS monitor: 24<br>LED: 25<br>VSYS voltage: 29 ADC3 | 30<br>+ 2 external but used:<br>Button: PA0<br>LED: PC13
PWM | 16<br>(8 timers each with 2 output pins) | 16<br>(8 timers each with 2 output pins) | 16<br>(8 timers each with 2 output pins) | 25<br>(6 * 16bit + 2 * 32bit timers)
Available UART | 3 | 3 | 6 (2 + 4*PIO)<br>+USB Serial debug | 3
Available SPI | 2 | 2 | 2 | 5
Available I2C | 2 | 2 | 2 | 2
Available ADC pins | 16 (12bit) | 20 (12bit) | 3 (12bit) | 10 (12bit)
On Board Peripherals | WIFI + BT + Button | WIFI + BT + Button + RGB LED | LED<br>+ WIFI/BT (W) | Button + LED (+ optional SPI flash)
MCU | ESP32 | ESP32S3 | RP2040 | STM32F411CE/CC
MCU GPIO | 34 | 45 | 30 | 32
Processor | 2 * 240MHz LX6 | 2 * 240MHz LX7 | 2 * 133MHz M0+ | 1 * 100MHz M4
Coremark (single core) | 351 | | 228 | 172
FPU | FPU | FPU | no FPU | FPU
RAM | 520K | 512K | 264K | 128K
Flash | 2-16M QuadSPI | 2-16M OctalSPI | 2M QuadSPI | 512K internal (CE)<br>256K internal (CC)
PSRAM | 0-8M | 0-8M | 0 | 0
Board price single piece | $4 | $5 | $4 | $3

![](doc/img/boards.jpeg)

# ESP32
## ESP32 DevKitC (38 pin) Board Connections 
| Function | GPIO | Board | GPIO | Function |
| --: | :-- | -- |--: | :-- |
| 3V3 out | 3V3 | Antenna side | GND | GND
| nc | EN | | 23 | I2C_SDA
| rcin_PPM | 36 input only (VP) | | 22 | I2C_SCL
| FREE | 39 input only (VN) | | TX | USB Serial Debug TX
| FREE | 34 input only | | RX | USB Serial Debug RX
| FREE | 35 input only | | 21 | RCIN_TX
| PWM1 | 32 | | GND | GND
| PWM2 | 33 | | 19 | RCIN_RX
| PWM3 | 25 | | 18 | PMW12
| PWM4 | 26 | | 5 strap | PMW11
| PWM5 | 27 | | 17 | PMW10
| PWM6 | 14 | | 16 | PMW9
| PWM7 | 12 | | 4 | SPI_MISO
| GND | GND | | 0 boot | SPI_MISO 
| PWM8 | 13 | | 2 strap | SPI_MISO
| nc | 9 flash (D2) | | 15 strap | SPI_CLK
| nc | 10 flash (D3) | | 8 flash (D1) | nc
| nc | 11 flash (CMD) | | 7 flash (D0) | nc
| 5V in via diode | 5V | USB connector | 6 flash (CLK) | nc

![](doc/img/ESP32-DEV-KIT-DevKitC-v4-pinout-mischianti.png)
![](doc/img/ESP32-DOIT-DEV-KIT-v1-pinout-mischianti.png)

# ESP32S3
![](doc/img/esp32-S3-DevKitC-1-original-pinout-high.png)

# RP2040

## Raspberry Pi Pico Board Connections
| Function | GPIO | Board | GPIO | Function |
| --: | :-- | -- |--: | :-- |
| RADIO_RX(tx0) | 0 | USB connector | VBUS | nc
| RADIO_TX(rx0) | 1 | | VSYS | 5V in via diode
| - | GND | | GND | -
| PWM1 | 2 | | EN | nc
| PWM2 | 3 | | 3.3V out | 3V3
| PWM3 | 4 | | VREF | nc
| PWM4 | 5 | | 28_A2 | FREE
| - | GND | | GND | -
| PWM5 | 6 | | 27_A1 | FREE
| PWM6 | 7 | | 26_A0 | FREE
| PWM7 | 8 | | RUN | -
| PWM8 | 9 | | 22 | RADIO_PPM
| - | GND | | GND | -
| PWM9 | 10 | | 21 | I2C0_SCL
| PWM10| 11 | | 20 | I2C0_SDA
| PWM11 | 12 | | 19 | SPI0_MOSI
| PWM12 | 13 | | 18 | SPI0_SCLK
| - | GND | | GND | -
| PWM13 | 14 | | 17 | SPI0_CS
| PWM14 | 15 | JTAG pins| 16 | SPI0_MISO
    

![](doc/img/Raspberry-Pi-Pico-rp2040-pinout-mischianti.png)
![](doc/img/Raspberry-Pi-Pico-W-rp2040-WiFi-pinout-mischianti.png)

# STM32F411
![](doc/img/STM32-STM32F4-STM32F411-STM32F411CEU6-pinout-high-resolution.png)


6-axis IMU
==========
| Part    | Breakout Board Availaible? | Interface | Notes |
| ------- | --- | --- | --- |
MPU6000   | yes | SPI, I2C | 8k gyro, WHO_AM_I=0x68 6-bit I2C address of the MPU-60X0, Released 2011 (EOL)
MPU6050   | yes | I2C | 8k gyro, WHO_AM_I=0x68 6-bit I2C address of the MPU-60X0, Released 2011 (EOL)
MPU6500   | yes $2 | SPI, I2C | 32k gyro, Released 2014
BMI160    | yes $2
BMI270    | | SPI | 6k gyro, Used in current commercial FC
LSM6DS3   | yes $2
LSM6DSO   | yes $10
MPU3000   | no | | Released 2011 (EOL)
MPU3050   | no | | Released 2011 (EOL)
MPU6886   | no | | Released
ICM20602  | no
ICM20608  | no
ICM20689  | no 
ICM42688P | no | | 32k gyro, Used in current commercial FC


9-axis IMU
==========
| Part    | Breakout Board Availaible? | Interface | Notes |
| ------- | --- | --- | --- |
MPU-9150  | yes $7 | I2C | 8k gyro, WHO_AM_I=0x68 or 0x69 depending on AD0 pin, MPU6050 + AK8975, (EOL)
MPU-9250  | yes $4 | SPI 20MHz, I2C | 8k gyro, WHO_AM_I=0x71, MPU6000 + AK8963, Released 2014 (EOL)
MPU-9255  | yes $9 | SPI 20MHz, I2C | 8k gyro, WHO_AM_I=0x73, MPU6000 + AK8963
ICM-20948 | yes $9 | SPI 7MHz, I2C | replacement of MPU-9250/9255, Released 2018
LSM9DSO   |


Barometer
=========
| Part    | Breakout Board Availaible? | Interface | Notes |
| ------- | --- | --- | --- |
DPS310    | yes $3 | SPI, I2C | Relative precision: 6Pa 0.5m, Resolution 0.06Pa, Used in current commercial FC
BMP280    | yes $2 | | Used in current commercial FC
BMP388    | | SPI, I2C | Relative precision: 
MS5611    | yes $4
SPL06-001 |

Table Remarks
* Breakout Board Availaible? is "yes" if an cheap & easy to find board is available.
* Prices aliexpress incl shipping.
* Table created Oct 2023




