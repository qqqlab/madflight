# Hardware Considerations

## UAV Controller Requirements

- Arduino support
- 32 bit processor
- At least 21 GPIO pins for:
  - 3x UART: Receiver, GPS, spare/debug (6 pins)
  - 1x SPI: fast IMU (4 pins + 1 interrupt pin)
  - 1x I2C: Magnetometer, Barometer, Current sensor, slow IMU (2 pins)
  - 8x PWM: Motor, Servo (8 pins)
  
## Considered UAV Controller Boards

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
RAM | 320K data<br>132K instruction<br>64K cache | 320K data<br>128K instruction<br>64K cache | 264K data/instr.<br>16K XIP cache | 128K
Flash | 2-16M QuadSPI | 2-16M OctalSPI | 2M QuadSPI | 512K internal (CE)<br>256K internal (CC)
PSRAM | 0-8M | 0-8M | 0 | 0
Board price single piece | $4 | $5 | $4 | $3

![](img/boards.jpeg)

### ESP32
![](img/ESP32-DEV-KIT-DevKitC-v4-pinout-mischianti.png)
![](img/ESP32-DOIT-DEV-KIT-v1-pinout-mischianti.png)

### ESP32S3
![](img/esp32-S3-DevKitC-1-original-pinout-high.png)

### RP2040
![](img/Raspberry-Pi-Pico-rp2040-pinout-mischianti.png)
![](img/Raspberry-Pi-Pico-W-rp2040-WiFi-pinout-mischianti.png)

### STM32F411
![](img/STM32-STM32F4-STM32F411-STM32F411CEU6-pinout-high-resolution.png)

## 6-axis IMU

| Part    | Breakout Board? | Interface | Notes |
| ------- | --- | --- | --- |
MPU6000   | yes | SPI, I2C | 8k gyro, 1k acc, WHO_AM_I=0x68 6-bit I2C address of the MPU-60X0, Released 2011 (EOL)
MPU6050   | yes | I2C | 8k gyro, 1k acc, WHO_AM_I=0x68 6-bit I2C address of the MPU-60X0, Released 2011 (EOL)
MPU6500   | $2 | SPI, I2C | 8k gyro, 4k acc, Released 2014
BMI160    | $2
BMI180    |
BMI270    | no | SPI | 6k gyro, Used in current commercial FC
LSM6DS3   | $2
LSM6DSO   | $10
MPU3000   | no | | Released 2011 (EOL)
MPU3050   | no | | Released 2011 (EOL)
MPU6886   | no | | Released
ICM20602  | no
ICM20608  | no
ICM20689  | no 
ICM42688P | no | | 32k gyro, Used in current commercial FC

## 9-axis IMU

| Part    | Breakout Board? | Interface | Notes |
| ------- | --- | --- | --- |
MPU9150  | $7 | I2C | 8k gyro, WHO_AM_I=0x68 or 0x69 depending on AD0 pin, MPU6050 + AK8975, (EOL)
MPU9250  | $7 | SPI 20MHz, I2C | 8k gyro, 4k acc, 100Hz mag, WHO_AM_I=0x71, MPU6500 + AK8963, Released 2014 (EOL) Note: many fake relabelled MPU6500 boards on the market for $4 or less.
MPU9255  | $9 | SPI 20MHz, I2C | 8k gyro, WHO_AM_I=0x73, MPU6000 + AK8963
ICM20948 | $9 | SPI 7MHz, I2C | replacement of MPU-9250/9255, Released 2018
LSM9DSO  | no

## Barometer

| Part    | Breakout Board? | Interface | Notes |
| ------- | --- | --- | --- |
DPS310    | $3 | SPI, I2C | Relative precision: 6Pa 0.5m, Resolution 0.06Pa, Used in current commercial FC
BMP180    | $2
BMP280    | $2 | | Used in current commercial FC
BMP388    | $8 | SPI, I2C 
MS5611    | $4 | SPI 20MHz, I2C | Resolution RMS: 0.012mbar, 1.2Pa, 10cm @ 100Hz
SPL06-001 | no

## Sensor Modules

| Module | Price | Sensors |
|-|-|-|
GY-85 | | 9DOF ITG3205 + ADXL345 + HMC5883L
GY-86 | | 10DOF MPU6050 HMC5883L MS5611
GY-87 | $4 | 10DOF MPU6050 HMC5883L BMP180
GY-91 | $8 | 10DOF MPU9250 BMP280
GY-521 | $2 | 6DOF MPU6050
GY-912 | $11 | 10DOF ICM20948 BMP388

Hardware Table Remarks
* The "Breakout Board?" column lists price if board is cheap & easy to procure.
* Prices aliexpress incl shipping.
* Table created Oct 2023
