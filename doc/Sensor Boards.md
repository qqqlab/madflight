# Sensor Boards

An overview of available sensor boards.

* The "Breakout Board?" column lists price if board is cheap & easy to procure.
* Prices in USD from online platforms incl shipping.
* Table created Dec 2023

## 6-axis IMU

| Part    | Breakout Board? | Interface | Notes |
|-|-|-|-|
MPU-6000   | yes | SPI, I2C | 8k gyro, 1k acc, Released 2011, EOL
MPU-6050   | yes | I2C | 8k gyro, 1k acc, Released 2011, EOL
MPU-6500   | $2 | SPI, I2C | 8k gyro, 4k acc, noise: 10 mdps/&radic;Hz 300 µg/&radic;Hz, upgraded MPU-6000, Released 2014, NRND
BMI160     | $2 | SPI, I2C | 3.2k gyro, 1.6k acc, noise: 8 mdps/&radic;Hz 180 µg/&radic;Hz
BMI270     | no | SPI | 6k gyro, noise: 7 mdps/&radic;Hz 160 µg/&radic;Hz, Used in current commercial FC as replacement for MPU-6000/6500
LSM6DS3    | $2 | | noise: 7 mdps/&radic;Hz 180 µg/&radic;Hz
LSM6DSO    | $10 | | noise: 3.8 mdps/&radic;Hz 110 µg/&radic;Hz
LSM6DSL    | | | 6.7k gyro, 6.7k acc, noise: 4 mdps/&radic;Hz 80 µg/&radic;Hz
MPU-3000   | no | | Released 2011 (EOL)
MPU-3050   | no | | Released 2011 (EOL)
MPU-6886   | no
ICM-20602  | no
ICM-20608  | no
ICM-20649  | no | SPI, I2C | noise: 17.5 mdps/&radic;Hz 285 µg/&radic;Hz, NRND
ICM-20689  | no 
ICM-42688-P | no | SPI | 32k gyro, noise: 2.8 mdps/&radic;Hz 70 µg/&radic;Hz, Used in current high performance FC

## 9-axis IMU

| Part    | Breakout Board? | Interface | Notes |
|-|-|-|-|
MPU-9150  | $7 | I2C | 8k gyro, MPU6050 + AK8975, EOL
MPU-9250  | $7 | SPI 20MHz, I2C | 8k gyro, 4k acc, 100Hz mag, MPU6500 + AK8963, Released 2014 (EOL) Note: many fake or relabelled MPU-6500 chips sold as MPU-9250 on the market...
MPU-9255  | $9 | SPI 20MHz, I2C | 8k gyro, MPU-6000 + AK8963, EOL
ICM-20948 | $9 | SPI 7MHz, I2C | noise: 15 mdps/&radic;Hz 230 µg/&radic;Hz, replacement of MPU-9250/9255, ICM-20649 + AK09916, Released 2018
LSM9DSO  | no

## Magnetometer

| Part    | Breakout Board? | Interface | Notes |
|-|-|-|-|
HMC5883L | $2 | I2C | 160 Hz, 0.2µT/LSB, 12-bit ADC, chip marking "L883"
QMC5883L | $2 | I2C | 200 Hz, 0.008µT/LSB, 16-bit ADC, chip marking "5883"
AK8963 | | I2C | 100 Hz, 0.15µT/LSB, 16-bit, integraded in MPU9250
AK8975 | | I2C | 100 Hz, 0.3µT/LSB, 13-bit, integraded in MPU9150
AK09916 | | I2C | 100 Hz, 0.15µT/LSB, 16-bit, integraded in ICM-20948
IST8310 | | I2C | 200 Hz, 0.3µT/LSB, 14-bit
LIS2MDL | | I2C | 100 Hz, 0.015µT/LSB, 16-bit

## Barometer

| Part    | Breakout Board? | Interface | Relative Precision | Resolution | Measurement Rate | RMS Noise | Notes |
|-|-|-|-|-|-|-|-|
DPS310    | $3 | SPI, I2C | 6Pa 50cm| 0.06Pa||| Used in current commercial FC
BMP180    | $2
BMP280    | $2 | | 12Pa 100cm| 0.18Pa| 157Hz | 1.3Pa| Used in current commercial FC
BMP388    | $8 | SPI, I2C 
MS5611    | $4 | SPI 20MHz, I2C ||1.2Pa 10cm|120Hz
SPL06-001 | no
LPS22HB   | $6 | SPI, I2C |10Pa 80cm|0.025Pa|75Hz|0.75Pa
LPS22DF   | $14 | SPI, I2C |1Pa 8cm|0.025Pa|200Hz|0.34Pa
ILPS22QS  | $14 | SPI, I2C |1.5Pa 12cm|0.025Pa|200Hz|0.34Pa

## Multi Sensor Modules

| Module | Price | DOF | Sensors |
|-|-|-|-|
GY-85 | | 9DOF | ITG3205 + ADXL345 + HMC5883L
GY-86 | | 10DOF | MPU6050 HMC5883L MS5611
GY-87 | $4 | 10DOF | MPU6050 HMC5883L BMP180
GY-91 | $8 | 10DOF | MPU9250 BMP280
GY-521 | $2 | 6DOF | MPU6050
GY-912 | $11 | 10DOF | ICM20948 BMP388
