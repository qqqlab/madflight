# Sensor Boards

An overview of available sensor boards.

* The "Breakout Board?" column lists price if board is cheap & easy to procure.
* Prices in USD from online platforms incl shipping.
* Table created Dec 2023

## 6-axis IMU

| Part    | Breakout Board? | Interface | Notes |
|-|-|-|-|
MPU-6000   | yes | SPI, I2C | 8k gyro, 1k acc, Released 2011 (EOL)
MPU-6050   | yes | I2C | 8k gyro, 1k acc, Released 2011 (EOL)
MPU-6500   | $2 | SPI, I2C | 8k gyro, 4k acc, upgraded MPU-6000, Released 2014 (not recommended for new designs)
BMI160    | $2
BMI180    |
BMI270    | no | SPI | 6k gyro, Used in current commercial FC as replacement for MPU-6000/6500
LSM6DS3   | $2
LSM6DSO   | $10
MPU-3000   | no | | Released 2011 (EOL)
MPU-3050   | no | | Released 2011 (EOL)
MPU-6886   | no
ICM-20602  | no
ICM-20608  | no
ICM-20649  |
ICM-20689  | no 
ICM-42688-P | no | SPI | 32k gyro, low noise IMU, Used in current high performance FC.

## 9-axis IMU

| Part    | Breakout Board? | Interface | Notes |
|-|-|-|-|
MPU-9150  | $7 | I2C | 8k gyro, MPU6050 + AK8975, (EOL)
MPU-9250  | $7 | SPI 20MHz, I2C | 8k gyro, 4k acc, 100Hz mag, MPU6500 + AK8963, Released 2014 (EOL) Note: many fake or relabelled MPU-6500 chips sold as MPU-9250 on the market...
MPU-9255  | $9 | SPI 20MHz, I2C | 8k gyro, MPU-6000 + AK8963 (EOL)
ICM-20948 | $9 | SPI 7MHz, I2C | replacement of MPU-9250/9255, ICM-20649 + AK09916, Released 2018
LSM9DSO  | no

## Magnetometer

| Part    | Breakout Board? | Interface | Notes |
|-|-|-|-|
HMC5883L | $2 | I2C | 160 Hz, 0.2µT/LSB, 12-bit ADC, chip marking "L883"
QMC5883L | $2 | I2C | 200 Hz, 0.008µT/LSB, 16-bit ADC, chip marking "5883"
AK8963 | | I2C | 100 Hz, 0.15µT/LSB, 16-bit, internal to MPU9250
AK8975 | | I2C | 100 Hz, 0.3µT/LSB, 13-bit, internal to MPU9150
AK09916 | | I2C | 100 Hz, 0.15µT/LSB, 16-bit, internal to ICM-20948
IST8310 | | I2C | 200 Hz, 0.3µT/LSB, 14-bit

## Barometer

| Part    | Breakout Board? | Interface | Notes |
|-|-|-|-|
DPS310    | $3 | SPI, I2C | Relative precision: 6Pa 0.5m, Resolution 0.06Pa, Used in current commercial FC
BMP180    | $2
BMP280    | $2 | | Used in current commercial FC
BMP388    | $8 | SPI, I2C 
MS5611    | $4 | SPI 20MHz, I2C | Resolution RMS: 0.012mbar, 1.2Pa, 10cm @ 100Hz
SPL06-001 | no

## Multi Sensor Modules

| Module | Price | DOF | Sensors |
|-|-|-|-|
GY-85 | | 9DOF | ITG3205 + ADXL345 + HMC5883L
GY-86 | | 10DOF | MPU6050 HMC5883L MS5611
GY-87 | $4 | 10DOF | MPU6050 HMC5883L BMP180
GY-91 | $8 | 10DOF | MPU9250 BMP280
GY-521 | $2 | 6DOF | MPU6050
GY-912 | $11 | 10DOF | ICM20948 BMP388
