# madflight

***M**anless **A**erial **D**evice*

This is a 1500 line Arduino ESP32 / RP2040 / STM32 flight controller, forked from [dRehmFlight](https://github.com/nickrehm/dRehmFlight). A working flight controller can be build for under $10 from readily available development boards and sensor breakout boards. Ideal if you want to try out new flight control concepts, without first having to setup a build environment and without having to read through thousands lines of code to find the spot where you want to change something.

`madflight/madflight.ino` is a demo program for a quadcopter, but can be easily adapted to control your plane or VTOL craft. The source code has extensive documentation explaning what the settings and functions do.

The source code is tested on ESP32, RP2040, and STM32F405 microcontrollers with the Arduino IDE. It mainly uses plain Arduino functionality: Serial, Wire, and SPI. A custom controller dependent library is used for PWM. Therefor, it can fairly easily ported to other 32 bit microcontrollers that support the Arduino framework. Also porting to other build environments like PlatformIO or CMake should not be a huge effort.

<img src="doc/img/madflight RP2040 flight controller.jpeg" title="madflight RP2040 flight controller" width="25%" /> <img src="doc/img/madflight drone.jpeg" title="madflight drone" width="19.6%" /> <img src="doc/img/madflight ESP32 flight controller.jpeg" title="madflight ESP32 flight controller" width="19.1%" />

# Required Hardware

- Development board: 
  - RP2040 (e.g. Raspberry Pi Pico)
  - or ESP32 (e.g. Espressiv DevKitC)
  - or STM32 (e.g. Black Pill or a flight controller)
- SPI or I2C IMU sensor: MPU6000, MPU6050, MP6500, MPU9150, or MPU9250
- RC Receiver: ELRS, CRSF, SBUS, DMX, or PPM
- BEC or DC-DC converter to power your board from a battery
- ESC (OneShot125 or 50-490Hz PWM) and/or servos (50-490Hz PWM)

# Getting Started

0. Open madflight/madflight.ino in the Arduino IDE.
1. Setup the USER-SPECIFIED DEFINES section in the main code, and configure the pins in hw_XXX.h (see below for default pinouts)
2. Connect your IMU sensor including the INT pin according to the configured pins:
    - Connect sensor VCC and GND pins to dev board 3.3V and GND.
    - For I2C sensors: connect sensor SDA to dev board I2C_SDA, SCL to I2C_SCL and INT to IMU_INT.
    - For SPI sensors: connect sensor SCL/SCLK to dev board SPI_SCLK, SDA/SDI to SPI_MOSI, ADD/SDO to SPI_MISO, NCS to SPI_CS, and INT to IMU_INT.
5. Uncomment print_imu_GyroData(), print_imu_AccData(), print_imu_MagData(), and/or print_ahrs_RollPitchYaw() and check that IMU sensor and AHRS are working correctly. 
6. Uncomment lines in setup() to calibate the sensor.
7. Connect radio receiver to your development board according to the configured pins.
8. Edit the RC RECEIVER CONFIG section in the main code. Either match you RC equipment to the settings, or change the settings to match your RC equipment. 
9. Uncomment print_rcin_RadioPWM() and print_rcin_RadioScaled() to check your radio setup.
11. Connect motors (no props) and battery and check that motor outputs are working correctly. For debugging, use print_out_MotorCommands() and calibrate_ESCs()
12. Mount props, go to an wide open space, and FLY!

# Change Log

2023-12-28 Add converted Betaflight targets  
2023-12-27 Add STM32 support  
2023-12-24 Add external magnetometer  
2023-12-18 Add CRSF telemetry  
2023-12-18 Add GPS  
2023-12-15 Add BMP280 and MS5611 barometers  
2023-12-14 Add CRSF/ELRS radio receiver  
2023-12-13 Add Mahony AHRS  
2023-12-11 Add USE_IMU_INTERRUPT for interrupt driven operation  
2023-12-11 Use C++ template for flexible I2C implementations  
2023-12-06 Add setup1() and loop1() for ESP32  
2023-12-06 Add IMU orientation setting  
2023-12-05 Initial release  

# Software Design

- Keep it simple!!!
- No external dependencies, all module libraries included in `src` directory
- The flight controller madflight.ino runs standard `setup()` and `loop()`.
- madflight.ino is written in plain C with minimal function arguments.
- Global variables to communicate between the different modules.
- Function names are prefixed with the module they belong to:
  - `loop_` Main loop control
  - `imu_` Inertial Measurement Unit, retrieves accelerometer, gyroscope, and magnetometer sensor data
  - `ahrs_` Attitude Heading Reference System, estimates roll, yaw, pitch
  - `rcin_` RC INput, retrieves RC receiver data
  - `control_` PID controller and output mixer
  - `out_` Output to motors and servos
  - `mag_` Magnetometer (compass)
  - `gps_` GPS receiver
  - `print_` Prints debugging info
  - `calibrate_` Calibration
- Module source code is in subdirectories of the `src` directory. Here you find a .h file with the same name (e.g. `src/imu/imu.h`) which is the interface between the main program and the module. There might also be an .ino example program, e.g. `src/imu/imu.ino`.
- The module files are usually header only, i.e. the header also includes the implemention.

## Default Pinout for ESP32 DevKitC (38 pin)

| Function | GPIO | Board | GPIO | Function |
| --: | :-- | -- |--: | :-- |
| 3V3 out      | 3V3 | Antenna side            |  GND | GND
| reset button | EN |                            | 23 | I2C_SDA
| SPI_MISO     | VP 36 input only |              | 22 | I2C_SCL
| IMU_INT      | VN 39 input only |            | 1 TX | USB Serial Debug TX
| BAT_ADC      | 34 input only |               | 3 RX | USB Serial Debug RX
| RCIN_RX      | 35 input only |                 | 21 | SPI_MOSI
| RCIN_TX      | 32 |                           | GND | GND
| PWM1         | 33 |                            | 19 | SPI_SCLK
| PWM2         | 25 |                            | 18 | SPI_CS
| PWM3         | 26 |                       | strap 5 | GPS_TX
| PWM4         | 27 |                            | 17 | GPS_RX
| PWM5         | 14 |                            | 16 | PWM11
| PWM6         | 12 |                             | 4 | PWM10
| GND          | GND |                       | boot 0 | PWM9
| PWM7         | 13 |                       | strap 2 | LED     
| nc           | D2 9 flash |              | strap 15 | PWM8
| nc           | D3 10 flash |           | flash 8 D1 | nc
| nc           | CMD 11 flash |          | flash 7 D0 | nc
| 5V in (*)    | 5V | USB connector     | flash 6 CLK | nc

Note: During boot the input voltage levels (pull up/pull down) on strap pins have a configuration function, therefor these pins are used as output only.

(*) 5V input via diode from BEC. Without a diode take care not connect USB and the battery at the same time!

<img src="doc/img/ESP32-DEV-KIT-DevKitC-v4-pinout-mischianti.png" width="60%" />

## Default Pinout for Raspberry Pi Pico

| Function | GPIO | Board | GPIO | Function |
| --: | :-- | -- |--: | :-- |
|      RCIN_TX | 0   | USB connector | VBUS     | nc
|      RCIN_RX | 1   |               | VSYS     | 5V input via diode (*)
|            - | GND |               | GND      | -
|         PWM1 | 2   |               | EN       | nc
|         PWM2 | 3   |               | 3.3V out | 3V3
|         PWM3 | 4   |               | VREF     | nc
|         PWM4 | 5   |               | 28_A2    | BAT_ADC
|            - | GND |               | GND      | -
|         PWM5 | 6   |               | 27_A1    | FREE
|         PWM6 | 7   |               | 26_A0    | FREE
|       GPS_TX | 8   |               | RUN      | reset button to GND
|       GPS_RX | 9   |               | 22       | IMU_INT
|            - | GND |               | GND      | -
|         PWM7 | 10  |               | 21       | I2C_SCL
|         PWM8 | 11  |               | 20       | I2C_SDA
|         PWM9 | 12  |               | 19       | SPI_MOSI
|        PWM10 | 13  |               | 18       | SPI_SCLK
|            - | GND |               | GND      | -
|        PWM11 | 14  |               | 17       | SPI_CS
|        PWM12 | 15  | JTAG pins     | 16       | SPI_MISO

(*) 5V input via diode from BEC. Without a diode take care not connect USB and the battery at the same time!

<img src="doc/img/Raspberry-Pi-Pico-rp2040-pinout-mischianti.png" width="45%" /> <img src="doc/img/Raspberry-Pi-Pico-W-rp2040-WiFi-pinout-mischianti.png" width="46.8%" />

## Default Pinout for STM32F411 Black Pill (40 pin)

| Function | GPIO | Board | GPIO | Function |
| --: | :-- | -- |--: | :-- |
|            - | VB  |   SWD pins    | 3V3 | -
|         FREE | C13 |               | G   | -
|         FREE | C14 |               | 5V  | 5V input (*)
|         FREE | C15 |               | B9  | FREE
|            - | R   |               | B8  | PWM6
|         FREE | A0  |               | B7  | I2C_SCL
|         FREE | A1  |               | B6  | I2C_SDA
|       GPS_TX | A2  |               | B5  | PWM2
|       GPS_RX | A3  |               | B4  | FREE
|       SPI_CS | A4  |               | B3  | RCIN_RX
|     SPI_SCLK | A5  |               | A15 | RCIN_TX
|     SPI_MISO | A6  |               | A12 | FREE
|     SPI_MOSI | A7  |               | A11 | FREE
|     BAT_CURR | B0  |               | A10 | PWM5
|      BAT_ADC | B1  |               | A9  | PWM4
|         PWM1 | B2  |               | A8  | PWM3
|      IMU_INT | B10 |               | B15 | FREE
|            - | 3V3 |               | B14 | FREE
|            - | G   |               | B13 | FREE
|            - | 5V  | USB connector | B12 | FREE

Internal LED: C13

This is basically the same pinout as a MATEKF411SE flight controller, except: LED and IMU_INT

(*) 5V input via diode from BEC. Without a diode take care not connect USB and the battery at the same time!

<img src="doc/img/STM32-STM32F4-STM32F411-STM32F411CEU6-pinout-high-resolution.png" width="45%" />

# Changes from dRehmFlight

- Add support for RP2040, ESP32, and STM32
- Dropped Teensy support, but could be re-added by creating a hw_TEENSY.h file. (I just don't have the hardware to test on)
- Moved all hardware specific code to hw_XXX.h and added hardware specific libraries
- Reduced the number of global variables
- Oneshot is implemented as PWM up to 3.9kHz
- New libs for IMU sensors
- Changed arming logic
- Loop rate set to 1kHz to match IMU sensor rate
- Interrupt driven IMU operation by default, but setup/loop still possible

# Flight Controllers on Github

In increasing order of complexity.

- [lobodol/drone-flight-controller](https://github.com/lobodol/drone-flight-controller) Arduino ATmega328P, single 700 line ino file, no libs
- [dRehmFlight](https://github.com/nickrehm/dRehmFlight) Arduino Teensy 4
- [madflight](https://github.com/qqqlab/madflight) Arduino RP2040, ESP32, and STM32
- [Crazyflie](https://github.com/bitcraze/crazyflie-firmware) STM32F405
- [esp-drone](https://github.com/espressif/esp-drone.git) ESP32 (forked from Crazyflie)
- [Betaflight](https://github.com/betaflight/betaflight) STM32 F4/F7/H7
- [inav](https://github.com/iNavFlight/inav) STM32 F4/F7/H7
- [Ardupilot](https://github.com/ArduPilot/ardupilot) STM32 F4/F7/H7 or Linux based
