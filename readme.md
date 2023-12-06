# madflight

This is a 1500 line Arduino ESP32 & RP2040 flight controller, forked from [dRehmFlight](https://github.com/nickrehm/dRehmFlight).

# Required Hardware

- Development board: RP2040 (e.g. Raspberry Pi Pico) or ESP32 (e.g. Espressiv DevKitC) 
- SPI or I2C IMU sensor: MPU6050, MP6500, MPU9150, or MPU9250
- RC Receiver: PPM, SBUS, or DMX
- ESC: OneShot125 or 50-490Hz PWM
- BEC or DC-DC converter to power your board from a battery
- Optional servos: 50-490Hz PWM

# Getting Started

1. Setup the USER-SPECIFIED DEFINES section in the main code, and configure the pins in hw_ESP32.h or hw_RP2040.h
2. Connect the IMU sensor and radio receiver to your development board according to the selected pinout.
3. Edit the RC RECEIVER CONFIG section in the main code. Either match you RC equipment to the settings, or change the settings to match your RC equipment. Uncomment print_rcin_RadioPWM() to check.
4. Uncomment print_ahrs_RollPitchYaw() and check that IMU sensor and AHRS are working correctly. Uncomment lines in setup() to calibate the sensor.
5. Connect motors (no props) and battery and check that motor outputs are working correctly. For debugging, use print_out_MotorCommands() and calibrate_ESCs()
6. Mount props, go to an wide open space, and FLY!

# Software Design

- Keep it simple!!!
- No external dependencies, all library code included `src` directory
- The main .ino is the full flight controller running standard `setup()` and `loop()`.
- Plain C with minimal function arguments.
- Global variables to communicate between the different functions.
- Function names are prefixed with the module it belongs to:
  - `loop_` Main loop control
  - `imu_` Inertial Measurement Unit, retrieves accelerometer, gyroscope, and magnetometer sensor data
  - `ahrs_` Attitude Heading Reference System, estimates roll, yaw, pitch
  - `rcin_` RC INput, retrieves RC receiver data
  - `control_` PID controller and output mixer
  - `out_` Output to motors and servos
  - `print_` Prints debugging info
  - `calibrate_` Calibration

## Default Pinout for ESP32 DevKitC (38 pin)

| Function | GPIO | Board | GPIO | Function |
| --: | :-- | -- |--: | :-- |
| 3V3 out      | 3V3 | Antenna side            |  GND | GND
| reset button | EN |                            | 23 | I2C_SDA
| VSPI_MISO    | VP 36 input only |              | 22 | I2C_SCL
| IMU_INT      | VN 39 input only |            | 1 TX | USB Serial Debug TX
| FREE         | 34 input only |               | 3 RX | USB Serial Debug RX
| RCIN_RX      | 35 input only |                 | 21 | VSPI_MOSI
| RCIN_TX      | 32 |                           | GND | GND
| PWM1         | 33 |                            | 19 | VSPI_SCLK
| PWM2         | 25 |                            | 18 | VSPI_CS
| PWM3         | 26 |                       | strap 5 | PMW13
| PWM4         | 27 |                            | 17 | PMW12
| PWM5         | 14 |                            | 16 | PWM11
| PWM6         | 12 |                             | 4 | PWM10
| GND          | GND |                       | boot 0 | PWM9
| PWM7         | 13 |                       | strap 2 | LED     
| nc           | D2 9 flash |              | strap 15 | PWM8
| nc           | D3 10 flash |           | flash 8 D1 | nc
| nc           | CMD 11 flash |          | flash 7 D0 | nc
| 5V in (*)    | 5V | USB connector     | flash 6 CLK | nc

(*) 5V input via diode from BEC. Without a diode take care not connect USB and the battery at the same time!

![](doc/img/ESP32-DEV-KIT-DevKitC-v4-pinout-mischianti.png)

## Default Pinout for Raspberry Pi Pico

| Function | GPIO | Board | GPIO | Function |
| --: | :-- | -- |--: | :-- |
| RCIN_TX(tx0) | 0   | USB connector | VBUS     | nc
| RCIN_RX(rx0) | 1   |               | VSYS     | 5V input via diode (*)
|            - | GND |               | GND      | -
|         PWM1 | 2   |               | EN       | nc
|         PWM2 | 3   |               | 3.3V out | 3V3
|         PWM3 | 4   |               | VREF     | nc
|         PWM4 | 5   |               | 28_A2    | FREE
|            - | GND |               | GND      | -
|         PWM5 | 6   |               | 27_A1    | FREE
|         PWM6 | 7   |               | 26_A0    | FREE
|         PWM7 | 8   |               | RUN      | reset button to GND
|         PWM8 | 9   |               | 22       | IMU_INT
|            - | GND |               | GND      | -
|         PWM9 | 10  |               | 21       | I2C0_SCL
|         PWM10| 11  |               | 20       | I2C0_SDA
|        PWM11 | 12  |               | 19       | SPI0_MOSI
|        PWM12 | 13  |               | 18       | SPI0_SCLK
|            - | GND |               | GND      | -
|        PWM13 | 14  |               | 17       | SPI0_CS
|        PWM14 | 15  | JTAG pins     | 16       | SPI0_MISO

(*) 5V input via diode from BEC. Without a diode take care not connect USB and the battery at the same time!

![](doc/img/Raspberry-Pi-Pico-rp2040-pinout-mischianti.png)
![](doc/img/Raspberry-Pi-Pico-W-rp2040-WiFi-pinout-mischianti.png)

# Changes from dRehmFlight

- Add support for RP2040 and ESP32
- Dropped Teensy support, but could be re-added by creating a hw_TEENSY.h file. (I just don't have the hardware to test on)
- Moved all(*) hardware specific code to hw_ESP32.h and hw_RP2040.h and added hardware specific libraries
- Reduced the number of global variables
- Oneshot is implemented as PWM up to 3.9kHz
- New libs for IMU sensors
- Changed arming logic

(*) Because of bugs in the ESP32 Wire.h a custom lib sensor/WireAlternative.h is used.

# Flight Controllers on Github

In increasing order of complexity.

- [lobodol/drone-flight-controller](https://github.com/lobodol/drone-flight-controller) Arduino ATmega328P, single 700 line ino file, no libs
- [dRehmFlight](https://github.com/nickrehm/dRehmFlight) Arduino Teensy 4
- [madflight](https://github.com/qqqlab/madflight) Arduino RP2040 & ESP32
- [Crazyflie](https://github.com/bitcraze/crazyflie-firmware) STM32F405
- [esp-drone](https://github.com/espressif/esp-drone.git) ESP32, forked from Crazyflie
- [Betaflight](https://github.com/betaflight/betaflight) STM32
- [Ardupilot](https://github.com/ArduPilot/ardupilot) STM32 or Linux based
