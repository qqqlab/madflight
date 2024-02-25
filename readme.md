<img src="doc/img/logo/madflight_logo_2000x538.png" width="100%" />

***M**anless **A**erial **D**evice*

This is an Arduino library to build ESP32 / ESP32-S3 / RP2040 / STM32 flight controllers. A functional DIY flight controller can be build for under $10 from readily available development boards and sensor breakout boards. Ideal if you want to try out new flight control concepts, without first having to setup a build environment and without having to read through thousands lines of code to find the spot where you want to change something.

`Quadcopter.ino` is a 1000 line demo program for a quadcopter. It has been flight tested on ESP32, ESP32-S3, RP2040, and STM32F405 microcontrollers with the Arduino IDE. The program can be easily adapted to control your plane or VTOL craft. The source code has extensive documentation explaning what the settings and functions do.

<img src="doc/img/madflight RP2040 flight controller.jpeg" title="madflight RP2040 flight controller" width="25%" /> <img src="doc/img/madflight drone.jpeg" title="madflight drone" width="19.6%" /> <img src="doc/img/madflight ESP32 flight controller.jpeg" title="madflight ESP32 flight controller" width="19.1%" />

## Feedback is Welcome

I enjoy hacking with electronics and I'm attempting to write some decent code for this project. If you enjoy it as well, please leave some feedback in the form of Stars, Issues, Pull Requests, or Discussions. Thanks!

[Required Hardware](#hardware)  
[Getting Started](#gettingstarted)  
[Safety First](#safetyfirst)  
[Software Design](#softwaredesign)  
[Connecting the IMU Sensor](#connectsensor)  
[ESP32 Pinout](#pinoutESP32)  
[ESP32-S3 Pinout](#pinoutESP32-S3)  
[RP2040 Pinout](#pinoutRP2040)  
[STM32 Pinout](#pinoutSTM32)  
[Of-the-shelf Flight Controller Pinout](#pinoutFC)  
[Changes from dRehmFlight](#changes)  
[Flight Controllers on Github](#github)  
[Disclaimer](#disclamer)  


<a name="hardware"></a>
## Required Hardware

- Development board: 
  - RP2040 (e.g. Raspberry Pi Pico)
  - or ESP32 (e.g. Espressiv DevKitC)
  - or STM32 (e.g. Black Pill or a commercial flight controller)
- SPI IMU sensor (BMI270, MPU9250, MP6500, or MPU6000), if not available then use an I2C IMU sensor (MPU6050 or MPU9150) 
- RC Receiver: ELRS, CRSF, SBUS, DMSX, or PPM
- BEC or DC-DC converter to power your board from a battery
- ESC (OneShot125 or 50-490Hz PWM) and/or servos (50-490Hz PWM)

## Optional Hardware

- GPS Module (Serial)
- Barometer (I2C BMP280, MS5611)
- Magnetometer (I2C QMC5883L)
- Current/Voltage Sensor (ADC or I2C INA226)
- [Optical Flow Sensor](https://github.com/qqqlab/ESP32-Optical-Flow) (I2C)


<a name="gettingstarted"></a>
## Getting Started

1. Install the Arduino madflight library
2. Open example Quadcopter.ino in the Arduino IDE.
3. Setup the USER-SPECIFIED DEFINES section
4. If you're not using a default pinout (see below) then setup your board pinout in the BOARD section.
5. Connect your IMU (gyro/acceleration) sensor as shown below.
6. Compile and upload Quadcopter.ino to your board. Connect the Serial Monitor at 115200 baud and check the messages. Type 'help' to see the available CLI commands.
7. Check that IMU sensor and AHRS are working correctly: use CLI print commands to show gyro, accelerometer, magnetometer and roll/pitch/yaw. 
8. Use CLI to calibate the sensor.
9. Connect radio receiver to your development board according to the configured pins.
10. Edit the RC RECEIVER CONFIG section. Either match you RC equipment to the settings, or change the settings to match your RC equipment. 
11. Check your radio setup: Use CLI print commands to show pwm and scaled radio values.
12. Connect motors (no props) and battery and check that motor outputs are working correctly. For debugging, use CLI to show motor output.
13. Mount props, go to an wide open space, and FLY!


<a name="safetyfirst"></a>
## Safety First!!!

By default madflight has these safety features enabled:

- Motors only rotate when armed.
- Arming Procedure: set throttle low then flip the arm switch from disarmed to armed.
- Kill Switch: when the arm switch is in the disarm position, disarm and stop motors until re-armed.
- Failsafe: when radio connection is lost, disarm and stop motors until re-armed.
- Armed Low Throttle: motors run at low speed, to give visible armed indication.
- LED armed/disarmed indicator.


<a name="softwaredesign"></a>
## Software Design

- Keep it simple!!!
- Forked from [dRehmFlight](https://github.com/nickrehm/dRehmFlight)
- Coded primarily for readability, then for speed and code size.
- No external dependencies, all modules are included in the `src/madflight` directory.
- The madflight flight controller runs standard `setup()` and `loop()`.
- It mainly uses plain Arduino functionality: Serial, Wire, and SPI. One custom hardware dependent library is used for PWM. Therefor, it can fairly easily ported to other 32 bit microcontrollers that support the Arduino framework. Also porting to other build environments like PlatformIO or CMake should not be a huge effort.
- The following modules are used:
  - `loop` Main loop control
  - `imu` Inertial Measurement Unit, retrieves accelerometer, gyroscope, and magnetometer sensor data
  - `ahrs` Attitude Heading Reference System, estimates roll, yaw, pitch
  - `rcin` RC INput, retrieves RC receiver data
  - `control` PID controller and output mixer
  - `out` Output to motors and servos
  - `mag` Magnetometer (external)
  - `baro` Barometer
  - `gps` GPS receiver
  - `bb` Black Box data logger
  - `cli` Command Line Interface for debugging, configuration and calibration
  - `cfg` Read and save configuration to flash
  - `hw` Hardware specific code for STM32, RP2040 and ESP32
- Most modules are interfaced through a global object, for example the `imu` object has property `imu.gx` which is the current gyro x-axis rate in degrees per second for the selected IMU chip.
- For a quick overview of the objects, see header `src/madflight/interfaces.h` which defines the module interfaces.
- The module implementations are in subdirectories of the `src/madflight` directory. Here you find the module header file, e.g. `src/madflight/imu/imu.h`. In the `extras` directory your find test programs for the modules, e.g. `extras/TestMadflight/imu.ino`.
- The module files are usually header only, that is, the header also includes the implemention.


<a name="connectsensor"></a>
## Connecting the IMU Sensor

SPI sensor: (highly recommended over I2C)
```
  Sensor       Dev Board
SCL/SCLK <---> SPI_SCLK
 SDA/SDI <---> SPI_MOSI
 ADD/SDO <---> SPI_MISO
     NCS <---> IMU_CS
     INT <---> IMU_EXTI
     VCC <---> 3V3
     GND <---> GND
```
I2C sensor:
```
  Sensor       Dev Board
     SCL <---> I2C_SCL 
     SDA <---> I2C_SDA
     INT <---> IMU_EXTI
     VCC <---> 3V3
     GND <---> GND
```


<a name="pinoutESP32"></a>
## Default Pinout for ESP32 - DevKitC (38 pin)

This pinout can be changed as needed in madflight_board_default_ESP32.h

| Function | GPIO | Board | GPIO | Function |
| --: | :-- | -- |--: | :-- |
| 3V3 out      | 3V3 | Antenna side            |  GND | GND
| reset button | EN |                            | 23 | I2C_SDA
| SPI_MISO     | VP 36 input only |              | 22 | I2C_SCL
| IMU_EXTI     | VN 39 input only |            | 1 TX | USB Serial Debug TX
| BAT_V        | 34 input only |               | 3 RX | USB Serial Debug RX
| RCIN_RX      | 35 input only |                 | 21 | SPI_MOSI
| RCIN_TX      | 32 |                           | GND | GND
| PWM1         | 33 |                            | 19 | SPI_SCLK
| PWM2         | 25 |                            | 18 | IMU_CS
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


<a name="pinoutRP2040"></a>
## Default Pinout for RP2040 - Raspberry Pi Pico (40 pin)

This pinout can be changed as needed in madflight_board_default_RP2040.h

| Function | GPIO | Board | GPIO | Function |
| --: | :-- | -- |--: | :-- |
|      RCIN_TX | 0   | USB connector | VBUS     | nc
|      RCIN_RX | 1   |               | VSYS     | 5V input via diode (*)
|            - | GND |               | GND      | -
|         PWM1 | 2   |               | EN       | nc
|         PWM2 | 3   |               | 3.3V out | 3V3
|         PWM3 | 4   |               | VREF     | nc
|         PWM4 | 5   |               | 28_A2    | BAT_V
|            - | GND |               | GND      | -
|         PWM5 | 6   |               | 27_A1    | FREE
|         PWM6 | 7   |               | 26_A0    | FREE
|       GPS_TX | 8   |               | RUN      | reset button to GND
|       GPS_RX | 9   |               | 22       | IMU_EXTI
|            - | GND |               | GND      | -
|         PWM7 | 10  |               | 21       | I2C_SCL
|         PWM8 | 11  |               | 20       | I2C_SDA
|         PWM9 | 12  |               | 19       | SPI_MOSI
|        PWM10 | 13  |               | 18       | SPI_SCLK
|            - | GND |               | GND      | -
|        PWM11 | 14  |               | 17       | IMU_CS
|        PWM12 | 15  | JTAG pins     | 16       | SPI_MISO

(*) 5V input via diode from BEC. Without a diode take care not connect USB and the battery at the same time!

<img src="doc/img/Raspberry-Pi-Pico-rp2040-pinout-mischianti.png" width="45%" /> <img src="doc/img/Raspberry-Pi-Pico-W-rp2040-WiFi-pinout-mischianti.png" width="46.8%" />


<a name="pinoutSTM32"></a>
## Default Pinout for STM32 - WeAct STM32F411 Black Pill (40 pin)

This pinout can be changed as needed in madflight_board_default_STM32.h

| Function | GPIO | Board | GPIO | Function |
| --: | :-- | -- |--: | :-- |
|            - | VB  |   SWD pins    | 3V3 | -
|          LED | C13 |               | G   | -
|         FREE | C14 |               | 5V  | 5V input (*)
|         FREE | C15 |               | B9  | PWM10(t4)
|            - | R   |               | B8  | PWM9(t4)
|         FREE | A0  |               | B7  | I2C_SCL
|         FREE | A1  |               | B6  | I2C_SDA
|       GPS_TX | A2  |               | B5  | PWM8(t3)
|       GPS_RX | A3  |               | B4  | PWM7(t3)
|       IMU_CS | A4  |               | B3  | RCIN_RX
|     SPI_SCLK | A5  |               | A15 | RCIN_TX
|     SPI_MISO | A6  |               | A12 | USB_DP
|     SPI_MOSI | A7  |               | A11 | USB_DN
|        BAT_I | B0  |               | A10 | PWM6(t1)
|        BAT_V | B1  |               | A9  | PWM5(t1)
|         FREE | B2  |               | A8  | PWM4(t1)
|     IMU_EXTI | B10 |               | B15 | PWM3(t1)
|            - | 3V3 |               | B14 | PWM2(t1)
|            - | G   |               | B13 | PWM1(t1)
|            - | 5V  | USB connector | B12 | FREE

Board: LED: C13, key button: A0

PWM1-6 are connected to timer1, PWM7-8 to timer3 and PWM9-10 to timer4. PWM pins connected to the same timer operate at the same frequency.

(*) 5V input via diode from BEC. Without a diode take care not connect USB and the battery at the same time!

<img src="doc/img/STM32-STM32F4-STM32F411-STM32F411CEU6-pinout-high-resolution.png" width="45%" />


<a name="pinoutESP32-S3"></a>
## Default Pinout for ESP32-S3 - DevKitC-1 (44 pin)

This pinout can be changed as needed in madflight_board_default_ESP32-S3.h

| Function | GPIO | Board | GPIO | Function |
| --: | :-- | -- |--: | :-- |
3V3 out | 3V3 | Antenna side | G | GND
3V3 out | 3V3 | | 43 | TX serial debug UART port
reset button | RST | | 44 | RX serial debug UART port
PWM1 | 4 | | 1 | -
PWM2 | 5 | | 2 | -
PWM3 | 6 | | 42 | -
PWM4 | 7 | | 41 | -
PWM5 | 15 | | 40 | -
PWM6 | 16 | | 39 | -
RCIN_TX | 17 | | 38 | LED
RCIN_RX | 18 | | 37 | -
I2C_SDA | 8 | | 36 | -
GPS_RX | 3 | | 35 | -
GPS_TX | 46 | | 0 | boot button
I2C_SCL | 9 | | 45| -
IMU_CS | 10 | | 48 | RGB_LED
SPI_MOSI | 11 | | 47 | -
MISO | 12 | | 21 | -
SCLK | 13 | | 20 | USB_D+ (serial debug alternate)
IMU_EXTI | 14 | | 19 | USB_D- (serial debug alternate)
5V in (*) | 5V | | G | GND
GND | G | USB connector | G | GND

(*) 5V input via diode from BEC. Without a diode take care not connect USB and the battery at the same time!

<img src="doc/img/esp32-S3-DevKitC-1-original-pinout-high.png" width="60%" />


<a name="pinoutFC"></a>
## Pinout for Of-the-shelf Flight Controllers

In the `src` directory you'll find header files for 400+ commercial flight controllers. These are converted Betaflight configuration files. Include the header file you want to use, and modify the 'USE' defines like IMU_USE to match your board. 

<a name="changes"></a>
## Changes from dRehmFlight

- Add support for RP2040, ESP32, ESP32-S3, and STM32
- Dropped Teensy support, but could be re-added by creating a hw_TEENSY.h file. (I just don't have the hardware to test on)
- Moved all hardware specific code to hw_XXX.h and added hardware specific libraries
- Reduced the number of global variables
- Oneshot is implemented as PWM up to 3.9kHz
- New libs for IMU sensors
- Changed arming logic
- Loop rate set to 1kHz to match IMU sensor rate
- Interrupt driven IMU operation by default, but setup/loop still possible


<a name="github"></a>
## Flight Controllers on Github

In increasing order of complexity.

- [lobodol/drone-flight-controller](https://github.com/lobodol/drone-flight-controller) Arduino ATmega328P, single 700 line ino file, no libs
- [dRehmFlight](https://github.com/nickrehm/dRehmFlight) Arduino Teensy 4
- [madflight](https://github.com/qqqlab/madflight) Arduino RP2040, ESP32, and STM32
- [Crazyflie](https://github.com/bitcraze/crazyflie-firmware) STM32F405
- [esp-drone](https://github.com/espressif/esp-drone.git) ESP32 (forked from Crazyflie)
- [Betaflight](https://github.com/betaflight/betaflight) STM32 F4/F7/H7
- [inav](https://github.com/iNavFlight/inav) STM32 F4/F7/H7
- [Ardupilot](https://github.com/ArduPilot/ardupilot) STM32 F4/F7/H7 or Linux based


<a name="disclamer"></a>
## Disclaimer

This code is a shared, open source flight controller for small micro aerial vehicles and is intended to be modified to suit your needs. It is NOT intended to be used on manned vehicles. I do not claim any responsibility for any damage or injury that may be inflicted as a result of the use of this code. Use and modify at your own risk. More specifically put:

THIS SOFTWARE IS PROVIDED BY THE CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Logo image copyright (c) 1975 Deutsches MAD Magazine. This project is not associated with MAD Magazine.
