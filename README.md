<img src="extras/img/logo/madflight_logo_2000x538.png" width="100%" />

***M**anless **A**erial **D**evice*

This is an Arduino library to build ESP32 / ESP32-S3 / RP2350 / RP2040 / STM32 flight controllers. A functional DIY flight controller can be build for under $10 from readily available development boards and sensor breakout boards. Ideal if you want to try out new flight control concepts, without first having to setup a build environment and without having to read through thousands lines of code to find the spot where you want to change something.

`Quadcopter.ino` is a 1000 line demo program for a quadcopter. It has been flight tested on ESP32, ESP32-S3, RP2040, and STM32F405 microcontrollers with the Arduino IDE. The program can be easily adapted to control your plane or VTOL craft. The source code has extensive documentation explaning what the settings and functions do.

<img src="extras/img/madflight RP2040 flight controller.jpeg" title="madflight RP2040 flight controller" width="25%" /> <img src="extras/img/madflight drone.jpeg" title="madflight drone" width="19.6%" /> <img src="extras/img/madflight ESP32 flight controller.jpeg" title="madflight ESP32 flight controller" width="19.1%" />



## Feedback is Welcome

I enjoy hacking with electronics and I'm attempting to write some decent code for this project. If you enjoy it as well, please leave some feedback in the form of Stars, Issues, Pull Requests, or Discussions. Thanks!



## Documentation

For full documention see [madflight.com](https://madflight.com) and the source code itself.



## Quick Start

[Required Hardware](#hardware)  
[Getting Started](#gettingstarted)  
[Safety First](#safetyfirst)  
[ESP32 / ESP32-S3 Setup and Pinout](#ESP32)  
[RP2350 / RP2040 Setup and Pinout](#RP2040)  
[STM32 Setup and Pinout](#STM32)  
[Connecting the IMU Sensor](#connectsensor)  
[Software Design](#softwaredesign)  
[Changes from dRehmFlight](#changes)  
[Flight Controllers on Github](#github)  
[Disclaimer](#disclamer)  



<a name="hardware"></a>
## Required Hardware

- Development board: 
  - RP2350 RP2040 (e.g. Raspberry Pi Pico 2)
  - or ESP32/ESP32-S3 (e.g. Espressiv DevKitC)
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
6. Upload Quadcopter.ino to your board. Connect the Serial Monitor at 115200 baud and check the messages. Type `help` to see the available CLI commands.
7. Use CLI print commands like `pimu`, `pgyro`, `proll` to Check that IMU sensor and AHRS are working correctly. 
8. IMPORTANT: Use CLI `calimu` and `calmag` to calibate the sensors.
9. Connect radio receiver to your development board according to the configured pins.
10. Review the RC RECEIVER CONFIG section. Either match you RC equipment to the settings, or change the settings to match your RC equipment. 
11. Check your radio setup: Use CLI `ppwm` and `pradio` to show pwm and scaled radio values.
12. Connect motors (no props) and battery and check that motor outputs are working correctly. For debugging, use CLI `pmot` to show motor output.
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



<a name="ESP32"></a>
## ESP32 / ESP32-S3 Setup and Pinout

madflight works with [Arduino-ESP32 v3.x.x](https://github.com/espressif/arduino-esp32)

madflight v1.1.2 and earlier works with Arduino-ESP32 v2.x.x

#### Dual Core / FPU / FreeRTOS

ESP32 and ESP32-S3 have dual core CPU, but single core FPU. ESP-IDF implementation limits [float usage](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos_idf.html#floating-point-usage) to a single core, and float can not be used in interrupts. FreeRTOS is always enabled and a watchdog limits interrupt execution time.

madflight uses float and is therefor limited to single core operation. The IMU loop runs as a high priorty task, triggered by the IMU interrupt.

#### I2C
Arduino-ESP32 v2.0.17 has an [I2C bug](https://github.com/espressif/esp-idf/issues/4999) which causes the bus to hang for 1 second after a failed read, which can happen a couple times per minute. This makes Wire I2C for IMU not a real option...

A workaround is to use #define USE_ESP32_SOFTWIRE which enables software I2C, but this does not work well with all sensors.
  
(!) So, until a better I2C solution is available: use an SPI IMU sensor on ESP32.

NOTE: as of June 2024 this bug is apparently fixed, but not yet confirmed with madflight.


## Pinout ESP32

This is the default pinout for ESP32. It is optimized for the Espressiv ESP32 DevKitC (38 pin) board. This pinout is defined in madflight_board_default_ESP32.h, but can be modified with `#define HW_PIN_XXX` in your program.

| Function | GPIO | Board | GPIO | Function |
| --: | :-- | :--: | --: | :-- |
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
| 5V input (*) | 5V | USB connector     | flash 6 CLK | nc

Note: During boot the input voltage levels (pull up/pull down) on strap pins have a configuration function, therefor these pins are used as output only.

(*) 5V input via diode from BEC. Without a diode take care not connect USB and the battery at the same time!

<img src="extras/img/ESP32-DEV-KIT-DevKitC-v4-pinout-mischianti.png" width="60%" />



## Pinout ESP32-S3

This is the default pinout for ESP32-S3. It is optimized for the Espressiv ESP32-S3 DevKitC-1 (44 pin) board. This pinout is defined in madflight_board_default_ESP32-S3.h, but can be modified with `#define HW_PIN_XXX` in your program.

| Function | GPIO | Board | GPIO | Function |
| --: | :-- | :--: | --: | :-- |
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

<img src="extras/img/esp32-S3-DevKitC-1-original-pinout-high.png" width="60%" />



<a name="RP2040"></a>
## RP2350 / RP2040 Setup and Pinout

madflight works with [arduino-pico v4.x.x](https://github.com/earlephilhower/arduino-pico)

#### Dual Core / FPU / FreeRTOS

RP2350 (Pico2) has dual core, dual FPU, but no FreeRTOS (arduino pico v4.0.1)

RP2040 (Pico) has dual core, no FPU, and FreeRTOS is optional.

madflight uses float and is much happier with RP2350 over RP2040.

madflight on RP2040 uses FreeRTOS and executes the 1000Hz IMU loop on the second core, which is 80% loaded at default 133MHz CPU speed. You can overclock the CPU to get some more headroom.

#### PWM

Consecutive even/odd PWM pins (e.g. pins 2,3 or 10,11) share the same timer and have the same frequency.

#### Serial

madflight uses a custom high performance SerialIRQ library.

## Pinout Raspberry Pi Pico / Pico2

This is the default pinout for RP2040. It is optimized for the Raspberry Pi Pico (40 pin) board. This pinout is defined in madflight_board_default_RP2040.h, but can be modified with `#define HW_PIN_XXX` in your program.

| Function | GPIO | Board | GPIO | Function |
| --: | :-- | :--: | --: | :-- |
|      RCIN_TX | 0   | USB connector | VBUS     | nc
|      RCIN_RX | 1   |               | VSYS     | 5V input via diode (*)
|          GND | GND |               | GND      | GND
|         PWM1 | 2   |               | EN       | nc
|         PWM2 | 3   |               | 3.3V out | 3V3
|         PWM3 | 4   |               | VREF     | nc
|         PWM4 | 5   |               | 28_A2    | BAT_V
|          GND | GND |               | GND      | GND
|         PWM5 | 6   |               | 27_A1    | -
|         PWM6 | 7   |               | 26_A0    | -
|       GPS_TX | 8   |               | RUN      | reset button to GND
|       GPS_RX | 9   |               | 22       | IMU_EXTI
|          GND | GND |               | GND      | GND
|         PWM7 | 10  |               | 21       | I2C_SCL
|         PWM8 | 11  |               | 20       | I2C_SDA
|         PWM9 | 12  |               | 19       | SPI_MOSI
|        PWM10 | 13  |               | 18       | SPI_SCLK
|          GND | GND |               | GND      | GND
|        PWM11 | 14  |               | 17       | IMU_CS
|        PWM12 | 15  | JTAG pins     | 16       | SPI_MISO

(*) 5V input via diode from BEC. Without a diode take care not connect USB and the battery at the same time!

<img src="extras/img/Raspberry-Pi-Pico-rp2040-pinout-mischianti.png" width="45%" /> <img src="extras/img/Raspberry-Pi-Pico-W-rp2040-WiFi-pinout-mischianti.png" width="46.8%" />

## Pinout RP2040-Zero

This pinout is optimized for the RP2040-Zero (21 pin) board. This pinout is defined in madflight_board_RP2040-Zero.h, but can be modified with `#define HW_PIN_XXX` in your program.

| Function | GPIO | Board | GPIO | Function |
| --: | :-- | :--: | --: | :-- |
| 5V input (*) | 5V  | USB connector | 0 | RCIN_TX
|          GND | GND |               | 1  | RCIN_RX
|      3V3 out | 3V3 |               | 2  | PWM1
|            - | 29  |               | 3  | PWM2
|        BAT_V | 28  |               | 4  | PWM3
|     I2C1_SCL | 27  |               | 5  | PWM4
|     I2C1_SDA | 26  |               | 6  | PWM5
|     IMU_EXTI | 15  |               | 7  | PWM6
|              | 14  |               | 8  | GPS_TX
|              |     |               |    | 
|              |     |               | 9  | GPS_RX
|              |     |               | 10 | SPI1_SCLK
|              |     |               | 11 | SPI1_MISO
|              |     |               | 12 | SPI1_MOSI
|              |     |               | 13 | IMU_CS

(*) 5V input via diode from BEC. Without a diode take care not connect USB and the battery at the same time!

<img src="extras/img/RP2040-Zero.jpg" width="45%" />



<a name="STM32"></a>
## STM32 Setup and Pinout

madflight works with [stm32duino Arduino Core for STM32 v2.x.x](https://github.com/stm32duino/Arduino_Core_STM32)

#### Dual Core / FPU / FreeRTOS

Most supported STM32 targets have a single cores with FPU. FreeRTOS support optional.

madflight runs the IMU loop in interrupt context.



## Pinout STM32 Of-the-shelf Flight Controllers

In the `src` directory you'll find 400+ Betaflight configuration files for commercial flight controllers. Include the madflight_board_betaflight_XXX.h header file of your board, and in your program set '#define HW_USE_XXX' to match your board. 



## Pinout STM32 Black Pill

This is the default pinout for STM32. It is optimized for the WeAct STM32F411 Black Pill (40 pin) board. This pinout is defined in madflight_board_default_STM32.h, but can be modified with `#define HW_PIN_XXX` in your program.

| Function | GPIO | Board | GPIO | Function |
| --: | :-- | :--: | --: | :-- |
|           nc | VB  |   SWD pins    | 3V3 | 3V3 out
|          LED | C13 |               | G   | GND
|            - | C14 |               | 5V  | 5V input (*)
|            - | C15 |               | B9  | PWM10(t4)
|           nc | R   |               | B8  | PWM9(t4)
|            - | A0  |               | B7  | I2C_SCL
|            - | A1  |               | B6  | I2C_SDA
|       GPS_TX | A2  |               | B5  | PWM8(t3)
|       GPS_RX | A3  |               | B4  | PWM7(t3)
|       IMU_CS | A4  |               | B3  | RCIN_RX
|     SPI_SCLK | A5  |               | A15 | RCIN_TX
|     SPI_MISO | A6  |               | A12 | USB_DP
|     SPI_MOSI | A7  |               | A11 | USB_DN
|        BAT_I | B0  |               | A10 | PWM6(t1)
|        BAT_V | B1  |               | A9  | PWM5(t1)
|            - | B2  |               | A8  | PWM4(t1)
|     IMU_EXTI | B10 |               | B15 | PWM3(t1)
|      3V3 out | 3V3 |               | B14 | PWM2(t1)
|          GND | G   |               | B13 | PWM1(t1)
| 5V input (*) | 5V  | USB connector | B12 | -

Internally connected: C13 - LED, A0 - key button

PWM1-6 are connected to timer1, PWM7-8 to timer3 and PWM9-10 to timer4. PWM pins connected to the same timer operate at the same frequency.

(*) 5V input via diode from BEC. Without a diode take care not connect USB and the battery at the same time!

<img src="extras/img/STM32-STM32F4-STM32F411-STM32F411CEU6-pinout-high-resolution.png" width="45%" />



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



<a name="softwaredesign"></a>
## Software Design

- Keep it simple!!!
- Forked from [dRehmFlight](https://github.com/nickrehm/dRehmFlight)
- Coded primarily for readability, then for speed and code size.
- No external dependencies, all modules are included in the `src/madflight` directory.
- The madflight flight controller runs standard `setup()` and `loop()`.
- It mainly uses plain Arduino functionality: Serial, Wire, and SPI. This makes it possible to have one code base for very different microcontroller platforms.
- Hardware platform specific libraries are used for PWM and to work around issues in the standard Arduino libraries.
- madflight can fairly easily ported to other 32 bit microcontrollers that support the Arduino framework. Also porting to other build environments like PlatformIO or CMake should not be a huge effort.
- The following modules are used:
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



<a name="changes"></a>
## Changes from dRehmFlight

- Add support for RP2040, RP2350, ESP32, ESP32-S3, and STM32
- Dropped Teensy support, but could be re-added by creating a hw_TEENSY.h file. (I just don't have the hardware to test on)
- Moved all hardware specific code to hw_XXX.h and added hardware specific libraries
- Reduced the number of global variables
- Oneshot is implemented as PWM up to 3.9kHz
- New libs for IMU sensors
- Changed arming logic
- Loop rate set to 1kHz to match IMU sensor rate
- Interrupt driven IMU operation, not setup/loop


<a name="github"></a>
## Flight Controllers on Github

In (approximate) increasing order of complexity.

| Flight Controller | Features | Development Environment | Microcontroller |
|:-|:-|:-|:-|
[drone-flight-controller](https://github.com/lobodol/drone-flight-controller) | Single 700 line ino file, no libs | Arduino | ATmega328P
[dRehmFlight](https://github.com/nickrehm/dRehmFlight) | Quad, Plane, VTOL | Arduino | Arduino Teensy 4
[madflight](https://github.com/qqqlab/madflight) | Quad, Plane, VTOL, based on dRehmFlight | Arduino | ESP32, RP2040, and STM32
[esp-fc](https://github.com/rtlopez/esp-fc) | FPV Quad | PlatformIO | ESP32
[Crazyflie](https://github.com/bitcraze/crazyflie-firmware) | FPV Quad | | STM32F405
[esp-drone](https://github.com/espressif/esp-drone) | FPV Quad, based on Crazyflie | | ESP32 
[Betaflight](https://github.com/betaflight/betaflight) | FPV Quad, based on cleanflight | | STM32 F4/F7/H7
[EmuFlight](https://github.com/emuflight/EmuFlight) | Multi-rotor, based on cleanflight | | STM32 F4/F7
[inav](https://github.com/iNavFlight/inav) | Plane, based on cleanflight | | STM32 F4/F7/H7
[Ardupilot](https://github.com/ArduPilot/ardupilot) | Quad, Plane, VTOL | Linux waf | STM32 F4/F7/H7 or Linux based
[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) | Quad, Plane, VTOL | | STM32 F4/F7/H7 |


<img src="https://api.star-history.com/svg?repos=qqqlab/madflight,rtlopez/esp-fc,lobodol/drone-flight-controller,emuflight/EmuFlight,espressif/esp-drone,nickrehm/dRehmFlight&type=Date" width="48%" /> <img src="https://api.star-history.com/svg?repos=bitcraze/crazyflie-firmware,iNavFlight/inav,PX4/PX4-Autopilot,betaflight/betaflight,ArduPilot/ardupilot&type=Date" width="48%" />


<a name="disclamer"></a>
## Disclaimer

This code is a shared, open source flight controller for small micro aerial vehicles and is intended to be modified to suit your needs. It is NOT intended to be used on manned vehicles. I do not claim any responsibility for any damage or injury that may be inflicted as a result of the use of this code. Use and modify at your own risk. More specifically put:

THIS SOFTWARE IS PROVIDED BY THE CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Logo image copyright (c) 1975 Deutsches MAD Magazine. This project is not associated with MAD Magazine.
