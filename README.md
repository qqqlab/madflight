<img src="extras/img/logo/madflight_logo_2000x538.png" width="100%" />

**madflight** is an Arduino library to build ESP32 / ESP32-S3 / RP2350 / RP2040 / STM32 flight controllers. A functional DIY flight controller can be build for under $10 from readily available [development boards](https://madflight.com/Controller-Boards/) and [sensor breakout boards](https://madflight.com/Sensor-Boards/). Ideal if you want to try out new flight control concepts, without first having to setup a build environment and without having to read through thousands lines of code to find the spot where you want to change something.

`Quadcopter.ino` is a 1000 line demo program for a quadcopter. It has been flight tested on ESP32, ESP32-S3, RP2350, RP2040, and STM32F405 microcontrollers with the Arduino IDE. The program can be easily adapted to control your plane or VTOL craft. The source code has extensive documentation explaning what the settings and functions do.

---
<p align="center">If you like <b>madflight</b>, give it a &star; star, or fork it and contribute!</p>

---

<img src="extras/img/madflight RP2040 flight controller.jpeg" title="madflight RP2040 flight controller" width="38%" /> <img src="extras/img/madflight drone.jpeg" title="madflight drone" width="30%" /> <img src="extras/img/madflight ESP32 flight controller.jpeg" title="madflight ESP32 flight controller" width="29%" />

## Required Hardware

- [Development board](https://madflight.com/Controller-Boards/): 
    - [RP2350/RP2040](https://madflight.com/Board-RP2040/) (e.g. Raspberry Pi Pico/Pico2)
    - [ESP32-S3/ESP32](https://madflight.com/Board-ESP32/) (e.g. Espressiv ESP32/ESP32-S3 DevKitC)
    - [STM32](https://madflight.com/Board-STM32/) (e.g. Black Pill or a commercial flight controller)
- [SPI IMU sensor](https://madflight.com/Sensor-Boards/) (BMI270, MPU9250, MPU6500, or MPU6000), if not available then use an I2C IMU sensor (MPU6050 or MPU9150) 
- RC Receiver: ELRS, CRSF, SBUS, DMSX, or PPM
- BEC or DC-DC converter to power your board from a battery
- ESC (OneShot125 or 50-490Hz PWM) and/or servos (50-490Hz PWM)

## Optional Hardware

- GPS Module (Serial)
- Barometer (I2C BMP390, BMP388, BMP280, MS5611)
- Magnetometer (I2C QMC5883L)
- Current/Voltage Sensor (ADC or I2C INA226)
- [Optical Flow Sensor](https://github.com/qqqlab/ESP32-Optical-Flow) (I2C)

## Getting Started

1. Connect the required hardware to your controller board: 
    - See [RP2350/RP2040 pinout and instructions](https://madflight.com/Board-RP2040/)
    - -or- [ESP32-S3/ESP32 pinout and instructions](https://madflight.com/Board-ESP32/)
    - -or- [STM32 pinout and instructions](https://madflight.com/Board-STM32/)
    - Connect your IMU (gyro/acceleration) sensor as shown [here](https://madflight.com/).
    - Connect your radio receiver according to the configured pins.
2. Install the madflight library in Arduino IDE. (Menu *Tools->Manage Libraries*, then search for **madflight**)
3. Open *Examples for custom libraries->madflight->Quadcopter.ino* in the Arduino IDE.
4. If you're not using the default pinout then setup your board pinout in the CUSTOM PINS section.
5. Edit the HARDWARE section to enable the connected peripherals.
6. Edit the RC RECEIVER section. Either match you RC equipment to the settings, or change the settings to match your RC equipment. 
7. Compile Quadcopter.ino and upload it to your board. Connect the Serial Monitor at 115200 baud and check the messages. Type `help` to see the available CLI commands.
8. IMPORTANT: Use CLI `calimu` and `calmag` to calibate the sensors.
9. Use CLI print commands like `pimu`, `pahrs`, `pradio` to check that IMU sensor, AHRS and RC Receiver are working correctly. 
10. Connect motors (no props) and battery and check that motors are spinning correctly. For debugging, use CLI `pmot` to show motor output.
11. Mount props, go to an wide open space, and FLY!

## Safety First!!!

By default **madflight** has these safety features enabled:

- Motors only rotate when armed.
- Arming Procedure: set throttle low then flip the arm switch from disarmed to armed.
- Kill Switch: when the arm switch is in the disarm position, disarm and stop motors until re-armed.
- Failsafe: when radio connection is lost, disarm and stop motors until re-armed.
- Armed Low Throttle: motors run at low speed, to give visible armed indication.
- LED armed/disarmed indicator.

## Software Design

- Keep it simple!!!
- Based on [dRehmFlight](https://github.com/nickrehm/dRehmFlight)
- Coded primarily for readability, then for speed and code size.
- No external dependencies, all modules are included in the `src/madflight` directory.
- The madflight flight controller runs standard `setup()` and `loop()`.
- It mainly uses plain Arduino functionality: Serial, Wire, and SPI. One custom hardware dependent library is used for PWM. Therefor, it can fairly easily ported to other 32 bit microcontrollers that support the Arduino framework. Also porting to other build environments like PlatformIO or CMake should not be a huge effort.
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

## Disclaimer

This code is a shared, open source flight controller for small micro aerial vehicles and is intended to be modified to suit your needs. It is NOT intended to be used on manned vehicles. I do not claim any responsibility for any damage or injury that may be inflicted as a result of the use of this code. Use and modify at your own risk. More specifically put:

THIS SOFTWARE IS PROVIDED BY THE CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Logo image copyright (c) 1975 Deutsches MAD Magazine. This project is not associated with MAD Magazine.
