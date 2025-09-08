<img src="https://raw.githubusercontent.com/qqqlab/madflight/refs/heads/main/extras/img/logo/madflight_logo_2000x538.png" width="100%" />

<p align="center">&star;&nbsp;&star;&nbsp;&star;&nbsp;</p>
<p align="center">If you like <i>madflight</i>, please give it a &star; star</p>
<p align="center">or fork it and contribute!</p>
<p align="center">&star;&nbsp;&star;&nbsp;&star;&nbsp;</p>

_madflight_ is a toolbox to build high performance flight controllers with Aduino IDE or PlatformIO for ESP32-S3 / ESP32 / RP2350 / RP2040 / STM32. A functional DIY flight controller can be build for under $10 from readily available [development boards](https://madflight.com/Controller-Boards/) and [sensor breakout boards](https://madflight.com/Sensor-Boards/).

Get started with the [Arduino IDE](https://madflight.com/Getting-Started) or [PlatformIO](https://madflight.com/Getting-Started)

Flight tested example programs for quadcopter and airplane are included. The example programs are only a couple hundred lines long, but contain the full flight controller logic. The nitty-gritty low-level sensor and input/output management is done by the _madflight_ library.

The source code and [website](https://madflight.com/) have extensive documentation explaning what the settings and functions do.

<img src="https://raw.githubusercontent.com/qqqlab/madflight/refs/heads/main/extras/img/madflight RP2040 flight controller.jpeg" title="madflight RP2040 flight controller" width="38%" /> <img src="https://raw.githubusercontent.com/qqqlab/madflight/refs/heads/main/extras/img/madflight drone.jpeg" title="madflight drone" width="30%" /> <img src="https://raw.githubusercontent.com/qqqlab/madflight/refs/heads/main/extras/img/madflight ESP32 flight controller.jpeg" title="madflight ESP32 flight controller" width="29%" />

## DEV Version

If you clone/download this repository you get the DEVELOPMENT version, which is BLEEDING EDGE - not flight tested at all, might not even compile, and will completely change in the next hour... 

Use a [release version](https://github.com/qqqlab/madflight/releases) if you want something that actually has logged flight hours.

## Required Hardware

- [Development board](https://madflight.com/Controller-Boards/): 
    - [RP2350/RP2040](https://madflight.com/Board-RP2040/) (e.g. Raspberry Pi Pico/Pico2)
    - [ESP32-S3/ESP32](https://madflight.com/Board-ESP32/) (e.g. Espressiv ESP32/ESP32-S3 DevKitC)
    - [STM32](https://madflight.com/Board-STM32/) (e.g. Black Pill or a commercial flight 
- [IMU sensor](https://madflight.com/Sensor-Boards/): MPU6000, MPU6050, MPU6500, MPU9250, BMI270, ICM42688P, etc.
- RC Receiver: MAVLink, ELRS, CRSF, SBUS, DMSX, or PPM

## Optional Hardware

- GPS Module (Serial)
- Barometer (I2C BMP390, BMP388, BMP280, MS5611)
- Magnetometer (I2C QMC5883L)
- Current/Voltage Sensor (ADC or I2C INA226, INA228)
- [Optical Flow Sensor](https://github.com/qqqlab/ESP32-Optical-Flow) (I2C)

## Getting Started

See [Getting Started](https://madflight.com/Getting-Started/) 

For additional help see [Discussions](https://github.com/qqqlab/madflight/discussions)

## Disclaimer

This code is a shared, open source flight controller for small micro aerial vehicles and is intended to be modified to suit your needs. It is NOT intended to be used on manned vehicles. I do not claim any responsibility for any damage or injury that may be inflicted as a result of the use of this code. Use and modify at your own risk. More specifically put:

THIS SOFTWARE IS PROVIDED BY THE CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Logo image copyright (c) 1975 Deutsches MAD Magazine. This project is not associated with MAD Magazine.
