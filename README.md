<img src="https://raw.githubusercontent.com/qqqlab/madflight/refs/heads/main/extras/img/logo/madflight_logo_2000x538.png" width="100%" />

<p align="center">&star;&nbsp;&star;&nbsp;&star;&nbsp;</p>
<p align="center">If you like <i>madflight</i>, please give it a &star; star</p>
<p align="center">&star;&nbsp;&star;&nbsp;&star;&nbsp;</p>

_madflight_ is a toolbox to build high performance flight controllers with PlatformIO or Aduino IDE for ESP32-S3 / ESP32 / RP2350 / RP2040 / STM32. A functional flight controller can be build for under $10 from readily available [development boards](https://madflight.com/Controller-Boards/) and [sensor breakout boards](https://madflight.com/Sensor-Boards/). Or, buy a complete flight controller.

## Getting Started

Build a multicopter or airplane with [PlatformIO](https://madflight.com/Getting-Started) or the [Arduino IDE](https://madflight.com/Getting-Started) 

For additional help see [Discussions](https://github.com/qqqlab/madflight/discussions)

## DEV Version

If you clone/download this repository you get the DEVELOPMENT version, which is BLEEDING EDGE - not flight tested at all, might not even compile, and will completely change in the next hour... 

Download a [release version](https://github.com/qqqlab/madflight/releases) if you want something that actually has logged flight hours.

## What is flying with madflight?

## RP2350 DJI Phantom 1 P330

<img src="https://raw.githubusercontent.com/qqqlab/madflight/refs/heads/main/extras/img/phantom.jpg"/>

Swapped existing NAZA flight controller with a madflight FC3 RP2350. The existing ESCs are recycled and driven in PWM 400Hz mode.

 - DJI Phantom 1 P330
 - [madflight FC3 RP2350 Flight Controller](https://www.tindie.com/products/madflight/flight-controller-raspberry-pi-rp2350b/)
 - ELRS Nano Receiver
 - Short USB-C cable for programming

## ESP32-S3 5" Sub-250 Gram

<img src="https://raw.githubusercontent.com/qqqlab/madflight/refs/heads/main/extras/img/bambu.jpg"/>

Lightweight frame made from 6mm bambu rods + 3D printed PETG motor mounts

 - [madflight FC1 ESP32-S3 Flight Controller](https://www.tindie.com/products/madflight/flight-controller-esp32-s3/)
 - ELRS Nano Receiver
 - ESCs: Favorite LittleBee Spring 20A with BlueJay bidir DSHOT
 - 1503 Motors
 - Folding 5" 120mm props
 - Weight: 201 gr (including 2S 18650 battery)

## ESP32-S3 M5Stack Stampfly

<img src="https://raw.githubusercontent.com/qqqlab/madflight/refs/heads/main/extras/img/stampfly.jpg"/>

 - M5Stack Stampfly
 - ELRS Nano Receiver
 - Openlog SDCard logger

## ESP32-S3 Bixler 3

<img src="https://raw.githubusercontent.com/qqqlab/madflight/refs/heads/main/extras/img/bixler.jpg"/>

 - ESP32-S3 Dev Board
 - MPU9250 Gyro/Acc/Mag Module
 - ELRS receiver
 - Hobbyking Bixler 3 155cm

## RP2040 Quad with 9" Props

<img src="https://raw.githubusercontent.com/qqqlab/madflight/refs/heads/main/extras/img/ifly.jpg"/>

 - Raspberry Pi Pico
 - MPU6500 Gyro/Acc Module
 - BME280 Barometer Module
 - INA226 Current Sensor Module
 - Micro SD Card Module
 - Mini DC-DC 12-20V to 5V 3A Buck Converter
 - uBlox M8 GPS with QMC5883L Compass Compass
 - ELRS receiver
 - DJI E300 Propulsion System (9.4x4.3 props, 2212 920KV motors, 15A ESCs)
 - Frame of a Ideafly IFLY-4 Quadcopter

## ESP32 Dualsky Hornet 460

<img src="https://raw.githubusercontent.com/qqqlab/madflight/refs/heads/main/extras/img/hornet.jpg"/>

This build does not use the default board for ESP32. And a custom pinout is used, so that the MPU6500 board can be soldered directly with pins to the ESP32 board, and just requires the red wire for 3V. The I2C sensor boards are also soldered directly with pins, plus one black ground wire.

 - Lolin ESP32 Lite
 - MPU6500 SPI Gyro/Acc Module
 - ELRS receiver
 - Dualsky Hornet 460 Quadcopter minus original brain
 - BME280 Barometer Module
 - QMC5883L Magnetometer Module

## Disclaimer

This code is a shared, open source flight controller for small micro aerial vehicles and is intended to be modified to suit your needs. It is NOT intended to be used on manned vehicles. I do not claim any responsibility for any damage or injury that may be inflicted as a result of the use of this code. Use and modify at your own risk. More specifically put:

THIS SOFTWARE IS PROVIDED BY THE CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Logo image copyright (c) 1975 Deutsches MAD Magazine. This project is not associated with MAD Magazine.
