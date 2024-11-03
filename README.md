<img src="extras/img/logo/madflight_logo_2000x538.png" width="100%" />

***M**anless **A**erial **D**evice*

This is an Arduino library to build ESP32 / ESP32-S3 / RP2350 / RP2040 / STM32 flight controllers. A functional DIY flight controller can be build for under $10 from readily available development boards and sensor breakout boards. Ideal if you want to try out new flight control concepts, without first having to setup a build environment and without having to read through thousands lines of code to find the spot where you want to change something.

`Quadcopter.ino` is a 1000 line demo program for a quadcopter. It has been flight tested on ESP32, ESP32-S3, RP2350, RP2040, and STM32F405 microcontrollers with the Arduino IDE. The program can be easily adapted to control your plane or VTOL craft. The source code has extensive documentation explaning what the settings and functions do.

<img src="extras/img/madflight RP2040 flight controller.jpeg" title="madflight RP2040 flight controller" width="25%" /> <img src="extras/img/madflight drone.jpeg" title="madflight drone" width="19.6%" /> <img src="extras/img/madflight ESP32 flight controller.jpeg" title="madflight ESP32 flight controller" width="19.1%" />



## Feedback is Welcome

I enjoy hacking with electronics and I'm attempting to write some decent code for this project. If you enjoy it as well, please leave some feedback in the form of Stars, Issues, Pull Requests, or Discussions. Thanks!


## Getting Started

See [madflight.com](https://madflight.com) for detailed instructions to get flying. 


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


## Disclaimer

This code is a shared, open source flight controller for small micro aerial vehicles and is intended to be modified to suit your needs. It is NOT intended to be used on manned vehicles. I do not claim any responsibility for any damage or injury that may be inflicted as a result of the use of this code. Use and modify at your own risk. More specifically put:

THIS SOFTWARE IS PROVIDED BY THE CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Logo image copyright (c) 1975 Deutsches MAD Magazine. This project is not associated with MAD Magazine.
