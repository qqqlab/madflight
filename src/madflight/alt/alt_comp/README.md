Modified for madflight, original source: https://github.com/juangallostra/AltitudeEstimation

# AltitudeEstimation
A hardware-independent, header-only, C++ arduino library to perform vertical position estimation using an IMU-Barometer system via a two-step Kalman/Complementary filter.

This work is an implementation of the algorithm explained in [this paper](http://www.koreascience.or.kr/article/ArticleFullRecord.jsp?cn=HSSHBT_2016_v25n3_202) written in 2016 by Jung Keun Lee. Although the original is in Korean you can find an English version of it [here](https://home.wlu.edu/~levys/TwoStepFilter.pdf) thanks to [Simon D. Levy](https://simondlevy.academic.wlu.edu/).


## Setup

### Hardware

The current setup used for testing consists in the [Butterfly STM32L433 Development Board](https://www.tindie.com/products/TleraCorp/butterfly-stm32l433-development-board/) plus the [MPU9250 Mini Add-On](https://www.tindie.com/products/onehorse/mpu9250-teensy-3x-add-on-shields/) connected through USB to the computer.

Yo can see it in the images below:

![Setup](https://i.imgur.com/XqFxrWS.png)

## Usage

This library is extremely easy to use. Go to the [releases](https://github.com/juangallostra/AltitudeEstimation/releases) page of the library and download the latest release. Once downloaded, uncompress it in your `Arduino/libraries` folder. Alternatively, clone this respository in your `Arduino/libraries` folder by opening a terminal session there and typing:

`git clone https://github.com/juangallostra/AltitudeEstimation.git`


Clonning/Downloading the library in your `Arduino/libraries` folder is the only step that has to be performed before getting into the actual coding. To be able to use the library in your code add the following line at the beginning of your file:

```cpp
#include "altitude.h"
```

We are ready now to instantiate the estimator where needed. This can be done by:

```cpp
// Altitude estimator
AltitudeEstimator altitude = AltitudeEstimator(0.0005, 	// sigma Accel
                                               0.0005, 	// sigma Gyro
                                               0.018,   // sigma Baro
                                               0.5, 	// ca
                                               0.1);	// accelThreshold
```

Note that here we are specifying the value of the parameters that will be used to perform the estimations. These can be tuned to achieve higher accuracy. A more in-detail description of the parameters is provided below.

Once we have our estimator the only thing we have to do to estimate the current vertical position, velocity and acceleration is to call its `estimate` method. Since one of the aims of this library is to be hardware-independent, this method expects to receive the latest acceleremoter, gyrometer and barometer readings as well as the timestamp they were obtained as parameters. Jump to the **Available methods** section if you wish to know more about these parameters and their expected values.

```cpp
altitude.estimate(accelData, gyroData, baroHeight, timestamp);
```

The call to the previous method will not return anything. Instead, it will perform the estimation and store the results. To access the results of the latest estimation you just have to call the following methods:

```cpp
altitude.getAltitude()	// get estimated altitude in meters
altitude.getVerticalVelocity() // get estimated vertical velocity in meters per second
altitude.getVerticalAcceleration() // get estimated vertical acceleration in m/s^2
```

A fully working [example](https://github.com/juangallostra/AltitudeEstimation/blob/master/examples/AltitudeEstimation/AltitudeEstimation.ino) can be found at `AltitudeEstimation.ino` under `examples/AltitudeEstimation`. The provided code assumes that the hardware used is an MPU9250 IMU and a MS5637 barometer. Arduino libraries for both them are available [here](https://github.com/simondlevy/MPU9250) (MPU9250) and [here](https://github.com/BonaDrone/MS5637) (MS5637).

There is [another example](https://github.com/juangallostra/AltitudeEstimation/blob/master/examples/SENtralAltitude/SENtralAltitude.ino) thanks to [Simon D. Levy](https://github.com/simondlevy), making use of the EM7180 SENtral sensor hub. The sketch is `SENtralAltitude.ino` and can be fund under `examples/SENtralAltitude`. The drivers for the EM7180 can be downloaded from [here](https://github.com/simondlevy/EM7180).


### Available methods

### estimate

Each time this method is called the estimation of the vertical position, velocity and acceleration will be updated. To update the estimation you must provide the latest available readings from the accelerometer (in g-s), the gyrometer (in rad/s) and the baro (in meters) as well as the timestamp in which the readings were obtained. Currently the library assumes the timestamp is measured in microseconds.

Calling this method will update the estimations and store the results internally but it will not return anything. There are specific methods provided to get the estimated values. 

Method signature:

```cpp
void estimate(float accel[3], float gyro[3], float baroHeight, uint32_t timestamp)
```

Parameters:

* `float accel[3]`: length 3 array of floats with acceleration readings in g-s. The order in which the estimator expects the readings is `ax`, `ay` and `az`.

* `float gyro[3]`: length 3 array of floats with gyrometer readings in rad/s. The order in which the estimator expects the readings is `gx`, `gy` and `gz`.

* `float baroHeight`: vertical height in meters as estimated from the barometer signal. The conversion from pressure to height can be easily achieved with a small helper function. [This](https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf) is the formula I used in the provided example to achieve so (see lines 29-33). Since the algorithm requires the altitude estimated from the baro and not the pressure reading itself I prefer to let the user choose freely how he wants to map pressure to altitude.

* `uint32_t timestamp`: The timestamp at which the readings were obtained. Currently the library expects it to be in microseconds.


### getAltitude

This method can be called to obtain the latest vertical position estimation. The estimated altitude is measured in meters.

Method signature:

```cpp
float getAltitude()

```


### getVerticalVelocity

This method can be called to obtain the latest vertical velocity estimation. The estimated velocity is measured in meters per second.

Method signature:

```cpp
float getVerticalVelocity()
```


### getVerticalAcceleration

This method can be called to obtain the latest vertical acceleration estimation. The estimated vertical acceleration is measured in meters per second^2.

Method signature:

```cpp
float getVerticalAcceleration()
```


## Parameter tunning

There are a few parameters that can be tuned to try to achieve higher accuracy. These paremeters are:

1. `sigmaAccel`: standard deviation of the accelerometer
2. `sigmaGyro`: standard deviation of the gyroscope
3. `sigmaBaro`: standard deviation of the barometer
4. `ca`:  constant value for the markov chain acceleration model: a(k) = ca * a(k-1) + e
5. `accelThreshold`: vertical acceleration threshold. If 12 consecutive vertical acceleration values are below the threshold the vertical velocity will be set to 0.

The value of this parameters must be specified when calling the constructor of `AltitudeEstimator`. The order is the one listed above. Below one can see the signature of the constructor:

```cpp
AltitudeEstimator(float sigmaAccel, float sigmaGyro, float sigmaBaro, float ca, float accelThreshold)
```

## Results

Legend:
* **Yellow**: Estimated vertical acceleration (m/s^2).
* **Green**: Estimated vertical velocity (m/s).
* **Blue**: Estimated altitude from barometer pressure readings (m).
* **Red**: Two step Kalman/Complementary filter estimated altitude (m).

## Test 1

Setup at rest the whole time.

![test_1](https://i.imgur.com/peAQHkP.png)

## Test 2

Lift to full cable length, hold, and lower it back to the initial position.

![test_2](https://i.imgur.com/dG4Hc73.png)


## Extras

There is a Python implementation of the algorithm under the folder `extras`. You can read about it [here](https://github.com/juangallostra/AltitudeEstimation/blob/master/extras/README.md).  
