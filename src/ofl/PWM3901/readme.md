# Arduino driver for PMW3901 optical flow sensor

Arduino driver for the Pixart PMW3901 optical flow sensor. The driver
is developed to support the [Bitcraze Flow Breakout board](https://wiki.bitcraze.io/breakout:flow). It communicates with
the sensor using SPI.

## Electrical connection

The library is using the standard SPI library, the sensor MOSI, MISO and SCK should be connected to the arduino according to the [SPI library documentation](https://www.arduino.cc/en/Reference/SPI). The CS (chip select) pin can be any digital pin (which allows to connect more than one SPI sensor to your Arduino).

## Usage

Look at the [flow](examples/flow/flow.ino) example for basic usage.

You can create a sensor by passing the chip select pin number:

``` C++
// Using digital pin 10 for chip select
Bitcraze_PMW3901 flow(10);
```

Initializing the sensor is done by calling *begin*, it returns *true* if the sensor is detected and initialized, *false* otherwise:

``` C++
  if (!flow.begin()) {
    Serial.println("Initialization of the flow sensor failed");
    while(1) { }
  }
```

As soon as it is initialized, the sensor will start accumulating motion count. Calling the *readMotionCount* function allows to get the value of the motion counters and resets the counters. So each time the function is called you get the motion count since the last call:

``` C++
  // Get motion count since last call
  flow.readMotionCount(&deltaX, &deltaY);
```

Alternatively, to use the framebuffer system to use it as a camera of sorts you will need to do a bit more. After the *begin* function, call the *enableFrameBuffer* function to switch modes, then in your loop call *readFrameBuffer* while passing it a 35*35 array to dump pixels into. 

``` C++
  // Get pixels from framebuffer into an array
  flow.readFrameBuffer(int *FBuffer);
```


## Output interpretation

The sensor outputs unitless ticks that corresponds to the amount of movement that has been observed in front of the sensor. These ticks can be converted to a distance if we know the distance to the tracked surface and we assume the tracked surface is flat.

Parameters of the PMW3901 sensor have been devised experimentally in order to implement it in the Crazyflie firmware. The measurement equation are described in  [Marcus Greiff's Master thesis](http://lup.lub.lu.se/luur/download?func=downloadFile&recordOId=8905295&fileOId=8905299) section 6.5. The implementation is part or the [Crazyflie kalman filter](https://github.com/bitcraze/crazyflie-firmware/blob/6308ff47ff4d4691f9b7f6f991564244c76d7910/src/modules/src/estimator_kalman.c#L1034-L1098).

## Contribute
Go to the [contribute page](https://www.bitcraze.io/contribute/) on our website to learn more.

