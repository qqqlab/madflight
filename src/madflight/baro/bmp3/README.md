[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

![Bolder Flight Systems Logo](img/logo-words_75.png) &nbsp; &nbsp; ![Arduino Logo](img/arduino_logo_75.png)

# Bmp3
This library communicates with the BMP3xy series of pressure sensors, such as the [BMP388](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp388/) and is compatible with Arduino and CMake build systems.
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)
   * [Contributing guide](CONTRIBUTING.md)

# Usage
This library supports both I2C and SPI commmunication with the BMP3xy sensors and supports collecting pressure and temperature data. The BMP3xy temperature data should be treated as die temperature, and is labled as such within this library.

# Installation

## Arduino
Simply clone or download this library into your Arduino/libraries folder. The library is added as:

```C++
#include "bmp3.h"
```

Example Arduino executables are located in: *examples/arduino/*. Teensy 3.x, 4.x, and LC devices are used for testing under Arduino and this library should be compatible with other Arduino devices.

## CMake
CMake is used to build this library, which is exported as a library target called *bmp3*. The header is added as:

```C++
#include "bmp3.h"
```

The library can be also be compiled stand-alone using the CMake idiom of creating a *build* directory and then, from within that directory issuing:

```
cmake .. -DMCU=MK66FX1M0
make
```

This will build the library and example executables called *i2c_example* and *spi_example*. The example executable source files are located at *examples/cmake/i2c.cc* and *examples/cmake/spi.cc*. Notice that the *cmake* command includes a define specifying the microcontroller the code is being compiled for. This is required to correctly configure the code, CPU frequency, and compile/linker options. The available MCUs are:
   * MK20DX128
   * MK20DX256
   * MK64FX512
   * MK66FX1M0
   * MKL26Z64
   * IMXRT1062_T40
   * IMXRT1062_T41
   * IMXRT1062_MMOD

These are known to work with the same packages used in Teensy products. Also switching packages is known to work well, as long as it's only a package change.

The example targets create executables for communicating with the sensor using I2C or SPI communication. Each target also has a *_hex*, for creating the hex file to upload to the microcontroller, and an *_upload* for using the [Teensy CLI Uploader](https://www.pjrc.com/teensy/loader_cli.html) to flash the Teensy. Please note that instructions for setting up your build environment can be found in our [build-tools repo](https://github.com/bolderflight/build-tools).

# Namespace
This library is within the namespace *bfs*.

# Bmp3

## Methods

**Bmp3()** Default constructor, requires calling the Config method to setup the I2C or SPI bus and I2C address or SPI chip select pin.

**Bmp3(TwoWire &ast;i2c, const I2cAddr addr)** Creates a Bmp3 object. This constructor is used for the I2C communication interface. A pointer to the I2C bus object is passed along with an enum of the I2C address of the sensor. Use I2C_ADDR_PRIM if the SDO pin is grounded and I2C_ADDR_SEC if the SDO pin is pulled high.

```C++
bfs::Bmp3 bmp(&Wire, bfs::Bmp3::I2C_ADDR_PRIM);
```

**Bmp3(SPIClass *spi, const uint8_t cs)** Creates a Bmp3 object. This constructor is used for the SPI communication interface. A pointer to the SPI bus object is passed along with the chip select pin of the sensor. Any pin capable of digital I/O can be used as a chip select pin.

```C++
bfs::Bmp3 bmp(&SPI, 2);
```

**void Config(TwoWire &ast;bus, const I2cAddr addr)** This is required when using the default constructor and sets up the I2C bus and I2C address.

**void Config(SPIClass &ast;spi, const uint8_t cs)** This is required when using the default constructor and sets up the SPI bus and chip select pin.

**int8_t error_code()** The *Begin*, *ConfigOsMode*, *ConfigFilterCoef*, and *Read* methods return boolean values, true if the method is successful (and in the case of *Read*, if new data is available) and false otherwise. This method returns the error code from the last operation, which can help in debugging. The error codes are:

| Error Code | Value |
| --- | --- |
| BMP3_OK | 0 |
| BMP3_E_NULL_PTR | -1 |
| BMP3_E_COMM_FAIL | -2 |
| BMP3_E_INVALID_ODR_OSR_SETTINGS | -3 |
| BMP3_E_CMD_EXEC_FAILED | -4 |
| BMP3_E_CONFIGURATION_ERR | -5 |
| BMP3_E_INVALID_LEN | -6 |
| BMP3_E_DEV_NOT_FOUND | -7 |
| BMP3_E_FIFO_WATERMARK_NOT_REACHED | -8 |

**bool Begin()** Initializes communication with the sensor and configures the default sampling rates, oversampling and low pass filter settings. True is returned if communication is able to be established with the sensor and configuration completes successfully, otherwise, false is returned. The communication bus is not initialized within this library and must be initialized seperately; this enhances compatibility with other sensors that may on the same bus.

```C++
Wire.begin();
Wire.setClock(400000);
bool status = bmp.Begin();
if (!status) {
  // ERROR
}
```

The BMP3xy features programmable oversampling, filtering, and output data rate. By default, these are set to the following values:

| Settings | |
| --- | --- |
| Oversampling | pressure x 8, temperature x 1 |
| IIR Filter Coefficient | 2 |
| Output Data Rate | 50 Hz |

**bool ConfigOsMode(const OsMode mode)** Configures the pressure oversampling, temperature oversampling, and output data rate:

| Enum | Pressure Oversampling | Temperature Oversampling | Output Data Rate |
| --- | --- | --- | --- |
| OS_MODE_PRES_1X_TEMP_1X | 1x | 1x | 200 Hz |
| OS_MODE_PRES_2X_TEMP_1X | 2x | 1x | 100 Hz |
| OS_MODE_PRES_4X_TEMP_1X | 4x | 1x | 50 Hz |
| OS_MODE_PRES_8X_TEMP_1X | 8x | 1x | 50 Hz |
| OS_MODE_PRES_16X_TEMP_2X | 16x | 2x | 25 Hz |
| OS_MODE_PRES_32X_TEMP_2X | 32x | 2x | 12.5 Hz |

```C++
bool status = bmp.ConfigOsMode(bfs::Bmp3::OS_MODE_PRES_32X_TEMP_2X);
if (!status) {
  // ERROR
}
```

**OsMode os_mode()** Returns the current oversampling mode.

**bool ConfigFilterCoef(const FilterCoef val)** Sets the digital low pass filter IIR coefficient. This filter is applied to all measurements. The filter is given by the following equation:

*data_filtered = (data_filtered_old &ast; (filter_coefficient - 1) + data) / filter_coefficient*

The following enumerated filter coefficients are supported:

| Filter Coefficient Name     | Filter Coefficient Value     |
| ---                         | ---                          |
| FILTER_COEF_OFF             | 1                            |
| FILTER_COEF_2               | 2                            |
| FILTER_COEF_4               | 4                            |
| FILTER_COEF_8               | 8                            |
| FILTER_COEF_16              | 16                           |
| FILTER_COEF_32              | 32                           |
| FILTER_COEF_64              | 64                           |
| FILTER_COEF_128             | 128                          |

```C++
bool status = bmp.ConfigFilterCoef(bfs::Bmp3::FILTER_COEF_16);
if (!status) {
  // ERROR
}
```

**FilterCoef filter_coef()** Returns the current filter coefficient value.

**bool Reset()** Performs a soft reset of the BMP3xy. True is returned on success.

**bool Read()** Reads data from the BMP3xy and stores the data in the Bmp3 object. Returns true if data is successfully read, otherwise, returns false.

```C++
/* Read the sensor data */
if (bmp.Read()) {
}
```

**float pres_pa()** Returns the pressure data from the Bmp3 object in units of Pa.

```C++
float pressure = bmp.pres_pa();
```

**float die_temp_c** Returns the die temperature of the sensor from the Bmp3 object in units of degrees C.

```C++
float temperature = bmp.die_temp_c();
```

## Example List
* **i2c**: demonstrates declaring a *Bmp3* object, initializing the sensor, and collecting data. I2C is used to communicate with the BMP3xy sensor.
* **spi**: demonstrates declaring a *Bmp3* object, initializing the sensor, and collecting data. SPI is used to communicate with the BMP3xy sensor.
