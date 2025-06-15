# GPS-uBlox - Platform agnostic C++ uBlox GPS driver

Ported from tried and tested Ardupilot GPS UBLOX driver.

Features:

- Platform agnostic C++ (Arduino, PlatformIO, Linux, Windows, whatever...)
- Supports uBlox-6 to uBlox-M10 GPS
- Automatically configures baudrate
- Uses high speed ubx protocol
- Auto reconnect on communication loss
- Optionally save the config to the GPS module for faster restarts

Changes from ArduPilot:

- Moved all external dependencies into AP_GPS_UBLOX.h & .cpp
- Added pure virtual I_xxx methods as interface to the outside world
- Removed float (except LOGGING)

Requirements:

See example directory. Basically, implement the 7 pure virtual methods to get going.