# madflight headers

- madflight.h - Default header to use in your projects, defines madflight_setup()
- madflight_rtos.h - madflight.h extended with RTOS, defines madflight_rtos_setup()
- madflight_modules.h - All modules without any setup

# madflight modules

- ahr - Attitude and heading reference system (AHRS)
- alt - Altitude estimators
- bar - Barometer sensors
- bat - Battery sensors (current, voltage, power, capacity)
- bbx - Blackbox logging to SDCARD
- brd - Board definitions
- cfg - Configuration from EEPROM, string
- cli - Command line interface
- gps - Global positioning system (GPS)
- hal - Hardware abstraction layer (HAL)
- imu - Inertial measurement unit (IMU)
- led - Light emmitting diode (LED)
- lua - Lua scripting
- mag - Magnetometer sensors
- nav - Waypoint navigation
- ofl - Optical flow sensors
- out - Output drivers (motors, servos, etc.)
- pid - Proportional–integral–derivative controller
- rcl - Remote control radio link
- rdr - Radar, lidar and ultrasonic distance sensors
- tbx - Toolbox (common functions)
- veh - Vehicle state info
