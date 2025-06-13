/*#########################################################################################################################

"Hello World" for madflight library

Upload, connect Serial Monitor at 115200 baud and send 'help' to see available commands

See http://madflight.com for detailed description

MIT license
Copyright (c) 2023-2025 https://madflight.com
##########################################################################################################################*/
#include <Arduino.h>

#include "madflight_config.h" //Edit this header file to setup the pins, hardware, radio, etc. for madflight
#include <madflight.h>

void setup() {
  //setup madflight components: Serial.begin(115200), imu, rcin, led, etc. See src/madflight/interface.h for full interface description of each component. 
  madflight_setup();
}

void loop() {
  rcl.update(); // get rc radio commands
  bar.update(); // barometer
  mag.update(); // magnetometer
  gps.update(); // gps
  bat.update(); // battery consumption
  rdr.update(); // radar
  cli.update(); // process CLI commands
}

//This is __MAIN__ function of this program. It is called when new IMU data is available.
void imu_loop() {
  //toggle led on every 1000 samples (normally 1 second)
  if(imu.update_cnt % 1000 == 0) led.toggle();

  ahr.update(); //Sensor fusion: update ahr.roll, ahr.pitch, and ahr.yaw angle estimates (degrees) from IMU data 
}
