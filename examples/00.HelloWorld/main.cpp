/*#########################################################################################################################

"Hello World" for madflight library

Upload, connect Serial Monitor at 115200 baud and send 'help' to see available commands

See http://madflight.com for detailed description

MIT license - Copyright (c) 2023-2026 https://madflight.com
##########################################################################################################################*/

#include "madflight_config.h" // Edit this header file to setup the pins, hardware, radio, etc. for madflight
#include <madflight.h> // Include the library, do this after madflight_config 

void setup() {
  // Setup madflight modules and start IMU, BBX, RCL, CLI, and SENSORS rtos tasks
  madflight_setup();
  Serial.println("Setup completed, CLI started - Type 'help' for help, or 'diff' to debug");
}

void loop() {
  // Nothing to do here for madflight, delay() yields to Idle Task for clearer CPU usage statistics
  delay(10);
}

// This function is called from the IMU task when fresh IMU data is available.
void imu_loop() {
  // Toggle led on every 1000 samples (E.g. 1 second peroid at 1000Hz sample rate)
  if(imu.update_cnt % 1000 == 0) led.toggle();

  // AHRS sensor fusion -  type 'pahr' in CLI to see results
  ahr.update();
}
