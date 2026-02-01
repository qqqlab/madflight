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
}

void loop() {
  // Optional runtime tracing - type 'ps' in CLI to see results
  static RuntimeTrace runtimeTrace = RuntimeTrace("_loop");
  runtimeTrace.start();
  // Add your code here (Nothing to do here for madflight, the rtos tasks do the module updates)
  runtimeTrace.stop(true);
}

// This function is called from the IMU task when fresh IMU data is available.
void imu_loop() {
  // Toggle led on every 1000 samples (E.g. 1 second peroid at 1000Hz sample rate)
  if(imu.update_cnt % 1000 == 0) led.toggle();

  // Log IMU data to SDCARD at full speed - type 'bbstart' in CLI to start logging.
  bbx.log_imu();

  // AHRS sensor fusion -  type 'pahr' in CLI to see results
  ahr.update();
}
