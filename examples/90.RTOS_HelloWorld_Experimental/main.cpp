// ========== Playground for RTOS experiments =============

#include <Arduino.h>
#include "madflight_config.h" //Edit this header file to setup the pins, hardware, radio, etc. for madflight
#include <madflight_rtos.h>

RuntimeTrace runtimeTrace = RuntimeTrace("_loop");

void setup() {
  madflight_rtos_setup(); //setup madflight modules and start IMU, BBX, RCL, CLI, and SENSORS rtos tasks 
}

void loop() {
  runtimeTrace.start();
  //nothing to do here for madflight (the rtos tasks do the module updates)
  runtimeTrace.stop(true);
}

//This is __MAIN__ function of this program. It is called when new IMU data is available.
void imu_loop() {
  //toggle led on every 1000 samples (normally 1 second)
  if(imu.update_cnt % 1000 == 0) led.toggle();

  bbx.log_imu(); //log to SDCARD at full speed

  ahr.update(); //Sensor fusion: update ahr.roll, ahr.pitch, and ahr.yaw angle estimates (degrees) from IMU data 
}
