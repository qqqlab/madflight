/*#########################################################################################################################

Minimal quadcopter demo program for madflight on a M5Stack StampFly v1.0/v1.1 Quadcopter

###########################################################################################################################

See http://madflight.com for detailed description (this example is based on the QuadcopterAdvanced example)

NOTE: This program does not work with the M5Stack Joystick and ESP-NOW, you need to connect a separate CRSF/ELRS/SBUS/DSM/PPM radio receiver

Arming/disarming with dedicated switch

    Arm: Set throttle low, then flip arm switch from DISARMED to ARMED.
    Disarm: Flip arm switch from ARMED to DISARMED, at any throttle position. "Kill switch".

Arming/disarming with sticks (when no arm switch is defined, i.e. cfg.rcl_arm_ch == 0 ) 

    Arm: Pull both sticks toward you, yaw full right, and roll full left
    Disarm: Pull both sticks toward you, yaw full left, and roll full right

LED State                              Meaning
---------                              -------
OFF                                    Not powered
ON (blue)                              Startup (don't move, running gyro calibration)
Blinking long OFF short ON (green)     DISARMED
Blinking long ON short OFF (red)       ARMED
Blink interval longer than 1 second    imu_loop() is taking too much time
Fast blinking                          Something is wrong, connect USB serial for info

MIT license - Copyright (c) 2023-2026 https://madflight.com
##########################################################################################################################*/

/*Configuration of the Grove Ports

Grove Port Pinout:
    pin1 black : GND
    pin2 red   : 5V
    pin3 white : Device_TX -> StampFly_RX
    pin4 yellow: Device_RX <- StampFly_TX

ser_bus 0 is the RED Grove port with 4.7k pullups - DOES NOT WORK with OPENLOG, might work with other devices
ser_bus 1 is the BLACK Grove port without pullups - should work with all serial devices
*/
const char madflight_config[] = R""(

// Default from brd/stampfly.h: CRSF receiver on ser_bus 1 (BLACK) and openlog on ser_bus 0 (RED)

// Use settings below to override 

// Uncomment to change receiver type
//rcl_gizmo      CRSF // Set receiver type: MAVLINK, CRSF, SBUS, DSM, PPM xxx
//rcl_ser_bus    1    // BLACK Grove

// Uncomment this block to use GPS instead of OPENLOG on RED Grove
//bbx_gizmo      NONE 
//bbx_ser_bus    -1   //disable
//gps_gizmo      UBLOX
//gps_ser_bus    0    // RED Grove
//gps_baud       0    // use 0 for auto baud

//flightmode mapping from 6-pos switch to flight mode (simulates a 2-pos switch: RATE/ANGLE)
rcl_flt0 RATE
rcl_flt0 RATE
rcl_flt0 RATE
rcl_flt0 RATE
rcl_flt0 RATE
rcl_flt0 ANGLE

)""; // End of madflight_config

#include <madflight.h>

//prototypes (for PlatformIO, not needed for Arduino IDE)
void led_Blink();
float degreeModulus(float v);
void control_Angle(bool zero_integrators);
void control_Rate(bool zero_integrators);
void out_KillSwitchAndFailsafe();
void out_Mixer();

//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //
//========================================================================================================================//

//IMPORTANT: This is a safety feature which keeps props spinning when armed, and hopefully reminds the pilot to disarm!!! 
const float armed_min_throttle = 0.03; //Minimum throttle when armed, set to a value between ~0.10 and ~0.25 which keeps the props spinning at minimum speed.

//Controller parameters
const float maxRoll        = 30.0;   //Max roll angle in deg for angle mode - DO NOT INCREASE OVER 70 OR YOU WILL CRASH DUE TO GIMBAL-LOCKS
const float maxPitch       = 30.0;   //Max pitch angle in deg for angle mode - DO NOT INCREASE OVER 70 OR YOU WILL CRASH DUE TO GIMBAL-LOCKS
const float maxRollRate    = 60.0;   //Max roll rate in deg/sec for rate mode 
const float maxPitchRate   = 60.0;   //Max pitch rate in deg/sec for rate mode
const float maxYawRate     = 160.0;  //Max yaw rate in deg/sec for angle and rate mode
const float i_limit        = 25.0;   //Integrator saturation level, mostly for safety (default 25.0)

//PID Angle Mode 
const float Kp_ro_pi_angle = 0.4;    //Roll/Pitch P-gain
const float Ki_ro_pi_angle = 0.1;    //Roll/Pitch I-gain
const float Kd_ro_pi_angle = 0.05;   //Roll/Pitch D-gain
const float Kp_yaw_angle   = 0.6;    //Yaw P-gain
const float Kd_yaw_angle   = 0.1;    //Yaw D-gain

//PID Rate Mode 
const float Kp_ro_pi_rate  = 0.15;   //Roll/Pitch rate P-gain
const float Ki_ro_pi_rate  = 0.2;    //Roll/Pitch rate I-gain
const float Kd_ro_pi_rate  = 0.0002; //Roll/Pitch rate D-gain (be careful when increasing too high, motors will begin to overheat!)
const float Kp_yaw_rate    = 0.3;    //Yaw rate P-gain
const float Ki_yaw_rate    = 0.05;   //Yaw rate I-gain
const float Kd_yaw_rate    = 0.00015;//Yaw rate D-gain (be careful when increasing too high, motors will begin to overheat!)

//Yaw to keep in ANGLE mode when yaw stick is centered
float yaw_desired = 0;

//Motor Setup

//Define motor outputs, for example 6 means use the GPIO pin defined with the `out6_pin` parameter
const int motor_outputs[4] = {0, 1, 2, 3}; //right-rear, right-front, left-rear, left-front

void setup_motors() {
  bool success = out.setup_brushed    (4, motor_outputs, 5000);           // Brushed motors: 5000Hz with 0-100% duty cycle

  out.print(); //print motor configuration
  if(!success) madflight_panic("Motor init failed.");
}

//========================================================================================================================//
//                                                       SETUP()                                                          //
//========================================================================================================================//

void setup() {
  // Setup madflight modules, start madflight RTOS tasks, Serial.begin(11520)
  madflight_setup();

  setup_motors();

  // Set initial desired yaw
  yaw_desired = ahr.yaw;

  Serial.println("Setup completed, CLI started - Type 'help' for help, or 'diff' to debug");
}

//========================================================================================================================//
//                                                            LOOP()                                                      //
//========================================================================================================================//

void loop() {
  // Nothing to do here for madflight, you can add your code here.
  delay(1000); //this delay() prevents empty loop wasting processor time, give this time to other tasks
}

//========================================================================================================================//
//                                                   IMU UPDATE LOOP                                                      //
//========================================================================================================================//

//This is __MAIN__ function of this program. It is called when new IMU data is available.
void imu_loop() {
  // Blink LED
  led_Blink();

  // Sensor fusion: update ahr.roll, ahr.pitch, and ahr.yaw angle estimates (degrees) from IMU data
  ahr.update(); 

  //PID Controller
  pid.controller();

  //Updates out.arm, the output armed flag
  out_KillSwitchAndFailsafe(); //Cut all motor outputs if DISARMED or failsafe triggered.

  //Actuator mixing
  out_Mixer(); //Mixes PID outputs and sends command pulses to the motors, if mot.arm == true
}

//========================================================================================================================
//                      IMU UPDATE LOOP FUNCTIONS - in same order as they are called from imu_loop()
//========================================================================================================================

void led_Blink() {
  //Blink LED once per second, if LED blinks slower then the loop takes too much time, use CLI 'pimu' to investigate.
  //DISARMED: green long off, short on, ARMED: red long on, short off
  uint32_t modulus = imu.update_cnt % imu.getSampleRate();
  if( modulus == 0) led.color( (out.armed() ? 0 : 0x00ff00) ); //start of pulse - armed: off, disarmed: green
  if( modulus == imu.getSampleRate() / 10)  led.color( (out.armed() ? 0xff0000 : 0) ); //end of pulse - armed: red, disarmed: off
}

void out_KillSwitchAndFailsafe() {
  //Change to ARMED when rcl is armed (by switch or stick command)
  if (!out.armed() && rcl.armed) {
    out.set_armed(true);
    Serial.println("OUT: ARMED");
    bbx.start(); //start blackbox logging
  }

  //Change to DISARMED when rcl is disarmed, or if radio lost connection
  if (out.armed() && (!rcl.armed || !rcl.connected())) {
    out.set_armed(false);
    if(!rcl.armed) {
      Serial.println("OUT: DISARMED");
      bbx.stop(); //stop blackbox logging
    }else{
      Serial.println("OUT: DISARMED due to lost radio connection");
      //keep on logging to document the crash...
    }
  }
}

void out_Mixer() {
  //DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
  /*
   * Takes pid.roll, pid.pitch, and pid.yaw computed from the PID controller and appropriately mixes them for the desired
   * vehicle configuration. For example on a quadcopter, the left two motors should have +pid.roll while the right two motors
   * should have -pid.roll. Front two should have +pid.pitch and the back two should have -pid.pitch etc... every motor has
   * normalized (0 to 1) rcl.throttle command for throttle control. Can also apply direct unstabilized commands from the transmitter with 
   * rcl.xxx variables are to be sent to the motor ESCs and servos.
   * 
   *Relevant variables:
   *rcl.throtle - direct thottle control
   *pid.roll, pid.pitch, pid.yaw - stabilized axis variables
   *rcl.roll, rcl.pitch, rcl.yaw - direct unstabilized command passthrough
   */
/*
Motor order diagram (Betaflight order)

      front
 CW -->   <-- CCW
     4     2 
      \ ^ /
       |X|
      / - \
     3     1 
CCW -->   <-- CW

                                        M1234
Pitch up (stick back)   (front+ back-)   -+-+
Roll right              (left+ right-)   --++
Yaw right               (CCW+ CW-)       -++-
*/

  // IMPORTANT: This is a safety feature to remind the pilot to disarm.
  // Set motor outputs to at least armed_min_throttle, to keep at least one prop spinning when armed. The [out] module will disable motors when out.armed() == false
  float thr = armed_min_throttle + (1 - armed_min_throttle) * rcl.throttle; //shift motor throttle range from [0.0 .. 1.0] to [armed_min_throttle .. 1.0]

  if(rcl.throttle == 0) {
    //if throttle idle, then run props at low speed without applying PID. This allows for stick commands for arm/disarm.
    out.set_output(motor_outputs[0], thr);
    out.set_output(motor_outputs[1], thr);
    out.set_output(motor_outputs[2], thr);
    out.set_output(motor_outputs[3], thr);
  }else{
    // Quad mixing
    out.set_output(motor_outputs[0], thr - pid.pitch.sum - pid.roll.sum - pid.yaw.sum); //M1 Back Right CW
    out.set_output(motor_outputs[1], thr + pid.pitch.sum - pid.roll.sum + pid.yaw.sum); //M2 Front Right CCW
    out.set_output(motor_outputs[2], thr - pid.pitch.sum + pid.roll.sum + pid.yaw.sum); //M3 Back Left CCW
    out.set_output(motor_outputs[3], thr + pid.pitch.sum + pid.roll.sum - pid.yaw.sum); //M4 Front Left CW
  }
}
