/*#########################################################################################################################

Quadcopter demo program for madflight Arduino ESP32-S3 / ESP32 / RP2350 / RP2040 / STM32 Flight Controller

###########################################################################################################################

See http://madflight.com for setup instructions and modify the USER-SPECIFIED VARIABLES section below

Required Hardware

    IMU sensor (SPI or I2C)
    RC receiver with 6 channels (AETR + arm switch + flightmode switch)
    4 brushless/brushed motors with ESCs

Connecting Hardware

    SPI IMU: connect pin_imu_int, pin_imu_cs, pin_spi0_miso, pin_spi0_mosi, pin_spi0_sclk
    or for I2C IMU: connect pin_imu_int, pin_i2c1_scl, pin_i2c1_sda
    RC receiver: connect pin_ser0_rx to receiver TX pin
    ESCs: pin_out0 ... pin_out3 to the ESC inputs of motor1 ... motor4

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

MIT license - Copyright (c) 2026 https://madflight.com
##########################################################################################################################*/

#include "config_quad.h" //Edit this header file to setup the pins, hardware, radio, etc. for madflight
#include <madflight.h>

//prototypes (for PlatformIO, not needed for Arduino IDE)
void led_Blink();
void out_KillSwitchAndFailsafe();
void out_Mixer();

//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //
//========================================================================================================================//

//IMPORTANT: This is a safety feature which keeps props spinning when armed, and hopefully reminds the pilot to disarm!!! 
const float armed_min_throttle = 0.20; //Minimum throttle when armed, set to a value between ~0.10 and ~0.25 which keeps the props spinning at minimum speed.

//Motor Setup

//Define motor outputs, for example 6 means use the GPIO pin defined with the `out6_pin` parameter
const int motor_outputs[4] = {0, 1, 2, 3}; //right-rear, right-front, left-rear, left-front

void setup_motors() {
  // Uncomment ONE line - select output type
  bool success = out.setup_motors     (4, motor_outputs, 400, 950, 2000); // Standard PWM: 400Hz, 950-2000 us
  //bool success = out.setup_motors     (4, motor_outputs, 2000, 125, 250); // Oneshot125: 2000Hz, 125-250 us
  //bool success = out.setup_dshot      (4, motor_outputs, 300);            // Dshot300
  //bool success = out.setup_dshot_bidir(4, motor_outputs, 300);            // Dshot300 Bi-Directional
  //bool success = out.setup_brushed    (4, motor_outputs, 5000);           // Brushed motors: 5000Hz with 0-100% duty cycle

  out.print(); //print motor configuration
  if(!success) madflight_panic("Motor init failed.");
}

//========================================================================================================================//
//                                                       SETUP()                                                          //
//========================================================================================================================//

void setup() {
  // Setup madflight modules, start madflight RTOS tasks, Serial.begin(11520)
  madflight_setup();

  // STOP if imu is not installed
  if(!imu.installed()) madflight_panic("This program needs an IMU.");

  setup_motors();

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

// This is the __MAIN__ part of this program. It is called from the IMU FreeRTOS task when new IMU data is available.
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
    M4     M2 
      \ ^ /
       |X|
      / - \
    M3     M1 
CCW -->   <-- CW

Mixer:

Roll right            ==> +left_motors. -right_motors ==> -m1 -m2 +m3 +m4 ==> 1st column of mix[][]
Pitch up (stick back) ==> +front motors, -back motors ==> -m1 +m2 -m3 +m4 ==> 2nd column of mix[][]
Yaw right             ==> +CCW motors, -CW motors     ==> -m1 +m2 +m3 -m4 ==> 3rd column of mix[][]
*/
  // Quad mix - the mix matrix maps PID [roll,pitch,yaw,throttle] to outputs [motor1,2,3,4]
  float const mix[4][4] = {
    {-1, -1, -1, +1},
    {-1, +1, +1, +1},
    {+1, -1, +1, +1},
    {+1, +1, -1, +1}
  };

  /* Example mix for a Quad with motor rotation direction reversed, i.e. props rotate outwards at front of drone -> mix with 3rd column (yaw) reversed
  float const mix[4][4] = {
    {-1, -1, +1, +1},
    {-1, +1, -1, +1},
    {+1, -1, -1, +1},
    {+1, +1, +1, +1}
  }; */

  // IMPORTANT: This is a safety feature to remind the pilot to disarm.
  // Set motor outputs to at least armed_min_throttle, to keep at least one prop spinning when armed. The [out] module will disable motors when out.armed() == false
  float thr_in = constrain(pid.throttle.sum, 0, 1);
  float thr = armed_min_throttle + (1 - armed_min_throttle) * thr_in; //shift motor throttle range from [0.0 .. 1.0] to [armed_min_throttle .. 1.0]
  thr = constrain(thr, armed_min_throttle, 1);

  if(rcl.throttle == 0) {
    //if throttle idle, then run props at low speed without applying PID. This allows for stick commands for arm/disarm.
    out.set_output(motor_outputs[0], thr);
    out.set_output(motor_outputs[1], thr);
    out.set_output(motor_outputs[2], thr);
    out.set_output(motor_outputs[3], thr);
  }else{
    // Apply mixer
    float mot1_mix = mix[0][0] * pid.roll.sum + mix[0][1] * pid.pitch.sum + mix[0][2] * pid.yaw.sum + mix[0][3] * thr; // nominal range [-1 to 1]
    float mot2_mix = mix[1][0] * pid.roll.sum + mix[1][1] * pid.pitch.sum + mix[1][2] * pid.yaw.sum + mix[1][3] * thr; // nominal range [-1 to 1]
    float mot3_mix = mix[2][0] * pid.roll.sum + mix[2][1] * pid.pitch.sum + mix[2][2] * pid.yaw.sum + mix[2][3] * thr; // nominal range [-1 to 1]
    float mot4_mix = mix[3][0] * pid.roll.sum + mix[3][1] * pid.pitch.sum + mix[3][2] * pid.yaw.sum + mix[3][3] * thr; // nominal range [0 to 1]

    out.set_output(motor_outputs[0], mot1_mix); //M1 Back Right CW
    out.set_output(motor_outputs[1], mot2_mix); //M2 Front Right CCW
    out.set_output(motor_outputs[2], mot3_mix); //M3 Back Left CCW
    out.set_output(motor_outputs[3], mot4_mix); //M4 Front Left CW
  }
}
