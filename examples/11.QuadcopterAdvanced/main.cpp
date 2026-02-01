/*#########################################################################################################################

NOTICE: First get Quadcopter.ino to fly before attempting this program. 

You can copy madflight_config.h from Quadcopter.ino to keep your settings.

###########################################################################################################################

See http://madflight.com for detailed description

Required Hardware

    IMU sensor (SPI or I2C)
    RC receiver with 6 channels (CRSF/ELRS preferred)
    4 brushless motors with ESCs

Connecting Hardware

    SPI IMU: connect pin_imu_int, pin_imu_cs, pin_spi0_miso, pin_spi0_mosi, pin_spi0_sclk
    or for I2C IMU: connect pin_imu_int, pin_i2c1_scl, pin_i2c1_sda
    RC receiver: connect pin_ser0_rx to receiver TX pin, and pin_ser0_tx to receiver RX pin
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

MIT license
MIT license - Copyright (c) 2023-2026 https://madflight.com
##########################################################################################################################*/

void setup() {}
void loop() {}
#if 0

//Vehicle specific madflight configuration
#define VEH_TYPE VEH_TYPE_COPTER //set the vehicle type for logging and mavlink
#define VEH_FLIGHTMODE_AP_IDS {AP_COPTER_FLIGHTMODE_ACRO, AP_COPTER_FLIGHTMODE_STABILIZE} //mapping of fightmode index to ArduPilot code for logging and mavlink
#define VEH_FLIGHTMODE_NAMES {"RATE", "ANGLE"} //fightmode names for telemetry
enum flightmode_enum { RATE, ANGLE };  //the available flightmode indexes
flightmode_enum rcl_to_flightmode_map[6] {RATE, RATE, RATE, RATE, ANGLE, ANGLE}; //flightmode mapping from 2/3/6 pos switch to flight mode (simulates a 2-pos switch: RATE/ANGLE)

#include "madflight_config.h" //Edit this header file to setup the pins, hardware, radio, etc. for madflight
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
const float armed_min_throttle = 0.20; //Minimum throttle when armed, set to a value between ~0.10 and ~0.25 which keeps the props spinning at minimum speed.

//Controller parameters (take note of defaults before modifying!): 
const float i_limit        = 25.0;      //Integrator saturation level, mostly for safety (default 25.0)
const float maxRoll        = 30.0;      //Max roll angle in degrees for angle mode (maximum ~70 degrees)
const float maxPitch       = 30.0;      //Max pitch angle in degrees for angle mode (maximum ~70 degrees)
const float maxRollRate    = 60.0;      //Max roll rate in deg/sec for rate mode 
const float maxPitchRate   = 60.0;      //Max pitch rate in deg/sec for rate mode
const float maxYawRate     = 160.0;     //Max yaw rate in deg/sec for angle and rate mode

//PID Angle Mode 
const float Kp_ro_pi_angle  = 0.2;      //Roll/Pitch P-gain
const float Ki_ro_pi_angle  = 0.1;      //Roll/Pitch I-gain
const float Kd_ro_pi_angle  = 0.05;     //Roll/Pitch D-gain
const float Kp_yaw_angle    = 0.6;      //Yaw P-gain
const float Kd_yaw_angle    = 0.1;      //Yaw D-gain

//PID Rate Mode 
const float Kp_ro_pi_rate   = 0.15;     //Roll/Pitch rate P-gain
const float Ki_ro_pi_rate   = 0.2;      //Roll/Pitch rate I-gain
const float Kd_ro_pi_rate   = 0.0002;   //Roll/Pitch rate D-gain (be careful when increasing too high, motors will begin to overheat!)
const float Kp_yaw_rate     = 0.3;       //Yaw rate P-gain
const float Ki_yaw_rate     = 0.05;      //Yaw rate I-gain
const float Kd_yaw_rate     = 0.00015;   //Yaw rate D-gain (be careful when increasing too high, motors will begin to overheat!)

//Yaw to keep in ANGLE mode when yaw stick is centered
float yaw_desired = 0;

//========================================================================================================================//
//                                                       SETUP()                                                          //
//========================================================================================================================//

void setup() {
  // Setup madflight modules: cli Serial.begin(115200), imu, radio link, led, etc.
  madflight_setup();

  // STOP if imu is not installed
  if(!imu.installed()) madflight_panic("This program needs an IMU.");

  // Setup 4 motors for the quadcopter
  int motor_idxs[] = {0, 1, 2, 3}; //motor indexes
  int motor_pins[] = {cfg.pin_out0, cfg.pin_out1, cfg.pin_out2, cfg.pin_out3}; //motor pins

  // Uncomment ONE line - select output type
  bool success = out.setupMotors(4, motor_idxs, motor_pins, 400, 950, 2000);   // Standard PWM: 400Hz, 950-2000 us
  //bool success = out.setupMotors(4, motor_idxs, motor_pins, 2000, 125, 250); // Oneshot125: 2000Hz, 125-250 us
  //bool success = out.setupDshot(4, motor_idxs, motor_pins, 300);             // Dshot300
  //bool success = out.setupDshotBidir(4, motor_idxs, motor_pins, 300);        // Dshot300 Bi-Directional
  //bool success = out.setupMotors(4, motor_idxs, motor_pins, 5000, 0, 1000000/5000);   // Brushed motors: 5000Hz PWM frequency with 0-100% duty cycle (duty cycle is 0 to 1000000/5000 = 200 us)
  if(!success) madflight_panic("Motor init failed.");

  // Set initial desired yaw
  yaw_desired = ahr.yaw;
}

//========================================================================================================================//
//                                                            LOOP()                                                      //
//========================================================================================================================//

void loop() {
  //update battery, and log if battery was updated.
  if(bat.update()) { 
    bbx.log_bat();
  } 
  
  alt.updateAccelUp(ahr.getAccelUp(), ahr.ts); //NOTE: do this here and not in imu_loop() because `alt` object is not thread safe. - Update altitude estimator with current earth-frame up acceleration measurement
  
  if(bar.update()) {
    alt.updateBarAlt(bar.alt, bar.ts); //update altitude estimator with current altitude measurement
    bbx.log_bar(); //log if pressure updated
  }

  mag.update();
  
  //update gps (and log GPS and ATT for plot.ardupilot.org visualization)
  if(gps.update()) {
    bbx.log_gps(); 
    bbx.log_att();
  } 

  //logging
  static uint32_t log_ts = 0;
  if(millis() - log_ts > 100) {
    log_ts = millis();
    bbx.log_sys();
  }

  cli.update(); //process CLI commands
}

//========================================================================================================================//
//                                                   IMU UPDATE LOOP                                                      //
//========================================================================================================================//

//This is the __MAIN__ part of this program. It is called when new IMU data is available, and runs as high priority FreeRTOS task.
void imu_loop() {
  //Blink LED
  led_Blink();

  //Sensor fusion: update ahr.roll, ahr.pitch, and ahr.yaw angle estimates (degrees) from IMU data
  ahr.update(); 

  //Get radio commands - Note: don't do this in loop() because loop() is a lower priority task than imu_loop(), so in worst case loop() will not get any processor time.
  rcl.update();
  if(rcl.connected() && veh.setFlightmode( rcl_to_flightmode_map[rcl.flightmode] )) { //map rcl.flightmode (0 to 5) to vehicle flightmode
    Serial.printf("Flightmode:%s\n",veh.flightmode_name());
  }

  //PID Controller
  switch( veh.getFlightmode() ) {
    case ANGLE: 
      control_Angle(rcl.throttle == 0); //Stabilize on pitch/roll angle setpoint, stabilize yaw on rate setpoint
      break;
    default: //RATE 
      control_Rate(rcl.throttle == 0); //Stabilize on rate setpoint
  }

  //Updates out.arm, the output armed flag
  out_KillSwitchAndFailsafe(); //Cut all motor outputs if DISARMED or failsafe triggered.

  //Actuator mixing
  out_Mixer(); //Mixes PID outputs and sends command pulses to the motors, if mot.arm == true

  //bbx.log_imu(); //uncomment for full speed black box logging of IMU data, but memory will fill up quickly...
}

//========================================================================================================================
//                      IMU UPDATE LOOP FUNCTIONS - in same order as they are called from imu_loop()
//========================================================================================================================

void led_Blink() {
  //Blink LED once per second, if LED blinks slower then the loop takes too much time, use CLI 'pimu' to investigate.
  //DISARMED: green long off, short on, ARMED: red long on, short off
  uint32_t modulus = imu.update_cnt % imu.getSampleRate();
  if( modulus == 0) led.color( (out.armed ? 0 : 0x00ff00) ); //start of pulse - armed: off, disarmed: green
  if( modulus == imu.getSampleRate() / 10)  led.color( (out.armed ? 0xff0000 : 0) ); //end of pulse - armed: red, disarmed: off
}

//returns angle in range -180 to 180
float degreeModulus(float v) {
  if(v >= 180) {
    return fmod(v + 180, 360) - 180;
  }else if(v < -180.0) {
    return fmod(v - 180, 360) + 180;
  }
  return v;
}

void control_Angle(bool zero_integrators) {
  //DESCRIPTION: Computes control commands based on angle error
  /*
   * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des. Error
   * is simply the desired state minus the actual state (ex. roll_des - ahr.roll). Two safety features
   * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   * they've maxed out throttle... saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
   * terms will always start from 0 on takeoff. This function updates the variables PIDroll.PID, PIDpitch.PID, and PIDyaw.PID which
   * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in out_Mixer().
   */ 

  //inputs: roll_des, pitch_des, yawRate_des
  //outputs: PIDroll.PID, PIDpitch.PID, PIDyaw.PID

  //desired values
  float roll_des = rcl.roll * maxRoll; //Between -maxRoll and +maxRoll
  float pitch_des = rcl.pitch * maxPitch; //Between -maxPitch and +maxPitch
  float yawRate_des = rcl.yaw * maxYawRate; //Between -maxYawRate roll_PIDand +maxYawRate

  //state vars
  static float integral_roll, integral_pitch, error_yawRate_prev, integral_yawRate;

  //Zero the integrators (used to don't let integrator build if throttle is too low, or to re-start the controller)
  if(zero_integrators) {
    integral_roll = 0;
    integral_pitch = 0;
    integral_yawRate = 0;
  }

  //Roll PID
  float error_roll = roll_des - ahr.roll;
  integral_roll += error_roll * imu.dt;
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_roll = ahr.gx;
  pid.roll = 0.01 * (Kp_ro_pi_angle*error_roll + Ki_ro_pi_angle*integral_roll - Kd_ro_pi_angle*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch PID
  float error_pitch = pitch_des - ahr.pitch;
  integral_pitch += error_pitch * imu.dt;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_pitch = ahr.gy; 
  pid.pitch = 0.01 * (Kp_ro_pi_angle*error_pitch + Ki_ro_pi_angle*integral_pitch - Kd_ro_pi_angle*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw PID
  if(-0.02 < rcl.yaw && rcl.yaw < 0.02) {
    //on reset, set desired yaw to current yaw
    if(zero_integrators) yaw_desired = ahr.yaw; 

    //Yaw stick centered: hold yaw_desired
    float error_yaw = degreeModulus(yaw_desired - ahr.yaw);
    float desired_yawRate = error_yaw / 0.5; //set desired yawRate such that it gets us to desired yaw in 0.5 second
    float derivative_yaw = desired_yawRate - ahr.gz;
    pid.yaw = 0.01 * (Kp_yaw_angle*error_yaw + Kd_yaw_angle*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

    //update yaw rate controller
    error_yawRate_prev = 0;
  }else{
    //Yaw stick not centered: stablize on rate from GyroZ
    float error_yawRate = yawRate_des - ahr.gz;
    integral_yawRate += error_yawRate * imu.dt;
    integral_yawRate = constrain(integral_yawRate, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
    float derivative_yawRate = (error_yawRate - error_yawRate_prev) / imu.dt; 
    pid.yaw = 0.01 * (Kp_yaw_rate*error_yawRate + Ki_yaw_rate*integral_yawRate + Kd_yaw_rate*derivative_yawRate); //Scaled by .01 to bring within -1 to 1 range

    //Update derivative variables
    error_yawRate_prev = error_yawRate;

    //update yaw controller: 
    yaw_desired = ahr.yaw; //set desired yaw to current yaw, the yaw angle controller will hold this value
  }
}

void control_Rate(bool zero_integrators) {
  //Computes control commands based on state error (rate)
  //See explanation for control_Angle(). Everything is the same here except the error is now: desired rate - raw gyro reading.

  //inputs: roll_des, pitch_des, yawRate_des
  //outputs: PIDroll.PID, PIDpitch.PID, PIDyaw.PID

  //desired values
  float rollRate_des = rcl.roll * maxRollRate; //Between -maxRoll and +maxRoll
  float pitchRate_des = rcl.pitch * maxPitchRate; //Between -maxPitch and +maxPitch
  float yawRate_des = rcl.yaw * maxYawRate; //Between -maxYawRate and +maxYawRate 
  
  //state vars
  static float integral_roll, error_roll_prev;
  static float integral_pitch, error_pitch_prev;
  static float integral_yaw, error_yaw_prev;

  //Zero the integrators (used to don't let integrator build if throttle is too low, or to re-start the controller)
  if(zero_integrators) {
    integral_roll = 0;
    integral_pitch = 0;
    integral_yaw = 0;
  }

  //Roll
  float error_roll = rollRate_des - ahr.gx;
  integral_roll += error_roll * imu.dt;
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_roll = (error_roll - error_roll_prev) / imu.dt;
  pid.roll = 0.01 * (Kp_ro_pi_rate*error_roll + Ki_ro_pi_rate*integral_roll + Kd_ro_pi_rate*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  float error_pitch = pitchRate_des - ahr.gy;
  integral_pitch += error_pitch * imu.dt;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_pitch = (error_pitch - error_pitch_prev) / imu.dt;   
  pid.pitch = 0.01 * (Kp_ro_pi_rate*error_pitch + Ki_ro_pi_rate*integral_pitch + Kd_ro_pi_rate*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  float error_yaw = yawRate_des - ahr.gz;
  integral_yaw += error_yaw * imu.dt;
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_yaw = (error_yaw - error_yaw_prev) / imu.dt; 
  pid.yaw = 0.01 * (Kp_yaw_rate*error_yaw + Ki_yaw_rate*integral_yaw + Kd_yaw_rate*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update derivative variables
  error_roll_prev = error_roll;
  error_pitch_prev = error_pitch;
  error_yaw_prev = error_yaw;
}

void out_KillSwitchAndFailsafe() {
  //Change to ARMED when rcl is armed (by switch or stick command)
  if (!out.armed && rcl.armed) {
    out.armed = true;
    Serial.println("OUT: ARMED");
    bbx.start(); //start blackbox logging
  }

  //Change to DISARMED when rcl is disarmed, or if radio lost connection
  if (out.armed && (!rcl.armed || !rcl.connected())) {
    out.armed = false;
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
   * Takes PIDroll.PID, PIDpitch.PID, and PIDyaw.PID computed from the PID controller and appropriately mixes them for the desired
   * vehicle configuration. For example on a quadcopter, the left two motors should have +PIDroll.PID while the right two motors
   * should have -PIDroll.PID. Front two should have +PIDpitch.PID and the back two should have -PIDpitch.PID etc... every motor has
   * normalized (0 to 1) rcl.throttle command for throttle control. Can also apply direct unstabilized commands from the transmitter with 
   * rcl.xxx variables are to be sent to the motor ESCs and servos.
   * 
   *Relevant variables:
   *rcl.throtle - direct thottle control
   *PIDroll.PID, PIDpitch.PID, PIDyaw.PID - stabilized axis variables
   *rcl.roll, rcl.pitch, rcl.yaw - direct unstabilized command passthrough
   *rcl.flight_mode - can be used to toggle things with an 'if' statement
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
  // Set motor outputs to at least armed_min_throttle, to keep at least one prop spinning when armed. The [out] module will disable motors when out.armed == false
  float thr = armed_min_throttle + (1 - armed_min_throttle) * rcl.throttle; //shift motor throttle range from [0.0 .. 1.0] to [armed_min_throttle .. 1.0]

  if(rcl.throttle == 0) {
    //if throttle idle, then run props at low speed without applying PID. This allows for stick commands for arm/disarm.
    out.set(0, thr);
    out.set(1, thr);
    out.set(2, thr);
    out.set(3, thr);
  }else{
    // Quad mixing
    out.set(0, thr - pid.pitch - pid.roll - pid.yaw); //M1 Back Right CW
    out.set(1, thr + pid.pitch - pid.roll + pid.yaw); //M2 Front Right CCW
    out.set(2, thr - pid.pitch + pid.roll + pid.yaw); //M3 Back Left CCW
    out.set(3, thr + pid.pitch + pid.roll - pid.yaw); //M4 Front Left CW
  }
}
#endif