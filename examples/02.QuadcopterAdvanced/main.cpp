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
Copyright (c) 2023-2025 https://madflight.com
##########################################################################################################################*/

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
void control_Angle();
void control_Rate();
void out_KillSwitchAndFailsafe();
void out_Mixer();

//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //
//========================================================================================================================//

// IMPORTANT: This is a safety feature which keeps props spinning when armed, and hopefully reminds the pilot to disarm!!! 
const float armed_min_throttle = 0.20; //Minimum throttle when armed, set to a value between ~0.10 and ~0.25 which keeps the props spinning at minimum speed.

// Controller parameters (take note of defaults before modifying!): 
const float maxRoll        = 30.0;      //Max roll angle in degrees for angle mode (maximum ~70 degrees)
const float maxPitch       = 30.0;      //Max pitch angle in degrees for angle mode (maximum ~70 degrees)
const float maxRollRate    = 60.0;      //Max roll rate in deg/sec for rate mode 
const float maxPitchRate   = 60.0;      //Max pitch rate in deg/sec for rate mode
const float maxYawRate     = 160.0;     //Max yaw rate in deg/sec for angle and rate mode

// PID controllers for ANGLE Mode - Kp, Ki, Kd, scale_factor, i_limit
PIDController pidRol(2.0, 1.0, 0.5, 0.001, 250); //Roll (deg)
PIDController pidPit(2.0, 1.0, 0.5, 0.001, 250); //Pitch (deg)
PIDController pidYaw(6.0, 0.0, 1.0, 0.001, 250); //Yaw (deg)

// PID controllers for RATE Mode - Kp, Ki, Kd, scale_factor, i_limit
PIDController pidRolRate(1.5, 2.0, 0.0020, 0.001, 250); //Roll Rate (deg/s)
PIDController pidPitRate(1.5, 2.0, 0.0020, 0.001, 250); //Pitch Rate (deg/s)
PIDController pidYawRate(3.0, 0.5, 0.0015, 0.001, 250); //Yaw Rate (deg/s)

// Be careful when increasing Kd too high, motors will begin to overheat!
// PIDController scale_factor is 0.001 to bring PID output to -1.0 to +1.0 range

//Yaw to keep in ANGLE mode when yaw stick is centered
float yaw_desired = 0;

//========================================================================================================================//
//                                                       SETUP()                                                          //
//========================================================================================================================//

void setup() {
  madflight_setup(); //setup madflight modules

  // STOP if imu is not installed
  if(!imu.installed()) madflight_die("This program needs an IMU.");

  // Setup 4 motors for the quadcopter
  int motor_idxs[] = {0, 1, 2, 3}; //motor indexes
  int motor_pins[] = {cfg.pin_out0, cfg.pin_out1, cfg.pin_out2, cfg.pin_out3}; //motor pins

  // Uncomment ONE line - select output type
  bool success = out.setupMotors(4, motor_idxs, motor_pins, 400, 950, 2000);   // Standard PWM: 400Hz, 950-2000 us
  //bool success = out.setupMotors(4, motor_idxs, motor_pins, 2000, 125, 250); // Oneshot125: 2000Hz, 125-250 us
  //bool success = out.setupDshot(4, motor_idxs, motor_pins, 300);             // Dshot300
  //bool success = out.setupDshotBidir(4, motor_idxs, motor_pins, 300);        // Dshot300 Bi-Directional
  if(!success) madflight_die("Motor init failed.");

  // Set initial desired yaw
  yaw_desired = ahr.yaw;
}

//========================================================================================================================//
//                                                            LOOP()                                                      //
//========================================================================================================================//

void loop() {
  rdr.update(); // radar (not used in this example)
  ofl.update(); // optical flow (not used in this example)
  
  //update battery sensor, log if updated (except for logging, not used in this example)
  if(bat.update()) { 
    bbx.log_bat();
  } 
  
  //update altitude estimator (not used in this example)
  alt.updateAccelUp(ahr.getAccelUp(), ahr.ts); //NOTE: do this here and not in imu_loop() because `alt` object is not thread safe. - Update altitude estimator with current earth-frame up acceleration measurement
  
  //update barometer sensor, log if updated (except for logging, not used in this example)
  if(bar.update()) {
    alt.updateBarAlt(bar.alt, bar.ts); //update altitude estimator with current altitude measurement
    bbx.log_bar(); //log if pressure updated
  }

  mag.update(); //magnetometer (used to calculate yaw heading)
  
  //update gps, and log GPS and ATT for plot.ardupilot.org visualization  (except for logging, not used in this example)
  if(gps.update()) {
    bbx.log_gps(); 
    bbx.log_att();
  } 

  //log system status
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

  //Zero the PID integrators if throttle is zero (don't let integrator build if throttle is too low, or to re-start the controller)
  if(rcl.throttle == 0) {
    pidRol.reset();
    pidPit.reset();
    pidYaw.reset();
    pidRolRate.reset();
    pidPitRate.reset();
    pidYawRate.reset();
    //on reset, set desired yaw to current yaw
    yaw_desired = ahr.yaw; 
  }

  //Select PID controller based on flight mode switch
  switch( veh.getFlightmode() ) {
    case ANGLE: 
      control_Angle(); //Stabilize on pitch/roll angle setpoint, stabilize yaw on rate setpoint
      break;
    default: //RATE 
      control_Rate(); //Stabilize on rate setpoint
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

void control_Rate() {
  //Computes control commands based on radio control (RCL) inputs and actual gyro rates

  //Roll rate, pitch rate, yaw rate PID control - get desired rate from radio controller and stablize on rate from gyro
  pid.roll = pidRol.control(rcl.roll  * maxRollRate,  ahr.gx, imu.dt);
  pid.pitch = pidPit.control(rcl.pitch * maxPitchRate, ahr.gy, imu.dt); 
  pid.yaw = pidPit.control(rcl.yaw   * maxYawRate,   ahr.gz, imu.dt);
}

void control_Angle() {
  //Computes control commands based on radio control (RCL) inputs and actual roll, pitch, yaw angles

  //Roll PID - use actual gyro roll rate (ahr.gx) instead of derivative error
  pid.roll = pidRol.controlDegrees(rcl.roll * maxRoll, ahr.roll, imu.dt, ahr.gx);

  //Pitch PID - use actual gyro pitch rate (ahr.gy) instead of derivative error
  pid.pitch = pidPit.controlDegrees(rcl.pitch * maxPitch, ahr.pitch, imu.dt, ahr.gy);

  //Yaw PID
  if(-0.02 < rcl.yaw && rcl.yaw < 0.02) {
    //Yaw stick centered: hold yaw_desired
    pid.yaw = pidYaw.controlDegrees(yaw_desired, ahr.yaw, imu.dt, ahr.gz);
  }else{
    //Yaw stick not centered: stablize on rate from GyroZ
    pid.yaw = pidYawRate.controlDegrees(rcl.yaw * maxYawRate, ahr.gz, imu.dt);

    //set desired yaw to current yaw, the yaw angle controller will hold this value
    yaw_desired = ahr.yaw; 
  }
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
   * Takes pid.roll, pid.pitch, and pid.yaw computed from the PID controller and appropriately mixes them for the desired
   * vehicle configuration. For example on a quadcopter, the left two motors should have +pid.roll while the right two motors
   * should have -pid.roll. Front two should have +pid.pitch and the back two should have -pid.pitch etc... every motor has
   * normalized (0 to 1) rcl.throttle command for throttle control. You can also apply direct unstabilized commands from the 
   * transmitter with rcl.xxx variables are to be sent to the motor ESCs and servos.
   * 
   * Relevant variables:
   *   rcl.throtle - direct thottle control
   *   pid.roll, pid.pitch, pid.yaw - stabilized axis variables
   *   rcl.roll, rcl.pitch, rcl.yaw - direct unstabilized command passthrough
   *   rcl.flight_mode - can be used to toggle things with an 'if' statement
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
    out.set(0, thr - pid.pitch - pid.roll - pid.yaw); //OUT0 Motor1 Back Right CW
    out.set(1, thr + pid.pitch - pid.roll + pid.yaw); //OUT1 Motor2 Front Right CCW
    out.set(2, thr - pid.pitch + pid.roll + pid.yaw); //OUT2 Motor3 Back Left CCW
    out.set(3, thr + pid.pitch + pid.roll - pid.yaw); //OUT3 Motor4 Front Left CW
  }
}
