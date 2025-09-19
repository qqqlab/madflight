/*#########################################################################################################################

Minimal quadcopter demo program for madflight Arduino ESP32-S3 / ESP32 / RP2350 / RP2040 / STM32 Flight Controller

###########################################################################################################################

See http://madflight.com for detailed description

Required Hardware

    IMU sensor (SPI or I2C)
    RC receiver with 5 channels (CRSF/ELRS preferred)
    4 brushless motors with ESCs

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

MIT license
Copyright (c) 2023-2025 https://madflight.com
##########################################################################################################################*/

#include <Arduino.h>
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

//IMPORTANT: This is a safety feature which keeps props spinning when armed, and hopefully reminds the pilot to disarm!!! 
const float armed_min_throttle = 0.20; //Minimum throttle when armed, set to a value between ~0.10 and ~0.25 which keeps the props spinning at minimum speed.

//Flight Mode: Uncommment only one
#define FLIGHTMODE_RATE   //control rate - stick centered will keep current roll/pitch angle
//#define FLIGHTMODE_ANGLE  //control angle - stick centered will return to horizontal - IMPORTANT: execute CLI 'calimu' and 'save' before using this!!!

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

  // Setup 4 motors for the quadcopter
  int motor_idxs[] = {0, 1, 2, 3}; //motor indexes
  int motor_pins[] = {cfg.pin_out0, cfg.pin_out1, cfg.pin_out2, cfg.pin_out3}; //motor pins

  // Uncomment ONE line - select output type
  bool success = out.setupMotors(4, motor_idxs, motor_pins, 400, 950, 2000);   // Standard PWM: 400Hz, 950-2000 us
  //bool success = out.setupMotors(4, motor_idxs, motor_pins, 2000, 125, 250); // Oneshot125: 2000Hz, 125-250 us
  //bool success = out.setupDshot(4, motor_idxs, motor_pins, 300);             // Dshot300
  //bool success = out.setupDshotBidir(4, motor_idxs, motor_pins, 300);        // Dshot300 Bi-Directional
  if(!success) madflight_die("Motor init failed.");

  //set initial desired yaw
  yaw_desired = ahr.yaw;
}

//========================================================================================================================//
//                                                            LOOP()                                                      //
//========================================================================================================================//

void loop() {
  mag.update(); // magnetometer (used to calculate yaw heading)
  bar.update(); // barometer (not used in this example)
  gps.update(); // gps (not used in this example)
  bat.update(); // battery consumption (not used in this example)
  rdr.update(); // radar (not used in this example)
  ofl.update(); // optical flow (not used in this example)

  cli.update(); //process CLI commands
}

//========================================================================================================================//
//                                                   IMU UPDATE LOOP                                                      //
//========================================================================================================================//

//This is __MAIN__ function of this program. It is called when new IMU data is available.
void imu_loop() {
  //Blink LED
  led_Blink();

  //Sensor fusion: update ahr.roll, ahr.pitch, and ahr.yaw angle estimates (degrees) from IMU data
  ahr.update(); 

  //Get radio commands - Note: don't do this in loop() because loop() is a lower priority task than imu_loop(), so in worst case loop() will not get any processor time.
  rcl.update();

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

  //PID Controller RATE or ANGLE based on FLIGHTMODE_ANGLE define
  #ifdef FLIGHTMODE_ANGLE
    control_Angle(); //Stabilize on angle setpoint
  #else
    control_Rate(); //Stabilize on rate setpoint
  #endif

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
  pid.roll = pidRol.controlDegreesActualDerivative(rcl.roll * maxRoll, ahr.roll, imu.dt, ahr.gx);

  //Pitch PID - use actual gyro pitch rate (ahr.gy) instead of derivative error
  pid.pitch = pidPit.controlDegreesActualDerivative(rcl.pitch * maxPitch, ahr.pitch, imu.dt, ahr.gy);

  //Yaw PID
  if(-0.02 < rcl.yaw && rcl.yaw < 0.02) {
    //Yaw stick centered: hold yaw_desired
    pid.yaw = pidYaw.controlDegreesActualDerivative(yaw_desired, ahr.yaw, imu.dt, ahr.gz);
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
  }

  //Change to DISARMED when rcl is disarmed, or if radio lost connection
  if (out.armed && (!rcl.armed || !rcl.connected())) {
    out.armed = false;
    if(!rcl.armed) {
      Serial.println("OUT: DISARMED");
    }else{
      Serial.println("OUT: DISARMED due to lost radio connection");
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
