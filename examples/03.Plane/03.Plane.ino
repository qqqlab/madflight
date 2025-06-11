/*#########################################################################################################################

WARNING: This program is experimental - it was only flight-tested in a couple of flights

This program is an airplane controller, it has 3 flight modes: MANUAL, ROLL and FBWA.

## MANUAL Mode

Regular RC control, no stabilization. All RC inputs are passed through to the servo outputs.

## ROLL Mode

Stabilize roll angle

## FBWA Fly By Wire A Mode (inspired by ArduPilot)

This is the most popular mode for assisted flying, and is the best mode for inexperienced flyers. In this mode the
plane will hold the roll and pitch specified by the control sticks. So if you hold the aileron stick hard right then the 
plane will hold its pitch level and will bank right by the angle specified in the roll limit parameter. It is not possible 
to roll the plane past the roll limit, and it is not possible to pitch the plane beyond the pitch limit settings.

Note that holding level pitch does not mean the plane will hold altitude. How much altitude a plane gains or loses at a 
particular pitch depends on its airspeed, which is primarily controlled by throttle. So to gain altitude you should raise 
the throttle, and to lose altitude you should lower the throttle.

In FBWA mode the rudder is under manual control.

## Setup Procedure

First edit sections "PINS", "BOARD", "HARDWARE", "RC RECEIVER", "OUTPUTS". Use CLI to verify things work as expected.

Calibrate gyro, accelerometer, and magnetometer --- important!!!

Connect power and then let plane sit for 15 seconds, during this time the gyro biases are re-calibrated.

Do a dry run:

Set to MANUAL and power up the plane. Move the rc controls and make sure that the aileron, elevator, and rudder move in 
the correct direction. Arm the plane, and carefully test the motor, then disarm.
If incorrect: modify the #define OUT_ELEVATOR_DOWN etc. statements.

Then set to FBWA flight mode, keep the radio sticks centered, and move the plane around, to make sure that the control 
surfaces work to oppose the move, that is: pitching the plane down should move elevator up, banking right should deflect 
the right aileron down, left aileron up. 

Another thing that needs to be set are the PID parameters. Set to ROLL or FBWA mode and adjust the PID parameters so that the 
control surfaces react quickly, but don't oscillate, on changes in attitude.

###########################################################################################################################

See http://madflight.com for detailed description

Arming/disarming with dedicated switch

    Arm: Set throttle low, then flip arm switch from DISARMED to ARMED.
    Disarm: Flip arm switch from ARMED to DISARMED, at any throttle position. "Kill switch".

Arming/disarming with sticks (when no arm switch is defined, i.e. cfg.rcl_arm_ch == 0 ) 

    Arm: Pull both sticks toward you, yaw full right, and roll full left
    Disarm: Pull both sticks toward you, yaw full left, and roll full right

LED State                              Meaning
---------                              -------
OFF                                    Not powered
ON                                     Startup (don't move, running gyro calibration)
Blinking long OFF short ON             DISARMED
Blinking long ON short OFF             ARMED
Blink interval longer than 1 second    imu_loop() is taking too much time
fast blinking                          Something is wrong, connect USB serial for info

MIT license
Copyright (c) 2024-2025 https://madflight.com
##########################################################################################################################*/

//Vehicle specific madflight configuration
#define VEH_TYPE VEH_TYPE_PLANE //set the vehicle type for logging and mavlink
#define VEH_FLIGHTMODE_AP_IDS {AP_PLANE_FLIGHTMODE_MANUAL, AP_PLANE_FLIGHTMODE_STABILIZE, AP_PLANE_FLIGHTMODE_FLY_BY_WIRE_A} //(approximate) mapping of fightmode index to ArduPilot code for logging and mavlink
#define VEH_FLIGHTMODE_NAMES {"MANUAL", "ROLL", "FBWA"} //fightmode names for telemetry
enum flightmode_enum {MANUAL, ROLL, FBWA}; //available flight modes: MANUAL send rc commands directly to motor and aileron/pitch/yaw servos, ROLL stabilize roll angle, FBWA stabilize roll/pitch angles
flightmode_enum rcin_to_flightmode_map[6] {MANUAL, MANUAL, ROLL, ROLL, FBWA, FBWA}; //flightmode mapping from 3/6 pos switch to flight mode (simulates a 3-pos switch: MANUAL/ROLL/FBWA)

#include "madflight_config.h" //Edit this header file to setup the pins, hardware, radio, etc. for madflight
#include <madflight.h>

//prototypes (for PlatformIO, not needed for Arduino IDE)
void led_Blink();
void control_FBWA(bool zero_integrators);
void control_ROLL(bool zero_integrators);
void control_MANUAL();
void out_Mixer()

//========================================================================================================================//
//                                               OUTPUTS                                                                  //
//========================================================================================================================//

//define outputs and their 1-based channel (NOTE: pin names are 0-based)
//select output name based on what the output does when pwm is high. For example: If the right aileron goes down on high 
//pwm and is connected to output channel 2 use #define OUT_RIGHT_AILERON_DOWN 2
#define OUT_MOTOR 1 //full throttle on high pwm (motor should be first channel)
#define OUT_LEFT_AILERON_UP 2 //left aileron deflects up on high pwm (and right aileron down, otherwise use two servo channels)
//#define OUT_RIGHT_AILERON_DOWN 3 //right aileron deflects down on high pwm
#define OUT_ELEVATOR_UP 3 //elevator deflects up on high pwm
#define OUT_RUDDER_LEFT 4 //rudder deflects left on high pwm
#define OUT_FLAPS_UP_HALF 5 //flaps deflect up on high pwm, but only use 0.5 to 1.0 servo range

//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //
//========================================================================================================================//

//Controller parameters (take note of defaults before modifying!): 
float i_limit        = 25;        //PID Integrator saturation level, mostly for safety
float maxRoll        = 45;        //Max roll angle in degrees for ROLL, FBWA modes
float maxPitch       = 30;        //Max pitch angle in degrees for FBWA mode
float fbwa_pitch_offset = 3;      //FBWA pitch up angle for neutral stick

//roll PID constants
float Kp_roll        = (1.0/90);  //Roll P-gain - apply full aileron on 90 degree roll error
float Ki_roll        = 0;         //Roll I-gain
float Kd_roll        = (1.0/180); //Roll D-gain - apply full opposite aileron when roll rate is 180 degrees/sec towards desired setpoint

//pitch PID constants
float Kp_pitch       = (1.0/30);  //Pitch P-gain - apply full elevator on 30 degree pitch error
float Ki_pitch       = 0;         //Pitch I-gain
float Kd_pitch       = (1.0/90);  //Pitch D-gain - apply full opposite elevator when pitch rate is 90 degrees/sec towards desired setpoint

//Radio communication:
float rcin_flaps; //flaps 0.0:up, 1.0:full down

//========================================================================================================================//
//                                                       SETUP()                                                          //
//========================================================================================================================//

void setup() {
  //setup madflight components: Serial.begin(115200), imu, rcin, led, etc. See src/madflight/interface.h for full interface description of each component. 
  madflight_setup();

  //Standard servo at 50Hz (set servos first just in case motors overwrite frequency of shared timers)
  out.setupServo(1, cfg.pin_out1, 50, 1000, 2000); //Aileron
  out.setupServo(2, cfg.pin_out2, 50, 1000, 2000); //Elevator
  out.setupServo(3, cfg.pin_out3, 50, 1000, 2000); //Rudder
  out.setupServo(4, cfg.pin_out4, 50, 1000, 2000); //Flaps

  //Motor
  //uncomment one line - sets pin, frequency (Hz), minimum (us), maximum (us)
  out.setupMotor(0, cfg.pin_out0, 400, 950, 2000); //Standard PWM: 400Hz, 950-2000 us
  //out.setupMotor(0, cfg.pin_out0, 2000, 125, 250); //Oneshot125: 2000Hz, 125-250 us
}

//========================================================================================================================//
//                                                            LOOP()                                                      //
//========================================================================================================================//

void loop() {
  //update all I2C sensors
  if(bat.update()) bbx.log_bat(); //update battery, and log if battery was updated. 
  if(bar.update()) bbx.log_bar(); //log if pressure updated
  mag.update();

  if(gps.update()) {bbx.log_gps(); bbx.log_att();} //update gps (and log GPS and ATT for plot.ardupilot.org visualization)

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

//This is __MAIN__ function of this program. It is called when new IMU data is available.
void imu_loop() {
  //Blink LED
  led_Blink();

  //Sensor fusion: update ahr.roll, ahr.pitch, and ahr.yaw angle estimates (degrees) from IMU data
  ahr.update(); 

  //Get radio commands - Note: don't do this in loop() because loop() is a lower priority task than imu_loop(), so in worst case loop() will not get any processor time.
  rcl.update();
  veh.setFlightmode( rcin_to_flightmode_map[rcl.flightmode] ); //map rcl.flightmode (0 to 5) to vehicle flightmode

  //PID Controller
  switch( veh.getFlightmode() ) {
    case ROLL:
      control_ROLL(rcl.throttle == 0); //Stabilize on roll angle setpoints
      break;    
    case FBWA:
      control_FBWA(rcl.throttle == 0); //Stabilize on pitch/roll angle setpoints
      break;
    default:
      control_MANUAL();
  }

  //Updates out.arm, the output armed flag
  out_KillSwitchAndFailsafe(); //Cut all motor outputs if DISARMED or failsafe triggered.

  //Actuator mixing
  out_Mixer(); //Mixes PID outputs and sends command pulses to the motors, if mot.arm == true

  //bbx.log_imu(); //full speed black box logging of IMU data, memory fills up quickly...
}

//========================================================================================================================//
//                      IMU UPDATE LOOP FUNCTIONS - in same order as they are called from imu_loop()                           //
//========================================================================================================================//

void led_Blink() {
  //Blink LED once per second, if LED blinks slower then the loop takes too much time, use CLI 'pimu' to investigate.
  //DISARMED: long off, short on, ARMED: long on, short off
  uint32_t modulus = imu.update_cnt % imu.getSampleRate();
  if( modulus == 0) led.set(!out.armed); //start of pulse
  if( modulus == imu.getSampleRate() / 10)  led.set(out.armed); //end of pulse
}

void control_FBWA(bool zero_integrators) {
/* FBWA Fly By Wire A Mode (inspired by ArduPilot)
This is the most popular mode for assisted flying, and is the best mode for inexperienced flyers. In this mode the
plane will hold the roll and pitch specified by the control sticks. So if you hold the aileron stick hard right then the 
plane will hold its pitch level and will bank right by the angle specified in the roll limit parameter. It is not possible 
to roll the plane past the roll limit, and it is not possible to pitch the plane beyond the pitch limit settings.

Note that holding level pitch does not mean the plane will hold altitude. How much altitude a plane gains or loses at a 
particular pitch depends on its airspeed, which is primarily controlled by throttle. So to gain altitude you should raise 
the throttle, and to lose altitude you should lower the throttle.

In FBWA mode the rudder is under manual control.
*/

  //inputs: roll_des, pitch_des, yawRate_des
  //outputs: PIDroll.PID, PIDpitch.PID, PIDyaw.PID

  //desired values
  float roll_des = rcl.roll * maxRoll; //Between -maxRoll and +maxRoll
  float pitch_des = rcl.pitch * maxPitch + fbwa_pitch_offset; //Between fbwa_pitch_offset-maxPitch and fbwa_pitch_offset+maxPitch

  //state vars
  static float integral_roll, integral_pitch, error_yaw_prev, integral_yaw;

  //Zero the integrators (used to don't let integrator build if throttle is too low, or to re-start the controller)
  if(zero_integrators) {
    integral_roll = 0;
    integral_pitch = 0;
    integral_yaw = 0;
  }

  //Roll PID - stabilize desired roll angle
  float error_roll = roll_des - ahr.roll;
  integral_roll += error_roll * imu.dt;
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_roll = ahr.gx;
  PIDroll.PID = Kp_roll*error_roll + Ki_roll*integral_roll - Kd_roll*derivative_roll; //nominal output -1 to 1 (can be larger)

  //Pitch PID - stabilize desired pitch angle
  float error_pitch = pitch_des - ahr.pitch;
  integral_pitch += error_pitch * imu.dt;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_pitch = ahr.gy; 
  PIDpitch.PID = Kp_pitch*error_pitch + Ki_pitch*integral_pitch - Kd_pitch*derivative_pitch; //nominal output -1 to 1 (can be larger)

  //Yaw PID - passthru rcin
  PIDyaw.PID = rcl.yaw;
  (void) integral_yaw;
  (void) error_yaw_prev;

  /*
  //TODO Yaw PID - Stabilize on zero slip, i.e. keep gravity Y component zero
  float error_yaw = 0 - ahr.ay;
  integral_yaw += error_yaw * imu.dt;
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_yaw = (error_yaw - error_yaw_prev) / imu.dt; 
  PIDyaw.PID = constrain(0.01 * (Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw), -1.0f, 1.0f); //Scaled by .01 to bring within -1 to 1 range

  //Update derivative variables
  error_yaw_prev = error_yaw;
  */  
}

void control_ROLL(bool zero_integrators) {
  //inputs: roll_des, pitch_des, yawRate_des
  //outputs: PIDroll.PID, PIDpitch.PID, PIDyaw.PID

  //desired values
  float roll_des = rcl.roll * maxRoll; //Between -maxRoll and +maxRoll

  //state vars
  static float integral_roll;

  //Zero the integrators (used to don't let integrator build if throttle is too low, or to re-start the controller)
  if(zero_integrators) {
    integral_roll = 0;
  }

  //Roll PID - stabilize desired roll angle
  float error_roll = roll_des - ahr.roll;
  integral_roll += error_roll * imu.dt;
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_roll = ahr.gx;
  PIDroll.PID = Kp_roll*error_roll + Ki_roll*integral_roll - Kd_roll*derivative_roll; //nominal output -1 to 1 (can be larger)

  //Pitch PID - passthru rcin
  PIDpitch.PID = rcl.pitch;

  //Yaw PID - passthru rcin
  PIDyaw.PID = rcl.yaw; 
}


void control_MANUAL() {
/* MANUAL Mode
Regular RC control, no stabilization. All RC inputs are passed through to the servo outputs.
*/  
  //pass rcin through to PID - PID values are -1 to +1, rcin values are -1 to +1
  PIDroll.PID = rcl.roll;  //-1 = left, 1 = right
  PIDpitch.PID = rcl.pitch; //-1 = pitch up/stick back, 1 = pitch down/stick forward
  PIDyaw.PID = rcl.yaw; //-1 = left, 1 = right
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
   * rcin_xxx variables are to be sent to the motor ESCs and servos.
   * 
   *Relevant variables:
   *rcl.throttle - direct thottle control
   *PIDroll.PID, PIDpitch.PID, PIDyaw.PID - stabilized axis variables
   *rcl.roll, rcl.pitch, rcl.yaw - direct unstabilized command passthrough

    rcl.throttle   0: idle throttle/stick back  1: full throttle/stick forward
    PIDroll.PID   -1: roll left/stick left      1: roll right/stick right
    PIDpitch.PID  -1: pitch up/stick back       1: pitch down/stick forward
    PIDyaw.PID    -1: yaw left/stick left       1: yaw right/stick right
   */

  //Plane mixing - PID values are -1 to +1 (nominal), SERVO values are 0 to 1 (clipped by pwm class)

  //motor: full throttle on rcl.throttle
  #ifdef OUT_MOTOR //full throttle on high pwm
    out.set(OUT_MOTOR-1, +rcl.throttle);
  #endif
  #ifdef OUT_MOTOR1_REVERSED //reversed: idle throttle on high pwm
    out.set(OUT_MOTOR1_REVERSED-1, 1.0 - rcl.throttle);
  #endif  
  #ifdef OUT_MOTOR2 //full throttle on high pwm
    out.set(OUT_MOTOR2-1, +rcl.throttle);
  #endif  
  #ifdef OUT_MOTOR2_REVERSED //reversed: idle throttle on high pwm
    out.set(OUT_MOTOR2_REVERSED-1, 1.0 - rcl.throttle);
  #endif 

  //aileron: when PIDroll.PID positive -> roll right -> deflect left aileron down, deflect right aileron up
  #ifdef OUT_LEFT_AILERON_DOWN //left aileron deflects down on high pwm
    out.set(OUT_LEFT_AILERON_DOWN-1, 0.5 +PIDroll.PID/2.0);
  #endif
  #ifdef OUT_RIGHT_AILERON_UP //right aileron deflects up on high pwm
    out.set(OUT_RIGHT_AILERON_UP-1, 0.5 +PIDroll.PID/2.0);
  #endif
  #ifdef OUT_LEFT_AILERON_UP //reversed: left aileron deflects up on high pwm
    out.set(OUT_LEFT_AILERON_UP-1, 0.5 -PIDroll.PID/2.0);
  #endif
  #if defined(OUT_RIGHT_AILERON_DOWN) //reversed: right aileron deflects down on high pwm
    out.set(OUT_RIGHT_AILERON_DOWN-1, 0.5 -PIDroll.PID/2.0);
  #endif

  //elevator: when PIDpitch.PID is positive -> pitch up -> deflect elevator down 
  #ifdef OUT_ELEVATOR_DOWN //elevator deflects down on high pwm
    out.set(OUT_ELEVATOR_UP-1, +PIDpitch.PID/2.0 + 0.5);
  #endif
  #ifdef OUT_ELEVATOR_UP //reversed: elevator deflects up on high pwm
    out.set(OUT_ELEVATOR_UP-1, -PIDpitch.PID/2.0 + 0.5);
  #endif

  //rudder: when PIDyaw.PID is positive -> yaw right -> deflect rudder right
  #ifdef OUT_RUDDER_RIGHT //rudder deflects right on high pwm 
    out.set(OUT_RUDDER_RIGHT-1, +PIDyaw.PID/2.0 + 0.5);
  #endif  
  #ifdef OUT_RUDDER_LEFT //reversed: rudder deflects left on high pwm
    out.set(OUT_RUDDER_LEFT-1, -PIDyaw.PID/2.0 + 0.5);
  #endif

  //flaps: (rcin_flaps 0.0:up, 1.0:full down)
  #ifdef OUT_FLAPS_DOWN //flaps deflect down on high pwm (flaps use full servo range 0.0 to 1.0)
    float rcin_flaps = constrain( ((float)(rcl.pwm[OUT_FLAPS_DOWN-1] - 1100)) / (1900 - 1100), 0.0, 1.0); //output: 0.0 to 1.0
    out.set(OUT_FLAPS_DOWN-1, +rcin_flaps);
  #endif
  #ifdef OUT_FLAPS_DOWN_HALF //flaps deflect down on high pwm (flaps only use servo range 0.5 to 1.0)
    float rcin_flaps = constrain( ((float)(rcl.pwm[OUT_FLAPS_DOWN_HALF-1] - 1100)) / (1900 - 1100), 0.0, 1.0); //output: 0.0 to 1.0
    out.set(OUT_FLAPS_DOWN_HALF-1, 0.5 + rcin_flaps/2.0);
  #endif  
  #ifdef OUT_FLAPS_UP //reversed: flaps deflect up on high pwm (flaps use full servo range 0.0 to 1.0)
    float rcin_flaps = constrain( ((float)(rcl.pwm[OUT_FLAPS_UP-1] - 1100)) / (1900 - 1100), 0.0, 1.0); //output: 0.0 to 1.0
    out.set(OUT_FLAPS_UP-1, -rcin_flaps);
  #endif
  #ifdef OUT_FLAPS_UP_HALF //reversed: flaps deflect up on high pwm (flaps only use servo range 0.5 to 1.0)
    float rcin_flaps = constrain( ((float)(rcl.pwm[OUT_FLAPS_UP_HALF-1] - 1100)) / (1900 - 1100), 0.0, 1.0); //output: 0.0 to 1.0
    out.set(OUT_FLAPS_UP_HALF-1, 0.5 - rcin_flaps/2.0);
  #endif 

  //delta wing:
  // when PIDroll.PID positive -> roll right -> deflect left elevon down, deflect right elevon up
  // when PIDpitch.PID is positive -> pitch up -> deflect left elevon down, deflect right elevon down 
  #ifdef OUT_LEFT_ELEVON_DOWN //left elevon deflects down on high input
    out.set(OUT_LEFT_ELEVON_DOWN-1, 0.5 +PIDroll.PID/2.0 +PIDpitch.PID/2.0);
  #endif
  #ifdef OUT_RIGHT_ELEVON_UP //right elevon deflects up on high input
    out.set(OUT_RIGHT_ELEVON_UP-1, 0.5 +PIDroll.PID/2.0 -PIDpitch.PID/2.0);
  #endif
  #ifdef OUT_LEFT_ELEVON_UP //reversed: left elevon deflects down on high input
    out.set(OUT_LEFT_ELEVON_UP-1, 0.5 -PIDroll.PID/2.0 -PIDpitch.PID/2.0);
  #endif  
  #ifdef OUT_RIGHT_ELEVON_DOWN //reversed: right elevon deflects down on high input
    out.set(OUT_RIGHT_ELEVON_DOWN-1, 0.5 -PIDroll.PID/2.0 +PIDpitch.PID/2.0);
  #endif 

  //0.0 is zero throttle if connecting to ESC for conventional PWM, 1.0 is max throttle
  //0.5 is centered servo, 0.0 and 1.0 are servo at their extreme positions as set with SERVO_MIN and SERVO_MAX
}
