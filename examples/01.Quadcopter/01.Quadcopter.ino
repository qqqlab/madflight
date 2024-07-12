/*#########################################################################################################################

Minimal quadcopter demo program for madflight Arduino ESP32 / RP2040 / STM32 Flight Controller

###########################################################################################################################

See http://madflight.com for detailed description

Needs: 
 - IMU sensor (SPI or I2C)
 - RC receiver with 5 channels (CRSF/ELRS preferred)
 - 4 brushless motors with ESCs

Connecting:
IMU: connect IMU_EXTI, IMU_CS, SPI_MISO, SPI_MOSI, SPI_CLK for SPI (or IMU_EXTI, I2C_SDA, I2C_SCL for I2C)
RC receiver: connect RCIN_RX to receiver TX pin
ESCs: PWM1-4 to the ESC inputs

Motor order diagram (Betaflight order)

      front
 CW -->   <-- CCW
     4     2 
      \ ^ /
       |X|
      / - \
     3     1 
CCW -->   <-- CW

Default flight mode is ACRO (rate). The mode can be changed to ANGLE by changing the PID controller in imu_loop(). Important: calibrate the accelometer before using ANGLE.  

Arming: Set throttle low, then flip arm switch from DISARMED to ARMED.
Disarming: Flip arm switch from ARMED to DISARMED, at any throttle position. "Kill switch".

LED Status
==========
OFF - not powered
startup: a couple blinks then ON while running gyro calibration (don't move)
blinking long OFF short ON - DISARMED
blinking long ON short OFF - ARMED
blink interval longer than 1 second - imu_loop() is taking too much time
fast blinking - something is wrong, connect USB serial for info

GPL-3.0 license
Copyright (c) 2023-2024 https://github.com/qqqlab/madflight
Copyright (c) 2022 Nicholas Rehm - dRehmFlight
##########################################################################################################################*/

//========================================================================================================================//
//                                                 PINS                                                                   //
//========================================================================================================================//
// PINS are defined in the board header file library/src/madflight_board_default_XXX.h, but you can use these defines to 
// override the pins. 

/*
//The pin numbers below are an example for an easy soldering ESP32 WeMos LOLIN32-Lite with GY-6500/GY-271/GY-BPM280 sensor modules

//LED:
#define HW_PIN_LED       22
#define HW_LED_ON         0 //0:low is on, 1:high is on

//IMU SPI:
#define HW_PIN_SPI_MISO  25
#define HW_PIN_SPI_MOSI  14
#define HW_PIN_SPI_SCLK  12
#define HW_PIN_IMU_CS    32
#define HW_PIN_IMU_EXTI  33 //external interrupt pin

//I2C for IMU if not using SPI
#define HW_PIN_I2C_SDA   23
#define HW_PIN_I2C_SCL   19

//Motor/Servo Outputs:
#define HW_OUT_COUNT     4 //number of outputs
#define HW_PIN_OUT_LIST  {13,15,2,0} //list of output pins

//RC Receiver:
#define HW_PIN_RCIN_RX    16
#define HW_PIN_RCIN_TX     4
#define HW_PIN_RCIN_INVERTER -1 //only used for STM32 targets
//*/

//========================================================================================================================//
//                                                 BOARD                                                                  //
//========================================================================================================================//
// Uncomment/change the following #include to the flight controller you want to use, or leave commented out to use the
// default board pinout (madflight_board_default_*.h). See library/madflight/src for all available boards

//#include <madflight_board_betaflight_MTKS-MATEKH743.h>

//========================================================================================================================//
//                                                 USER-SPECIFIED DEFINES                                                 //
//========================================================================================================================//

//--- RC RECEIVER
#define RCIN_USE  RCIN_USE_CRSF //RCIN_USE_CRSF, RCIN_USE_SBUS, RCIN_USE_DSM, RCIN_USE_PPM, RCIN_USE_PWM
#define RCIN_NUM_CHANNELS 5 //number of receiver channels (minimal 5)

//--- IMU SENSOR
#define IMU_USE  IMU_USE_SPI_MPU6500 // IMU_USE_SPI_MPU6500, IMU_USE_SPI_MPU9250,IMU_USE_SPI_MPU6000, IMU_USE_SPI_BMI270, IMU_USE_I2C_MPU9250, IMU_USE_I2C_MPU9150, IMU_USE_I2C_MPU6500, IMU_USE_I2C_MPU6050, IMU_USE_I2C_MPU6000
//Set sensor orientation. The label is yaw / roll (in that order) needed to rotate the sensor from it's normal position to it's mounted position.
//If not sure what is needed: use CLI 'proll' and try each setting until roll-right gives positive ahrs.roll, pitch-up gives positive ahrs.pitch, and yaw-right gives positive ahrs.yaw
#define IMU_ALIGN  IMU_ALIGN_CW90 //IMU_ALIGN_CW0, IMU_ALIGN_CW90, IMU_ALIGN_CW180, IMU_ALIGN_CW270, IMU_ALIGN_CW0FLIP, IMU_ALIGN_CW90FLIP, IMU_ALIGN_CW180FLIP, IMU_ALIGN_CW270FLIP
#define IMU_I2C_ADR  0x69 //IMU I2C address. If unknown, use CLI 'i2c'

//========================================================================================================================//
//                                               RC RECEIVER CONFIG                                                       //
//========================================================================================================================//

//set channels
const int rcin_cfg_thro_channel  = 1; //low pwm = zero throttle/stick back, high pwm = full throttle/stick forward
const int rcin_cfg_roll_channel  = 2; //low pwm = left, high pwm = right
const int rcin_cfg_pitch_channel = 3; //low pwm = pitch up/stick back, high pwm = pitch down/stick forward
const int rcin_cfg_yaw_channel   = 4; //low pwm = left, high pwm = right
const int rcin_cfg_arm_channel   = 5; //ARM/DISARM switch

//throttle pwm values
const int rcin_cfg_thro_low      = 1250; //used to set rcin_thro_is_low flag when pwm is below. Note: your craft won't arm if this is too low.
const int rcin_cfg_thro_max      = 1900;
const float out_armed_speed      = 0.2; //Safety feature: make props spin when armed, the motors spin at this speed when armed and throttle is low. The default 0.2 is probably fairly high, set lower as needed.

//roll, pitch, yaw pwm values
const int rcin_cfg_pwm_min       = 1150;
const int rcin_cfg_pwm_center    = 1500;
const int rcin_cfg_pwm_max       = 1900;
const int rcin_cfg_pwm_deadband  = 0; //Amount of deadband around center, center-deadband to center+deadband will be interpreted as central stick. Set to 15 for PPM or 0 for jitter-free serial protocol receivers.

//pwm range for arm switch in ARMED position
const int rcin_cfg_arm_min       = 1600;
const int rcin_cfg_arm_max       = 2200;

//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //
//========================================================================================================================//

//number of motors - out[0..out_MOTOR_COUNT-1] are motors, out[out_MOTOR_COUNT..HW_OUT_COUNT-1] are servos
const int out_MOTOR_COUNT = 4;
//name the outputs, to make code more readable
enum out_enum {MOTOR1, MOTOR2, MOTOR3, MOTOR4}; 

const uint32_t imu_sample_rate = 1000; //imu sample rate in Hz (default 1000) NOTE: not all IMU drivers support a different rate

//Low Pass Filter cutoff frequency in Hz. Do not touch unless you know what you are doing.
float LP_acc         = 70;        //Accelerometer  (default MPU6050: 50Hz, MPU9250: 70Hz)
float LP_gyr         = 60;        //Gyro           (default MPU6050: 35Hz, MPU9250: 60Hz)
float LP_mag         = 1e10;      //Magnetometer   (default 1e10Hz, i.e. no filtering)
float LP_radio       = 400;       //Radio Input    (default 400Hz)

//Controller parameters (take note of defaults before modifying!): 
float i_limit        = 25.0;      //Integrator saturation level, mostly for safety (default 25.0)
float maxRoll        = 30.0;      //Max roll angle in degrees for angle mode (maximum ~70 degrees)
float maxPitch       = 30.0;      //Max pitch angle in degrees for angle mode (maximum ~70 degrees)
float maxRollRate    = 30.0;      //Max roll rate in deg/sec for rate mode 
float maxPitchRate   = 30.0;      //Max pitch rate in deg/sec for rate mode
float maxYawRate     = 160.0;     //Max yaw rate in deg/sec for angle and rate mode

float Kp_ro_pi_angle  = 0.2;      //Roll/Pitch P-gain - angle mode 
float Ki_ro_pi_angle  = 0.1;      //Roll/Pitch I-gain - angle mode
float Kd_ro_pi_angle  = 0.05;     //Roll/Pitch D-gain - angle mode (has no effect on control_Angle2)
float B_loop_ro_pi    = 0.9;      //Roll/Pitch damping term for control_Angle2(), lower is more damping (must be between 0 to 1)

float Kp_ro_pi_rate   = 0.15;     //Roll/Pitch P-gain - rate mode
float Ki_ro_pi_rate   = 0.2;      //Roll/Pitch I-gain - rate mode
float Kd_ro_pi_rate   = 0.0002;   //Roll/Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

float Kp_yaw          = 0.3;       //Yaw P-gain
float Ki_yaw          = 0.05;      //Yaw I-gain
float Kd_yaw          = 0.00015;   //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)

//========================================================================================================================//
//                              DECLARE GLOBAL VARIABLES                                                                  //
//========================================================================================================================//

//Radio communication:
int rcin_pwm[RCIN_NUM_CHANNELS]; //filtered raw PWM values
float rcin_thro, rcin_roll, rcin_pitch, rcin_yaw; //rcin_thro 0(cutoff) to 1(full); rcin_roll, rcin_pitch, rcin_yaw -1(left,down) to 1(right,up) with 0 center stick
bool rcin_armed; //status of arm switch, true = armed
bool rcin_thro_is_low; //status of throttle stick, true = throttle low
int rcin_aux; // six position switch connected to aux channel, values 0-5

//Controller:
float roll_PID = 0, pitch_PID = 0, yaw_PID = 0;

//Flight status
bool out_armed = false; //motors will only run if this flag is true

//Low pass filter parameters
float B_radio;

//========================================================================================================================//
//                                                 INCLUDE MADFLIGHT LIBRARY                                              //
//========================================================================================================================//
//Note: most madflight modules are header only. By placing the madflight include here allows the modules to access the global variables without declaring them extern.
#include <madflight.h>

//========================================================================================================================//
//                                                       SETUP()                                                          //
//========================================================================================================================//

void setup() {
  led.setup(HW_PIN_LED, HW_LED_ON); //Set built in LED to turn on to signal startup
  Serial.begin(115200); //start console serial

  //6 second startup delay
  for(int i=20;i>0;i--) { 
    Serial.printf(MADFLIGHT_VERSION " on " HW_ARDUINO_STR " starting %d ...\n",i);
    delay(300);
    led.toggle();
  } 
  led.on();

  hw_setup(); //hardware specific setup for spi and Wire (see hw_xxx.h)
  cfg.begin(); //read config from EEPROM
  cli.print_boardInfo(); //print board info and pinout
  cli.print_i2cScan(); //print i2c scan
  rcin.setup(); //Initialize radio communication. Set correct USE_RCIN_xxx user specified defines above. Note: rcin_Setup() function is defined in rcin.h, but normally no changes needed there.

  //IMU: keep on trying until no error is returned (some sensors need a couple of tries...)
  while(true) {
    int rv = imu.setup(imu_sample_rate); //request 1000 Hz sample rate, returns 0 on success, positive on error, negative on warning
    if(rv<=0) break;
    warn("IMU: init failed rv= " + String(rv) + ". Retrying...\n");
  }

  //set filter parameters after imu.setup(), as imu.setup() can modify requested sample rate
  B_radio = Ahrs::lowpass_to_beta(LP_radio, imu.getSampleRate()); //Note: uses imu sample rate because radio filter is applied in imu_loop

  //Motors
  for(int i=0;i<out_MOTOR_COUNT;i++) {
    //uncomment one line - sets pin, frequency (Hz), minimum (us), maximum (us)
    out[i].begin(HW_PIN_OUT[i], 400, 950, 2000); //Standard PWM: 400Hz, 950-2000 us
    //out[i].begin(HW_PIN_OUT[i], 2000, 125, 250); //Oneshot125: 2000Hz, 125-250 us

    out_command[i] = 0; //set output to 0 for motors
    out[i].writeFactor(out_command[i]); //start the PWM output to the motors
  }

  ahrs.setup(LP_gyr, LP_acc, LP_mag); //setup low pass filters for Mahony/Madgwick filters
  ahrs.setInitalOrientation(); //do this before IMU update handler is started

  //start IMU update handler
  imu.onUpdate = imu_loop;
  if(!imu.waitNewSample()) die("IMU interrupt not firing. Is IMU data ready interrupt pin IMU_EXTI connected?");

  //Calibrate for zero gyro readings, assuming vehicle not moving when powered up. Comment out to only use cfg values. (Use CLI to calibrate accelerometer and magnetometer.)
  cli.calibrate_gyro();

  cli.welcome();

  led.off(); //Set built in LED off to signal end of startup
}

//========================================================================================================================//
//                                                            LOOP()                                                      //
//========================================================================================================================//

void loop() {
  cli.loop(); //process CLI commands
}

//========================================================================================================================//
//                                                   IMU UPDATE LOOP                                                      //
//========================================================================================================================//

//This is __MAIN__ function of this program. It is called when new IMU data is available.
void imu_loop() {
  //Blink LED
  led_Blink();

  //Sensor fusion: update ahrs.roll, ahrs.pitch, and ahrs.yaw angle estimates (degrees) from IMU data
  ahrs.update(); 

  //Get radio state
  rcin_GetCommands(); //Pulls current available radio commands
  rcin_Normalize(); //Convert raw commands to normalized values based on saturated control limits

  //Uncomment to debug without remote (and no battery!) - pitch drone up: motors m1,m3 should increase and m2,m4 decrease; bank right: m1,m2 increase; yaw right: m1,m4 increase
  //rcin_thro = 0.5; rcin_thro_is_low = false; rcin_roll = 0; rcin_pitch = 0; rcin_yaw = 0; rcin_armed = true; rcin_aux = 0; out_armed = true;

  //PID Controller RATE or ANGLE - SELECT ONE:
  control_Rate(rcin_thro_is_low); //Stabilize on rate setpoint
  //control_Angle(rcin_thro_is_low); //Stabilize on pitch/roll angle setpoint, stabilize yaw on rate setpoint  //control_Angle2(rcin_thro_is_low); //Stabilize on pitch/roll setpoint using cascaded method. Rate controller must be tuned well first!

  //Actuator mixing
  control_Mixer(); //Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here

  //Motor output
  out_KillSwitchAndFailsafe(); //Cut all motor outputs if DISARMED or failsafe triggered.
  out_SetCommands(); //Sends command pulses to motors (only if out_armed=true) and servos
}

//========================================================================================================================//
//                      IMU UPDATE LOOP FUNCTIONS - in same order as they are called from imu_loop()                           //
//========================================================================================================================//

void led_Blink() {
  //Blink LED once per second, if LED blinks slower then the loop takes too much time, use print_loop_Rate() to investigate.
  //DISARMED: long off, short on, ARMED: long on, short off
  if(imu.update_cnt % imu.getSampleRate() <= imu.getSampleRate() / 10)
    led.set(!out_armed); //short interval
  else
    led.set(out_armed); //long interval
}

void rcin_GetCommands() {
  //DESCRIPTION: Get raw PWM values for every channel from the radio
  /*
   * Updates radio PWM commands in loop based on current available commands. channel_x_pwm is the raw command used in the rest of 
   * the loop. If using a PWM or PPM receiver, the radio commands are retrieved from a function in the readPWM file separate from this one which 
   * is running a bunch of interrupts to continuously update the radio readings. If using an SBUS receiver, the values are pulled from the SBUS library directly.
   * The raw radio commands are filtered with a first order low-pass filter to eliminate any really high frequency noise. 
   */

  bool got_new_data = rcin.update();

  //Low-pass the critical commands and update previous values
  for(int i=0; i<RCIN_NUM_CHANNELS; i++) {
    rcin_pwm[i] = lrintf( (1.0 - B_radio)*rcin_pwm[i] + B_radio*rcin.pwm[i] );
  }

  if(got_new_data) {
    //Serial.print("rcin_GetCommands() "); for(int i=0; i<RCIN_NUM_CHANNELS; i++) Serial.printf("CH%d:%d->%d ",i+1,pwm_new[i],rcin_pwm[i]); Serial.println(); //uncomment for debugging
  }
}

void rcin_Normalize() {
  //DESCRIPTION: Normalizes desired control values to appropriate values
  /*
   * Updates the normalized rcin variables rcin_thro, rcin_roll, rcin_pitch, and rcin_yaw. These are computed by using the raw
   * RC pwm commands and scaling them to be within our limits defined in setup. rcin_thro stays within 0 to 1 range.
   * rcin_roll, rcin_pitch, and rcin_yaw are scaled to -1 to +1.
   */

  //normalized values
  //throttle: 0.0 in range from stick full back to rcin_cfg_thro_low, 1.0 on full throttle
  int pwm = rcin_pwm[rcin_cfg_thro_channel-1];
  rcin_thro = constrain( ((float)(pwm - rcin_cfg_thro_low)) / (rcin_cfg_thro_max - rcin_cfg_thro_low), 0.0, 1.0);
  rcin_thro_is_low = (pwm <= rcin_cfg_thro_low); 

  //roll,pitch,yaw
  rcin_roll = _rcin_ChannelNormalize(rcin_pwm[rcin_cfg_roll_channel-1], rcin_cfg_pwm_min, rcin_cfg_pwm_center, rcin_cfg_pwm_max, rcin_cfg_pwm_deadband); // output: -1 (roll left, stick left) to 1 (roll right, stick right)
  rcin_pitch = - _rcin_ChannelNormalize(rcin_pwm[rcin_cfg_pitch_channel-1], rcin_cfg_pwm_min, rcin_cfg_pwm_center, rcin_cfg_pwm_max, rcin_cfg_pwm_deadband); // output: -1 (pitch down, stick back) to 1 (pitch up, stick forward)
  rcin_yaw = _rcin_ChannelNormalize(rcin_pwm[rcin_cfg_yaw_channel-1], rcin_cfg_pwm_min, rcin_cfg_pwm_center, rcin_cfg_pwm_max, rcin_cfg_pwm_deadband); //  output: -1 (yaw left, stick left) to 1 (yaw right, stick right)

  //arm switch
  pwm = rcin_pwm[rcin_cfg_arm_channel-1];
  rcin_armed = (rcin_cfg_arm_min <= pwm && pwm <= rcin_cfg_arm_max);
}

//helper to nomalize a channel based on min,center,max calibration
float _rcin_ChannelNormalize(int val, int min, int center, int max, int deadband) {
  int rev = 1; //1=normal, -1=reverse channel
  //needs: min < center < max
  if(val<min) return rev * -1.0;
  if(val<center-deadband) return (float)(rev * (val-(center-deadband))) / ((center-deadband)-min); //returns -1 to 0
  if(val<center+deadband) return 0;
  if(val<max) return (float)(rev * (val-(center+deadband))) / (max-(center+deadband)); 
  return rev * 1.0;
}

void control_Angle(bool zero_integrators) {
  //DESCRIPTION: Computes control commands based on state error (angle)
  /*
   * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in 
   * rcin_Normalize(). Error is simply the desired state minus the actual state (ex. roll_des - ahrs.roll). Two safety features
   * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
   * terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
   * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in control_Mixer().
   */ 

  //inputs: roll_des, pitch_des, yawRate_des
  //outputs: roll_PID, pitch_PID, yaw_PID

  //desired values
  float roll_des = rcin_roll * maxRoll; //Between -maxRoll and +maxRoll
  float pitch_des = rcin_pitch * maxPitch; //Between -maxPitch and +maxPitch
  float yawRate_des = rcin_yaw * maxYawRate; //Between -maxYawRate and +maxYawRate

  //state vars
  static float integral_roll, integral_pitch, error_yaw_prev, integral_yaw;

  //Zero the integrators (used to don't let integrator build if throttle is too low, or to re-start the controller)
  if(zero_integrators) {
    integral_roll = 0;
    integral_pitch = 0;
    integral_yaw = 0;
  }

  //Roll PID
  float error_roll = roll_des - ahrs.roll;
  integral_roll += error_roll * imu.dt;
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_roll = ahrs.gx;
  roll_PID = 0.01 * (Kp_ro_pi_angle*error_roll + Ki_ro_pi_angle*integral_roll - Kd_ro_pi_angle*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch PID
  float error_pitch = pitch_des - ahrs.pitch;
  integral_pitch += error_pitch * imu.dt;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_pitch = ahrs.gy; 
  pitch_PID = 0.01 * (Kp_ro_pi_angle*error_pitch + Ki_ro_pi_angle*integral_pitch - Kd_ro_pi_angle*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw PID, stablize on rate from GyroZ - TODO: use compass heading, not gyro rate
  float error_yaw = yawRate_des - ahrs.gz;
  integral_yaw += error_yaw * imu.dt;
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_yaw = (error_yaw - error_yaw_prev) / imu.dt; 
  yaw_PID = 0.01 * (Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update derivative variables
  error_yaw_prev = error_yaw;
}

void control_Rate(bool zero_integrators) {
  //DESCRIPTION: Computes control commands based on state error (rate)
  /*
   * See explanation for control_Angle(). Everything is the same here except the error is now the desired rate - raw gyro reading.
   */

  //inputs: roll_des, pitch_des, yawRate_des
  //outputs: roll_PID, pitch_PID, yaw_PID

  //desired values
  float rollRate_des = rcin_roll * maxRollRate; //Between -maxRoll and +maxRoll
  float pitchRate_des = rcin_pitch * maxPitchRate; //Between -maxPitch and +maxPitch
  float yawRate_des = rcin_yaw * maxYawRate; //Between -maxYawRate and +maxYawRate 
  
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
  float error_roll = rollRate_des - ahrs.gx;
  integral_roll += error_roll * imu.dt;
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_roll = (error_roll - error_roll_prev) / imu.dt;
  roll_PID = 0.01 * (Kp_ro_pi_rate*error_roll + Ki_ro_pi_rate*integral_roll + Kd_ro_pi_rate*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  float error_pitch = pitchRate_des - ahrs.gy;
  integral_pitch += error_pitch * imu.dt;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_pitch = (error_pitch - error_pitch_prev) / imu.dt;   
  pitch_PID = 0.01 * (Kp_ro_pi_rate*error_pitch + Ki_ro_pi_rate*integral_pitch + Kd_ro_pi_rate*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  float error_yaw = yawRate_des - ahrs.gz;
  integral_yaw += error_yaw * imu.dt;
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_yaw = (error_yaw - error_yaw_prev) / imu.dt; 
  yaw_PID = 0.01 * (Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update derivative variables
  error_roll_prev = error_roll;
  error_pitch_prev = error_pitch;
  error_yaw_prev = error_yaw;
}

void control_Mixer() {
  //DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
  /*
   * Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and appropriately mixes them for the desired
   * vehicle configuration. For example on a quadcopter, the left two motors should have +roll_PID while the right two motors
   * should have -roll_PID. Front two should have +pitch_PID and the back two should have -pitch_PID etc... every motor has
   * normalized (0 to 1) rcin_thro command for throttle control. Can also apply direct unstabilized commands from the transmitter with 
   * rcin_xxx variables are to be sent to the motor ESCs and servos.
   * 
   *Relevant variables:
   *rcin_thro - direct thottle control
   *roll_PID, pitch_PID, yaw_PID - stabilized axis variables
   *rcin_roll, rcin_pitch, rcin_yaw - direct unstabilized command passthrough
   *rcin_aux - free auxillary channel, can be used to toggle things with an 'if' statement
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

  //Quad mixing
  out_command[MOTOR1] = rcin_thro - pitch_PID - roll_PID - yaw_PID; //Back Right CW
  out_command[MOTOR2] = rcin_thro + pitch_PID - roll_PID + yaw_PID; //Front Right CCW
  out_command[MOTOR3] = rcin_thro - pitch_PID + roll_PID + yaw_PID; //Back Left CCW
  out_command[MOTOR4] = rcin_thro + pitch_PID + roll_PID - yaw_PID; //Front Left CW
}

void out_KillSwitchAndFailsafe() {
  static bool rcin_armed_prev = true; //initial value is true: forces out_armed false on startup even if arm switch is ON

  //Change to ARMED when throttle is low and radio armed switch was flipped from disamed to armed position
  if (!out_armed && rcin_thro_is_low && rcin_armed && !rcin_armed_prev) {
    out_armed = true;
    Serial.println("OUT: ARMED");
  }

  //Change to DISARMED when radio armed switch is in disarmed position, or if radio lost connection
   if (out_armed && (!rcin_armed || !rcin.connected())) {
    out_armed = false;
    if(!rcin_armed) {
      Serial.println("OUT: DISARMED (arm switch)");
    }else{
      Serial.println("OUT: DISARMED (rcin lost connection)");
    }
  }

  //If armed and throttle is low -> set motor outputs to out_armed_speed
  if(out_armed && rcin_thro_is_low) for(int i=0;i<out_MOTOR_COUNT;i++) out_command[i] = out_armed_speed; 

  //IF DISARMED -> STOP MOTORS
  if(!out_armed) for(int i=0;i<out_MOTOR_COUNT;i++) out_command[i] = 0; 

  rcin_armed_prev = rcin_armed;
}

void out_SetCommands() {
  //DESCRIPTION: Send pulses to motor pins, oneshot125 or PWM protocol and set servos
  for(int i=0;i<HW_OUT_COUNT;i++) out[i].writeFactor( out_command[i] );
}

//===============================================================================================
// HELPERS
//===============================================================================================

void warn_or_die(String msg, bool never_return) {
  bool do_print = true;
  do{
    if(do_print) Serial.print(msg + "\n");
    for(int i=0;i<20;i++) {
      led.toggle();
      uint32_t ts = millis();
      while(millis() - ts < 50) {
        if(cli.loop()) do_print = false; //process CLI commands, stop error output after first command
      } 
    }
  } while(never_return);
}
void die(String msg) { warn_or_die(msg, true); }
void warn(String msg) { warn_or_die(msg, false); }
