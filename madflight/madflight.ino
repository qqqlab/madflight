//Arduino ESP32 / RP2040 Flight Controller
//GPL-3.0 license
//Copyright (c) 2023 https://github.com/qqqlab/madflight
//Copyright (c) 2022 Nicholas Rehm - dRehmFlight
 
/*#########################################################################################################################
How to use this quadcopter demo program 
=======================================

Motor order diagram (Betaflight order)

      front
 CW -->   <-- CCW
     4     2 
      \ ^ /
       |X|
      / - \
     3     1 
CCW -->   <-- CW

Arming: Set throttle low, then flip arm switch from DISARMED to ARMED.
Disarming: Flip arm switch from ARMED to DISARMED, at any throttle position. "Kill switch".

LED Status
==========
OFF - not powered
ON - running setup
blinking long OFF shot ON - running loop() DISARMED
blinking long ON shot OFF - running loop() ARMED
blink interval longer than 1 second - loop() is taking too much time
##########################################################################################################################*/

//========================================================================================================================//
//                                                 USER-SPECIFIED DEFINES                                                 //                                                                 
//========================================================================================================================//

//-------------------------------------
//Include hardware architecture dependent definitions
#if defined ARDUINO_ARCH_ESP32
  #include "hw_ESP32.h"
#elif defined ARDUINO_ARCH_RP2040
  #include "hw_RP2040.h"
#else 
  #error "Unknown hardware architecture"
#endif

//-------------------------------------
//Uncomment only one receiver type
#define USE_RCIN_PPM
//#define USE_RCIN_PWM 
//#define USE_RCIN_SBUS
//#define USE_RCIN_DSM
#include "src/RCIN/RCIN.h" //first define USE_RCIN_xxx then include RCIN.h

//-------------------------------------
//Uncomment only one IMU
#define USE_IMU_MPU6050_I2C  //acc/gyro
//#define USE_IMU_MPU9150_I2C  //acc/gyro/mag
//#define USE_IMU_MPU6500_I2C  //acc/gyro      Note: SPI interface is faster
//#define USE_IMU_MPU9250_I2C  //acc/gyro/mag  Note: SPI interface is faster
//#define USE_IMU_MPU6500_SPI  //acc/gyro
//#define USE_IMU_MPU9250_SPI    //acc/gyro/mag

//Uncomment one I2C address. If unknown, see output of print_i2c_scan()
//#define IMU_I2C_ADR 0x68 //MPU9250
#define IMU_I2C_ADR 0x69 //MPU9150

//Full scale gyro range in deg/sec. Most IMUs support 250,500,1000,2000. Can use any value here, driver will pick next greater setting.
#define IMU_GYRO_DPS 500

//Full scale gyro accelerometer in G's. Most IMUs support 2,4,8,16. Can use any value here, driver will pick next greater setting.
#define IMU_ACCEL_G 2

//Uncomment only one sensor orientation. The labels is yaw / roll (in that order) needed to rotate the sensor from it's normal position to it's mounted position.
//if not sure what is needed: try each setting until roll-right gives positive ahrs_roll, pitch-up gives positive ahrs_pitch, and yaw-right gives increasing ahrs_yaw
//#define IMU_ROTATE_NONE
#define IMU_ROTATE_YAW90
//#define IMU_ROTATE_YAW180
//#define IMU_ROTATE_YAW270
//#define IMU_ROTATE_ROLL180
//#define IMU_ROTATE_YAW90_ROLL180
//#define IMU_ROTATE_YAW180_ROLL180
//#define IMU_ROTATE_YAW270_ROLL180
#include "src/sensor/IMU.h" //first define IMU_xxx then include IMO.h

//========================================================================================================================//
//                                               RC RECEIVER CONFIG                                                      //
//========================================================================================================================//

const int rcin_pwm_fs[] = {1000,1500,1500,1500,1000,1000}; //failsafe pwm values

//set channels
const int rcin_channels = 6;
const int rcin_cfg_thro_channel  = 1; //low pwm = zero throttle/stick back, high pwm = full throttle/stick forward
const int rcin_cfg_roll_channel  = 2; //low pwm = left, high pwm = right
const int rcin_cfg_pitch_channel = 3; //low pwm = pitch up/stick back, high pwm = pitch down/stick forward
const int rcin_cfg_yaw_channel   = 4; //low pwm = left, high pwm = right
const int rcin_cfg_arm_channel   = 5; //ARM/DISARM switch
const int rcin_cfg_aux_channel   = 6; //Fight mode - 6 position switch

//config throttle pwm values
const int rcin_cfg_thro_min      = 1150;
const int rcin_cfg_thro_low      = 1250; //used to set rcin_thro_is_low flag when pwm is below. Note: your craft won't arm if this is too low
const int rcin_cfg_thro_max      = 1900;

//config roll, pitch, yaw pwm values
const int rcin_cfg_pwm_min       = 1150;
const int rcin_cfg_pwm_center    = 1500;
const int rcin_cfg_pwm_max       = 1900;
const int rcin_cfg_pwm_deadband  = 15; //Amount of deadband around center, center-deadband to center+deadband will be interpreted as central stick. Set to 0 for jitter-free serial protocol receivers.

//config pwm range for ARMED on arm channel
const int rcin_cfg_arm_min       = 1600;
const int rcin_cfg_arm_max       = 2200;

//config 6 position switch on aux channel
int rcin_cfg_aux_min = 1115; //lowest switch position
int rcin_cfg_aux_max = 1945; //higest switch position

//========================================================================================================================//
//                                               CALIBRATION PARAMETERS                                                   //
//========================================================================================================================//

//Magnetometer calibration parameters - if using MPU9250, uncomment calibrate_Magnetometer() in void setup() to get these values, else just ignore these
float MagErrorX = 0.0;
float MagErrorY = 0.0; 
float MagErrorZ = 0.0;
float MagScaleX = 1.0;
float MagScaleY = 1.0;
float MagScaleZ = 1.0;

//IMU calibration parameters - calibrate IMU using calibrate_IMU_error() in the void setup() to get these values, then comment out calibrate_IMU_error()
float AccErrorX = 0.0;
float AccErrorY = 0.0;
float AccErrorZ = 0.0;
float GyroErrorX = 0.0;
float GyroErrorY = 0.0;
float GyroErrorZ = 0.0;

//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //                           
//========================================================================================================================//

//number of motors - out[0..out_MOTOR_COUNT-1] are motors, out[out_MOTOR_COUNT..hw_OUT_COUNT-1] are servos
const int out_MOTOR_COUNT = 4;
//name the outputs, to make code more readable
enum out_enum {MOTOR1,MOTOR2,MOTOR3,MOTOR4,SERVO1,SERVO2,SERVO3,SERVO4,SERVO5,SERVO6,SERVO7,SERVO8,SERVO9,SERVO10,SERVO11,SERVO12}; 

uint32_t loop_freq = 2000; //Loop frequency in Hz. Do not change, all filter parameters tuned to 2000Hz by default

//Low Pass Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
float B_madgwick = 0.041;     //ahrs_Madgwick filter parameter
float B_accel = 0.2;          //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
float B_gyro = 0.17;          //Gyro LP filter parameter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
float B_mag = 1.0;            //Magnetometer LP filter parameter
float B_radio = 0.7;          //Radio input filter parameter. Lower=slower, higher=noiser

//Controller parameters (take note of defaults before modifying!): 
float i_limit = 25.0;         //Integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 30.0;         //Max roll angle in degrees for angle mode (maximum ~70 degrees)
float maxPitch = 30.0;        //Max pitch angle in degrees for angle mode (maximum ~70 degrees)
float maxRollRate = 30.0;     //Max roll rate in deg/sec for rate mode 
float maxPitchRate = 30.0;    //Max pitch rate in deg/sec for rate mode
float maxYawRate = 160.0;     //Max yaw rate in deg/sec for angle and rate mode

float Kp_roll_angle = 0.2;    //Roll P-gain - angle mode 
float Ki_roll_angle = 0.3;    //Roll I-gain - angle mode
float Kd_roll_angle = 0.05;   //Roll D-gain - angle mode (has no effect on control_Angle2)
float B_loop_roll = 0.9;      //Roll damping term for control_Angle2(), lower is more damping (must be between 0 to 1)
float Kp_pitch_angle = 0.2;   //Pitch P-gain - angle mode
float Ki_pitch_angle = 0.3;   //Pitch I-gain - angle mode
float Kd_pitch_angle = 0.05;  //Pitch D-gain - angle mode (has no effect on control_Angle2)
float B_loop_pitch = 0.9;     //Pitch damping term for control_Angle2(), lower is more damping (must be between 0 to 1)

float Kp_roll_rate = 0.15;    //Roll P-gain - rate mode
float Ki_roll_rate = 0.2;     //Roll I-gain - rate mode
float Kd_roll_rate = 0.0002;  //Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
float Kp_pitch_rate = 0.15;   //Pitch P-gain - rate mode
float Ki_pitch_rate = 0.2;    //Pitch I-gain - rate mode
float Kd_pitch_rate = 0.0002; //Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

float Kp_yaw = 0.3;           //Yaw P-gain
float Ki_yaw = 0.05;          //Yaw I-gain
float Kd_yaw = 0.00015;       //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)

//========================================================================================================================//
//                              DECLARE GLOBAL VARIABLES
//========================================================================================================================//

//General stuff
float loop_dt;
uint32_t loop_time, loop_cnt=0;
uint32_t print_time;
bool print_need_newline;
uint32_t loop_rt, loop_rt_imu; //runtime of loop and imu sensor retrieval

//Radio communication:
int rcin_pwm[16]; //raw PWM values
float rcin_thro, rcin_roll, rcin_pitch, rcin_yaw; //rcin_thro 0(cutoff) to 1(full); rcin_roll, rcin_pitch, rcin_yaw -1(left,down) to 1(right,up) with 0 center stick
bool rcin_armed; //status of arm switch, true = armed
bool rcin_thro_is_low; //status of throttle stick, true = throttle low
int rcin_aux; // six position switch connected to aux channel, values 0-5

//IMU:
float AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ;
float ahrs_roll, ahrs_pitch, ahrs_yaw;  //ahrs_Madgwick() estimate output in degrees. Positive angles are: roll right, yaw right, pitch up
float q0 = 1.0f; //Initialize quaternion for madgwick filter (shared between ahrs_Madgwick6DOF and ahrs_Madgwick9DOF)
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

//Controller:
float roll_PID = 0, pitch_PID = 0, yaw_PID = 0;

//Mixer output (motor and servo values are scaled 0.0 to 1.0)
//Outputs:
float out_command[hw_OUT_COUNT] = {0}; //Mixer outputs
PWM out[hw_OUT_COUNT]; //ESC and Servo outputs

//Flight status
bool out_armed = false; //motors will only run if this flag is true

//converson
const float rad_to_deg = 57.29577951; //radians to degrees conversion constant

//========================================================================================================================//
//                                          SETUP1() LOOP1() EXECUTING ON SECOND CORE                                     //
//========================================================================================================================//
//Uncomment setup1() and/or loop1() to use the second core on ESP32 / RP2040
/*
void setup1() {
  Serial.println("setup1()");
}

void loop1() {
  Serial.println("loop1()"); delay(100);
}
//*/

//========================================================================================================================//
//                                                       SETUP()                                                          //
//========================================================================================================================//

void setup() {
  //Set built in LED to turn on to signal startup
  pinMode(led_PIN, OUTPUT);
  digitalWrite(led_PIN, HIGH);

  //debug serial
  Serial.begin(115200);
  for(int i=6;i>0;i--) {
    Serial.printf("madflight starting %d ...\n",i);
    delay(500); //delay to get Arduino debugger connected
  }

  //hardware specific setup for spi and Wire (see hw.h)
  hw_setup();

  //debug i2c
  print_i2c_scan();
  
  //Initialize radio communication. Set correct USE_RCIN_xxx user specified defines above. (function defined in rcin.h, but normally changes needed there.)
  rcin_Setup();
  //calibrateRadio();   //generates the rcin_cal_ calibration constants
  //Set radio channels to default (safe) values before entering main loop
  for(int i=0;i<rcin_channels;i++) rcin_pwm[i] = rcin_pwm_fs[i];

  //Initialize IMU communication
  for(int i=0;i<10;i++) {
    int rv = imu_Setup();
    if(rv==0) break;
    Serial.printf("IMU init failed rv=%d. Retrying...\n", rv);
    delay(500);
  }

  //Init Motors & servos
  for(int i=0;i<out_MOTOR_COUNT;i++) {
    //uncomment one line - sets pin, frequency (Hz), minimum (us), maximum (us)
    out[i].begin(out_PIN[i], 400, 900, 2000); //Standard PWM: 400Hz, 900-2000 us
    //out[i].begin(out_PIN[i], 2000, 125, 250); //Oneshot125: 2000Hz, 125-250 us
  }
  for(int i=out_MOTOR_COUNT;i<hw_OUT_COUNT;i++) {
    out[i].begin(out_PIN[i], 50, 1000, 2000); //Standard servo at 50Hz
  }
  //Arm motors
  for(int i=0;i<out_MOTOR_COUNT;i++) {
    out_command[i] = 0;
    out[i].writeFactor(out_command[i]); //start the PWM output to the motors
  }
  //Arm servo channels
  for(int i=out_MOTOR_COUNT;i<hw_OUT_COUNT;i++) {
    out_command[i] = 0; //keep at 0 if you are using servo outputs for motors
    out[i].writeFactor(out_command[i]); //start the PWM output to the servos
  } 

  //Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powered up
  //calibrate_IMU_error(); //Calibration parameters printed to serial monitor. Paste these in the user specified variables section, then comment this out. (Or.. keep this active to calibrate on every startup.)

  //If using IMU with magnetometer, uncomment for one-time magnetometer calibration (may need to repeat for new locations)
  //calibrate_Magnetometer(); //Generates magentometer error and scale factors to be pasted in user-specified variables section

  //calibrate_ESCs(); //PROPS OFF. Uncomment this to calibrate your ESCs by setting throttle stick to max, powering on, and lowering throttle to zero after the beeps
  //Code will not proceed past here if this function is uncommented!

  //set quarterion to initial yaw, so that madgwick settles faster
  ahrs_Setup();

  //set times for loop
  loop_RateBegin();

  //Set built in LED off to signal end of startup
  digitalWrite(led_PIN, LOW);
}



//========================================================================================================================//
//                                                            LOOP()                                                      //
//========================================================================================================================//
void loop() {
  //debugging: number of times imu took too long
  static int imu_err_cnt = 0;
  if(loop_rt_imu > 500) imu_err_cnt++;

  //Keeps loop sample rate constant. Keeps track of loop_time (starting time of loop), and loop_dt (elapsed time since the last loop).
  loop_Rate();

  //Blink LED
  loop_Blink();

  //Debugging - Print data at 50hz, uncomment line(s) for troubleshooting
  if (loop_time - print_time > 20000) {
    print_time = micros();
    print_need_newline = false;
    //Serial.printf("loop_time:%d\t",loop_time); //print loop time stamp
    print_overview(); //prints: pwm1, rcin_roll, gyroX, accX, magX, ahrs_roll, pid_roll, motor1, loop_rt
    //print_rcin_RadioPWM();     //Prints radio pwm values (expected: 1000 to 2000)
    //print_rcin_RadioScaled();     //Prints scaled radio values (expected: -1 to 1)    
    //print_imu_GyroData();      //Prints filtered gyro data direct from IMU (expected: -250 to 250, 0 at rest)
    //print_imu_AccData();     //Prints filtered accelerometer data direct from IMU (expected: -2 to 2; x,y 0 when level, z 1 when level)
    //print_imu_MagData();       //Prints filtered magnetometer data direct from IMU (expected: -300 to 300)
    //print_ahrs_RollPitchYaw();  //Prints roll, pitch, and yaw angles in degrees from ahrs_Madgwick filter (expected: degrees, 0 when level)
    //print_control_PIDoutput();     //Prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1 to 1)
    //print_out_MotorCommands(); //Prints the values being written to the motors (expected: 0 to 1)
    //print_out_ServoCommands(); //Prints the values being written to the servos (expected: 0 to 1)
    //print_loop_Rate();      //Prints the time between loops in microseconds (expected: 1000000 / loop_freq)
    Serial.printf("imu_err_cnt:%d\t",imu_err_cnt); //prints number of times imu update took too long
    if(print_need_newline) Serial.println();
  }

  //Get vehicle state
  imu_GetData(); //Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
  ahrs_Madgwick(); //Updates ahrs_roll, ahrs_pitch, and ahrs_yaw angle estimates (degrees)
 
  //Get radio state
  rcin_GetCommands(); //Pulls current available radio commands
  rcin_Failsafe(); //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup
  rcin_Normalize(); //Convert raw commands to normalized values based on saturated control limits

  //Uncomment to debug without remote (and no battery!) - pitch drone up: motors m1,m3 should increase and m2,m4 decrease; bank right: m1,m2 increase; yaw right: m1,m4 increase
  //rcin_thro = 0.5; rcin_thro_is_low = false; rcin_roll = 0; rcin_pitch = 0; rcin_yaw = 0; rcin_armed = true; rcin_aux = 0; out_armed = true;

  //PID Controller - SELECT ONE:
  control_Angle(rcin_thro_is_low); //Stabilize on pitch/roll angle setpoint, stabilize yaw on rate setpoint
  //control_Angle2(rcin_thro_is_low); //Stabilize on pitch/roll setpoint using cascaded method. Rate controller must be tuned well first!
  //control_Rate(rcin_thro_is_low); //Stabilize on rate setpoint

  //Actuator mixing
  control_Mixer(); //Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here

  //Command actuators
  out_KillSwitch(); //Cut all motor outputs if DISARMED.
  out_SetCommands(); //Sends command pulses to motors (only if out_armed=true) and servos
}

//========================================================================================================================//
//                        LOOP() FUNCTIONS - in same order as they are called from loop()                                 //
//========================================================================================================================//

void loop_Rate() {
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters.
   */

  loop_rt = micros() - loop_time; //record runtime of last loop
  
  //Waste time until sample time has passed.
  while ( (micros() - loop_time) < (1000000U / loop_freq) );
  uint32_t now = micros();
  loop_dt = (now - loop_time)/1000000.0;
  loop_time = now;
  loop_cnt++;
}

void loop_Blink() {
  //Blink LED once per second, if LED blinks slower then the loop takes too much time, use print_loop_Rate() to investigate.
  //DISARMED: long off, short on, ARMED: long on, short off
  if(loop_cnt % loop_freq <= loop_freq / 10) 
    digitalWrite(led_PIN,!out_armed);
  else
    digitalWrite(led_PIN,out_armed); 
}

void imu_GetData() {
  //DESCRIPTION: Request full dataset from IMU and LP filter gyro, accelerometer, and magnetometer data
  /*
   * Reads accelerometer, gyro, and magnetometer data from IMU as AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ. AK8975
   * These values are scaled according to the IMU datasheet to put them into correct units of g's, deg/sec, and uT. A simple first-order
   * low-pass filter is used to get rid of high frequency noise in these raw signals. Generally you want to cut
   * off everything past 80Hz, but if your loop rate is not fast enough, the low pass filter will cause a lag in
   * the readings. The filter parameters B_gyro and B_accel are set to be good for a 2kHz loop rate. Finally,
   * the constant errors found in calibrate_IMU_error() on startup are subtracted from the accelerometer and gyro readings.
   */

  float ax,ay,az,gx,gy,gz,mx=0,my=0,mz=0;

  uint32_t t1 = micros();
  //imu_Read() returns correctly NED oriented and correctly scaled values in g's, deg/sec, and uT (see file imu.h)
  imu_Read(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  loop_rt_imu = micros() - t1;

 //Accelerometer
  //Correct the outputs with the calculated error values
  ax = ax - AccErrorX;
  ay = ay - AccErrorY;
  az = az - AccErrorZ;
  //LP filter accelerometer data
  AccX = (1.0 - B_accel) * AccX + B_accel * ax;
  AccY = (1.0 - B_accel) * AccY + B_accel * ay;
  AccZ = (1.0 - B_accel) * AccZ + B_accel * az;

  //Gyro
  //Correct the outputs with the calculated error values
  gx = gx - GyroErrorX;
  gy = gy - GyroErrorY;
  gz = gz - GyroErrorZ;
  //LP filter gyro data
  GyroX = (1.0 - B_gyro) * GyroX + B_gyro * gx;
  GyroY = (1.0 - B_gyro) * GyroY + B_gyro * gy;
  GyroZ = (1.0 - B_gyro) * GyroZ + B_gyro * gz;

  //Magnetometer
  if(mx == 0 && my == 0 && mz == 0) {
    MagX = 0;
    MagY = 0;
    MagZ = 0;
  }else{
    //Correct the outputs with the calculated error values
    mx = (mx - MagErrorX) * MagScaleX;
    my = (my - MagErrorY) * MagScaleY;
    mz = (mz - MagErrorZ) * MagScaleZ;
    //LP filter magnetometer data
    MagX = (1.0 - B_mag) * MagX + B_mag * mx;
    MagY = (1.0 - B_mag) * MagY + B_mag * my;
    MagZ = (1.0 - B_mag) * MagZ + B_mag * mz;
  }
}

void ahrs_Madgwick() {
  //Use 6DOF algorithm if magnetometer measurement invalid or unavailable (avoids NaN in magnetometer normalisation)
  if( (MagX == 0.0f) && (MagY == 0.0f) && (MagZ == 0.0f) ) {
    _ahrs_Madgwick6DOF(GyroX, GyroY, GyroZ, AccX, AccY, AccZ, loop_dt);
  }else{
    _ahrs_Madgwick9DOF(GyroX, GyroY, GyroZ, AccX, AccY, AccZ, MagX, MagY, MagZ, loop_dt);
  }

  //compute angles - NWU
  ahrs_roll = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * rad_to_deg; //degrees
  ahrs_pitch = asin(constrain(-2.0f * (q1*q3 - q0*q2), -1.0, 1.0)) * rad_to_deg; //degrees - use constrain() to prevent NaN due to rounding
  ahrs_yaw = atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) * rad_to_deg; //degrees
}

void _ahrs_Madgwick9DOF(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 9DOF
  /*
   * This function fuses the accelerometer gyro, and magnetometer readings AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, and MagZ for attitude estimation.
   * Don't worry about the math. There is a tunable parameter B_madgwick in the user specified variable section which basically
   * adjusts the weight of gyro data in the state estimate. Higher beta leads to noisier estimate, lower 
   * beta leads to slower to respond estimate. It is currently tuned for 2kHz loop rate. This function updates the ahrs_roll,
   * ahrs_pitch, and ahrs_yaw variables which are in degrees.
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    //Normalise accelerometer measurement
    recipNorm = 1.0/sqrtf(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Normalise magnetometer measurement
    recipNorm = 1.0/sqrtf(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    //Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    //Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = 1.0/sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  //Normalize quaternion
  recipNorm = 1.0/sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void _ahrs_Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  /*
   * See description of ahrs_Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
   * available (for example when using the recommended MPU6050 IMU for the default setup).
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = 1.0/sqrtf(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = 1.0/sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  //Normalise quaternion
  recipNorm = 1.0/sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void rcin_GetCommands() {
  //DESCRIPTION: Get raw PWM values for every channel from the radio
  /*
   * Updates radio PWM commands in loop based on current available commands. channel_x_pwm is the raw command used in the rest of 
   * the loop. If using a PWM or PPM receiver, the radio commands are retrieved from a function in the readPWM file separate from this one which 
   * is running a bunch of interrupts to continuously update the radio readings. If using an SBUS receiver, the values are pulled from the SBUS library directly.
   * The raw radio commands are filtered with a first order low-pass filter to eliminate any really high frequency noise. 
   */

  int pwm_new[16];
  rcin_GetPWM(pwm_new);

  //Low-pass the critical commands and update previous values
  for(int i=0;i<rcin_channels;i++) {
    rcin_pwm[i] = (1.0 - B_radio)*rcin_pwm[i] + B_radio*pwm_new[i];
  }
}

void rcin_Failsafe() {
  //DESCRIPTION: If radio gives garbage values, set all commands to default values
  /*
   * Radio connection failsafe used to check if the getCommands() function is returning acceptable pwm values. If any of 
   * the commands are lower than 800 or higher than 2200, then we can be certain that there is an issue with the radio
   * connection (most likely hardware related). If any of the channels show this failure, then all of the radio commands 
   * channel_x_pwm are set to default failsafe values specified in the setup. Comment out this function when troubleshooting 
   * your radio connection in case any extreme values are triggering this function to overwrite the print_need_newline variables.
   */
  int minVal = 800;
  int maxVal = 2200;
  for(int i=0;i<rcin_channels;i++) {
    if (rcin_pwm[i] > maxVal || rcin_pwm[i] < minVal) {
      //If any failures, set to default failsafe values
      for(int i=0;i<rcin_channels;i++) rcin_pwm[i] = rcin_pwm_fs[i];
      return;
    }
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
  //throttle
  int pwm = rcin_pwm[rcin_cfg_thro_channel-1];
  rcin_thro = constrain( ((float)(pwm - rcin_cfg_thro_min)) / (rcin_cfg_thro_max - rcin_cfg_thro_min), 0.0, 1.0); // output: 0 (throttle cutoff, stick back) to 1 (full throttle, stick forward),
  rcin_thro_is_low = (pwm <= rcin_cfg_thro_low); 

  //roll,pitch,yaw
  rcin_roll = _rcin_ChannelNormalize(rcin_pwm[rcin_cfg_roll_channel-1], rcin_cfg_pwm_min, rcin_cfg_pwm_center, rcin_cfg_pwm_max, rcin_cfg_pwm_deadband); // output: -1 (roll left, stick left) to 1 (roll right, stick right)
  rcin_pitch = - _rcin_ChannelNormalize(rcin_pwm[rcin_cfg_pitch_channel-1], rcin_cfg_pwm_min, rcin_cfg_pwm_center, rcin_cfg_pwm_max, rcin_cfg_pwm_deadband); // output: -1 (pitch down, stick back) to 1 (pitch up, stick forward)
  rcin_yaw = _rcin_ChannelNormalize(rcin_pwm[rcin_cfg_yaw_channel-1], rcin_cfg_pwm_min, rcin_cfg_pwm_center, rcin_cfg_pwm_max, rcin_cfg_pwm_deadband); //  output: -1 (yaw left, stick left) to 1 (yaw right, stick right)

  //arm switch
  pwm = rcin_pwm[rcin_cfg_arm_channel-1];
  rcin_armed = (rcin_cfg_arm_min <= pwm && pwm <= rcin_cfg_arm_max);

  //auc 6 position switch
  int spacing = (rcin_cfg_aux_max - rcin_cfg_aux_min) / 5;
  rcin_aux = ( rcin_pwm[rcin_cfg_aux_channel-1] - rcin_cfg_aux_min + spacing/2) / spacing; //output 0,1,2,3,4,5
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
   * rcin_Normalize(). Error is simply the desired state minus the actual state (ex. roll_des - ahrs_roll). Two safety features
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

  //Serial.printf("r_des:%+.2f p_des:%+.2f y_des:%+.2f",roll_des,pitch_des,yawRate_des);

  //state vars
  static float integral_roll, integral_pitch, error_yaw_prev, integral_yaw;

  //Zero the integrators (used to don't let integrator build if throttle is too low, or to re-start the controller)
  if(zero_integrators) {
    integral_roll = 0;
    integral_pitch = 0;
    integral_yaw = 0;
  }

  //Roll PID
  float error_roll = roll_des - ahrs_roll;
  integral_roll += error_roll * loop_dt;
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_roll = GyroX;
  roll_PID = 0.01 * (Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch PID
  float error_pitch = pitch_des - ahrs_pitch;
  integral_pitch += error_pitch * loop_dt;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_pitch = GyroY; 
  pitch_PID = 0.01 * (Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw PID, stablize on rate from GyroZ
  float error_yaw = yawRate_des - GyroZ;
  integral_yaw += error_yaw * loop_dt;
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_yaw = (error_yaw - error_yaw_prev) / loop_dt; 
  yaw_PID = 0.01 * (Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update derivative variables
  error_yaw_prev = error_yaw;
}

void control_Angle2(bool zero_integrators) {
  //DESCRIPTION: Computes control commands based on state error (angle) in cascaded scheme
  /*
   * Gives better performance than control_Angle() but requires much more tuning. Not reccommended for first-time setup.
   * See the documentation for tuning this controller.
   */

  //inputs: roll_des, pitch_des, yawRate_des
  //outputs: roll_PID, pitch_PID, yaw_PID

  //desired values
  float roll_des = rcin_roll * maxRoll; //Between -maxRoll and +maxRoll
  float pitch_des = rcin_pitch * maxPitch; //Between -maxPitch and +maxPitch
  float yawRate_des = rcin_yaw * maxYawRate; //Between -maxYawRate and +maxYawRate

  //state vars
  static float integral_roll_ol, integral_roll_il, error_roll_prev, roll_IMU_prev, roll_des_prev;
  static float integral_pitch_ol, integral_pitch_il, error_pitch_prev, pitch_IMU_prev, pitch_des_prev;
  static float integral_yaw, error_yaw_prev;

  //Zero the integrators (used to don't let integrator build if throttle is too low, or to re-start the controller)
  if(zero_integrators) {
    integral_roll_ol = 0;
    integral_roll_il = 0;
    integral_pitch_ol = 0;
    integral_pitch_il = 0;
    integral_yaw = 0;
  }

  //Outer loop - PID on angle for roll & pitch
  //Roll
  float error_roll = roll_des - ahrs_roll;
  integral_roll_ol += error_roll * loop_dt;
  integral_roll_ol = constrain(integral_roll_ol, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_roll = (ahrs_roll - roll_IMU_prev) / loop_dt;  
  float roll_des_ol = Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll_ol;// - Kd_roll_angle*derivative_roll;

  //Pitch
  float error_pitch = pitch_des - ahrs_pitch; 
  integral_pitch_ol += error_pitch * loop_dt;
  integral_pitch_ol = constrain(integral_pitch_ol, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  float derivative_pitch = (ahrs_pitch - pitch_IMU_prev) / loop_dt; 
  float pitch_des_ol = Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch_ol;// - Kd_pitch_angle*derivative_pitch;

  //Apply loop gain, constrain, and LP filter for artificial damping
  float Kl = 30.0;
  roll_des_ol = Kl*roll_des_ol;
  pitch_des_ol = Kl*pitch_des_ol;
  roll_des_ol = constrain(roll_des_ol, -240.0, 240.0);
  pitch_des_ol = constrain(pitch_des_ol, -240.0, 240.0);
  roll_des_ol = (1.0 - B_loop_roll)*roll_des_prev + B_loop_roll*roll_des_ol;
  pitch_des_ol = (1.0 - B_loop_pitch)*pitch_des_prev + B_loop_pitch*pitch_des_ol;

  //Inner loop - PID on rate for roll & pitch
  //Roll
  error_roll = roll_des_ol - GyroX;
  integral_roll_il += error_roll * loop_dt;
  integral_roll_il = constrain(integral_roll_il, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup 
  derivative_roll = (error_roll - error_roll_prev) / loop_dt;
  roll_PID = 0.01 * (Kp_roll_rate*error_roll + Ki_roll_rate*integral_roll_il + Kd_roll_rate*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des_ol - GyroY;
  integral_pitch_il += error_pitch * loop_dt;
  integral_pitch_il = constrain(integral_pitch_il, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev) / loop_dt; 
  pitch_PID = 0.01 * (Kp_pitch_rate*error_pitch + Ki_pitch_rate*integral_pitch_il + Kd_pitch_rate*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range
  
  //Single loop
  //Yaw
  float error_yaw = yawRate_des - GyroZ;
  integral_yaw += error_yaw * loop_dt;    
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_yaw = (error_yaw - error_yaw_prev) / loop_dt;  
  yaw_PID = 0.01 * (Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range
  
  //Update derivative variables
  error_roll_prev = error_roll;
  roll_IMU_prev = ahrs_roll;
  roll_des_prev = roll_des_ol;
  error_pitch_prev = error_pitch;
  pitch_IMU_prev = ahrs_pitch;
  pitch_des_prev = pitch_des_ol;
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
  float error_roll = rollRate_des - GyroX;
  integral_roll += error_roll * loop_dt;
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_roll = (error_roll - error_roll_prev) / loop_dt;
  roll_PID = 0.01 * (Kp_roll_rate*error_roll + Ki_roll_rate*integral_roll + Kd_roll_rate*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  float error_pitch = pitchRate_des - GyroY;
  integral_pitch += error_pitch * loop_dt;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_pitch = (error_pitch - error_pitch_prev) / loop_dt;   
  pitch_PID = 0.01 * (Kp_pitch_rate*error_pitch + Ki_pitch_rate*integral_pitch + Kd_pitch_rate*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  float error_yaw = yawRate_des - GyroZ;
  integral_yaw += error_yaw * loop_dt;
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_yaw = (error_yaw - error_yaw_prev) / loop_dt; 
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

  //Quad mixing - EXAMPLE
  out_command[MOTOR1] = rcin_thro - pitch_PID - roll_PID - yaw_PID; //Back Right CW
  out_command[MOTOR2] = rcin_thro + pitch_PID - roll_PID + yaw_PID; //Front Right CCW
  out_command[MOTOR3] = rcin_thro - pitch_PID + roll_PID + yaw_PID; //Back Left CCW
  out_command[MOTOR4] = rcin_thro + pitch_PID + roll_PID - yaw_PID; //Front Left CW

  //0.0 is zero throttle if connecting to ESC for conventional PWM, 1.0 is max throttle
  //0.5 is centered servo, 0.0 and 1.0 are servo at their extreme positions as set with SERVO_MIN and SERVO_MAX
  //out_command[SERVO1] = 0;
  //out_command[SERVO2] = 0; 
}

void out_KillSwitch() {
  //DESCRIPTION: update out_armed status
  /*
      Change to ARMED when throttle is low and radio armed switch was flipped from disamed to armed position
      Change to DISARMED when radio armed switch is in disamed position
  */
  static bool rcin_armed_prev = false; 

  //Set ARMED when throttle is low and rcin_armed was flipped from unarmed to armed
  if (rcin_thro_is_low && rcin_armed && !rcin_armed_prev) {
    out_armed = true;
  }

  //Set DISARMED when rcin_armed is disarmed
   if (!rcin_armed) {
    out_armed = false;
  }

  //If DISARMED set motor outputs to 0
  if(!out_armed) for(int i=0;i<out_MOTOR_COUNT;i++) out_command[i] = 0; 

  rcin_armed_prev = rcin_armed;
}

void out_SetCommands() {
  //DESCRIPTION: Send pulses to motor pins, oneshot125 or PWM protocol and set servos
  for(int i=0;i<hw_OUT_COUNT;i++) out[i].writeFactor( out_command[i] );
}

//========================================================================================================================//
//                                              SETUP() FUNCTIONS                                                         //
//========================================================================================================================//

void loop_RateBegin() {
  loop_time = micros();
  loop_cnt = 0;    
}

void out_ArmMotors() {
  //DESCRIPTION: Sends many command pulses to the motors, to be used to arm motors in the void setup()
  /*  
   *  Loops over the commandMotors() function 50 times with a delay in between, simulating how the commandMotors()
   *  function is used in the main loop. Ensures motors arm within the void setup() where there are some delays
   *  for other processes that sometimes prevent motors from arming.
   */
  for (int i = 0; i <= 50; i++) {
    out_SetCommands();
    delay(2);
  }
}

//set initial quarterion
void ahrs_Setup() 
{
  //estimate yaw based on mag only (assumes vehicle is horizontal)

  //warm up imu by getting 100 samples
  for(int i=0;i<100;i++) {
    imu_GetData();
    delayMicroseconds(1000000/loop_freq);
  }

  //calculate yaw angle
  if(MagX == 0 && MagY == 0 && MagZ == 0) Serial.println("ahrs_Setup() No Magnetometer");
  float yaw = -atan2(MagY, MagX);
  ahrs_yaw = yaw * rad_to_deg;
  ahrs_pitch = 0;
  ahrs_roll = 0;

  //set initial quarterion
  q0 = cos(yaw/2);
  q1 = 0;
  q2 = 0;
  q3 = sin(yaw/2);

  Serial.printf("ahrs_Setup() Estimated yaw:%+.2f\n",ahrs_yaw);  
}

//========================================================================================================================//
//                                          CALIBRATION FUNCTIONS                                                         //
//========================================================================================================================//

void calibrate_IMU_error() {
  //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
  /*
   * Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and 
   * accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in imu_GetData(). This eliminates drift in the
   * measurement. 
   */
  float AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;

  AccErrorX = 0.0;
  AccErrorY = 0.0;
  AccErrorZ = 0.0;
  GyroErrorX = 0.0;
  GyroErrorY= 0.0;
  GyroErrorZ = 0.0;
  
  //Read IMU values 12000 times
  int cnt;
  for(cnt=0;cnt<12000;cnt++) {
    imu_Read(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
    
    //Sum all readings
    AccErrorX  = AccErrorX + AcX;
    AccErrorY  = AccErrorY + AcY;
    AccErrorZ  = AccErrorZ + AcZ;
    GyroErrorX = GyroErrorX + GyX;
    GyroErrorY = GyroErrorY + GyY;
    GyroErrorZ = GyroErrorZ + GyZ;
  }
  //Divide the sum by 12000 to get the error value
  AccErrorX  = AccErrorX / cnt;
  AccErrorY  = AccErrorY / cnt;
  AccErrorZ  = AccErrorZ / cnt - 1.0;
  GyroErrorX = GyroErrorX / cnt;
  GyroErrorY = GyroErrorY / cnt;
  GyroErrorZ = GyroErrorZ / cnt;

  Serial.printf("float AccErrorX = %+f;\n",AccErrorX);
  Serial.printf("float AccErrorY = %+f;\n",AccErrorY);
  Serial.printf("float AccErrorZ = %+f;\n",AccErrorZ);
  Serial.printf("float GyroErrorX = %+f;\n",GyroErrorX);
  Serial.printf("float GyroErrorY = %+f;\n",GyroErrorY);
  Serial.printf("float GyroErrorZ = %+f;\n",GyroErrorZ);

  Serial.println("Paste these values in user specified variables section and comment out calculate_IMU_error() in void setup.");
}

/*
void ahrs_warmup() {
  //DESCRIPTION: Used to warm up the main loop to allow the madwick filter to converge before commands can be sent to the actuators
  //Assuming vehicle is powered up on level surface!
  loop_RateBegin();
  for (int i = 0; i <= 10000; i++) {
    loop_Rate();
    imu_GetData();
    ahrs_Madgwick();
  }
}
*/

void calibrate_ESCs() {
  //DESCRIPTION: Used in void setup() to allow standard ESC calibration procedure with the radio to take place.
  /*  
   *  Simulates the void loop(), but only for the purpose of providing throttle pass through to the motors, so that you can
   *  power up with throttle at full, let ESCs begin arming sequence, and lower throttle to zero. This function should only be
   *  uncommented when performing an ESC calibration.
   */
  loop_RateBegin();
  while (true) {
    loop_Rate();
  
    digitalWrite(led_PIN, HIGH); //LED on to indicate we are not in main loop

    rcin_GetCommands(); //Pulls current available radio commands
    rcin_Failsafe(); //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup
    rcin_Normalize(); //Convert raw commands to normalized values based on saturated control limits
    imu_GetData(); //Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
    ahrs_Madgwick(); //Updates ahrs_roll, ahrs_pitch, and ahrs_yaw (degrees)
    rcin_Normalize(); //Convert raw commands to normalized values based on saturated control limits
    
    //set all motors
    for(int i=0;i<out_MOTOR_COUNT;i++) out_command[i] = rcin_thro;
  
    //out_KillSwitch(); //Don't update motor outputs to 0 if disarmed
    out_SetCommands(); //Sends command pulses to each motor pin
    
    //printRadioData(); //Radio pwm values (expected: 1000 to 2000)
  }
}

void calibrate_Magnetometer() {
  float bias[3], scale[3];

  Serial.println("Magnetometer calibration. Rotate the IMU about all axes until complete.");
  int rv = _calibrate_Magnetometer(bias, scale);
  if(rv==0) {
    Serial.println("Calibration Successful!");
    Serial.println("Please comment out the calibrateMagnetometer() function and copy these values into the code:");
    Serial.printf("float MagErrorX = %f;\n", bias[0]);
    Serial.printf("float MagErrorY = %f;\n", bias[1]);
    Serial.printf("float MagErrorZ = %f;\n", bias[2]);      
    Serial.printf("float MagScaleX = %f;\n", scale[0]);  
    Serial.printf("float MagScaleY = %f;\n", scale[1]);
    Serial.printf("float MagScaleZ = %f;\n", scale[2]);
    Serial.println(" ");
    Serial.println("If you are having trouble with your attitude estimate at a new flying location, repeat this process as needed.");
  }
  else {
    Serial.printf("Calibration Unsuccessful rv=%d. Please reset the board and try again.\n",rv);
  }

  while(1); //Halt code so it won't enter main loop until this function commented out
}

// finds bias and scale factor calibration for the magnetometer, the sensor should be rotated in a figure 8 motion until complete
// Note: Earth's field ranges between approximately 25 and 65 uT. (Europe & USA: 45-55 uT, inclination 50-70 degrees)
int _calibrate_Magnetometer(float bias[3], float scale[3]) 
{
  const int maxCounts = 1000; //sample for at least 10 seconds @ 100Hz
  const float deltaThresh = 0.3f; //uT
  const float B_coeff = 0.125;

  float ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
  float m[3] = {0};
  int counter;
  float m_filt[3];
  float m_max[3];
  float m_min[3];

  // get a starting set of data
  imu_Read(&ax, &ay, &az, &gx, &gy, &gz, &m[0], &m[1], &m[2]);
  for(int i=0;i<3;i++) {
    m_max[i] = m[i];
    m_min[i] = m[i];
    m_filt[i] = m[i];
  }

  // collect data to find max / min in each channel
  // sample counter times, restart sampling when a min/max changed at least deltaThresh uT
  uint32_t start_time = millis();
  counter = 0;
  while (counter < maxCounts) {
    imu_Read(&ax, &ay, &az, &gx, &gy, &gz, &m[0], &m[1], &m[2]);
    for(int i=0;i<3;i++) {
      m_filt[i] = m_filt[i] * (1 - B_coeff) + m[i] * B_coeff;
      if (m_max[i] < m_filt[i]) {
        float delta =  m_filt[i] - m_max[i];
        if (delta > deltaThresh) counter = 0;
        m_max[i] = m_filt[i];        
      }
      if (m_min[i] > m_filt[i]) {
        float delta = m_min[i] - m_filt[i];
        if (delta > deltaThresh) counter = 0;
        m_min[i] = m_filt[i];
      }
    }
    counter++;
    delay(10); //sample rate = 100Hz

    //print progress
    if(millis() - start_time > 1000) {
      start_time = millis();
      Serial.printf("xmin:%+.2f\txmax:%+.2f\tymin:%+.2f\tymax:%+.2f\tzmin:%+.2f\tzmax:%+.2f\n", m_min[0], m_max[0], m_min[1], m_max[1], m_min[2], m_max[2]);
    }
  }

  // find the magnetometer bias and scale
  float avg_scale = 0;
  for(int i=0;i<3;i++) { 
    bias[i] = (m_max[i] + m_min[i]) / 2;
    scale[i] = (m_max[i] - m_min[i]) / 2;
    avg_scale += scale[i];
  }
  for(int i=0;i<3;i++) {
    scale[i] = (avg_scale / 3) / scale[i];
  }

  return 0;
}

//========================================================================================================================//
//                                                PRINT FUNCTIONS                                                         //
//========================================================================================================================//

void print_overview() {
  Serial.printf("CH%d:%d\t",1,rcin_pwm[0]);  
  Serial.printf("rcin_roll:%+.2f\t",rcin_roll);
  Serial.printf("GyroX:%+.2f\t",GyroX);
  Serial.printf("AccX:%+.2f\t",AccX);
  Serial.printf("MagX:%+.2f\t",MagX);
  Serial.printf("ahrs_roll:%+.1f\t",ahrs_roll);
  Serial.printf("roll_PID:%+.3f\t",roll_PID);  
  Serial.printf("m%d%%:%1.0f\t", 1, 100*out_command[0]);
  Serial.printf("loop_rt:%d\t",(int)loop_rt);  
  print_need_newline = true;    
}

void print_rcin_RadioPWM() {
  for(int i=0;i<rcin_channels;i++) Serial.printf("pwm%d:%d\t",i+1,rcin_pwm[i]);
  print_need_newline = true;
}

void print_rcin_RadioScaled() {
  Serial.printf("rcin_thro:%.2f\t",rcin_thro);
  Serial.printf("rcin_roll:%+.2f\t",rcin_roll);
  Serial.printf("rcin_pitch:%+.2f\t",rcin_pitch);
  Serial.printf("rcin_yaw:%+.2f\t",rcin_yaw);
  Serial.printf("rcin_arm:%d\t",rcin_armed);
  Serial.printf("rcin_aux:%d\t",rcin_aux);
  Serial.printf("out_armed:%d\t",out_armed);  
  print_need_newline = true;
}

void print_imu_GyroData() {
  Serial.printf("GyroX:%+.2f\tGyroY:%+.2f\tGyroZ:%+.2f\t",GyroX,GyroY,GyroZ);
  print_need_newline = true;
}

void print_imu_AccData() {
  Serial.printf("AccX:%+.2f\tAccY:%+.2f\tAccZ:%+.2f\t",AccX,AccY,AccZ);
  print_need_newline = true;
}

void print_imu_MagData() {
  Serial.printf("MagX:%+.2f\tMagY:%+.2f\tMagZ:%+.2f\t",MagX,MagY,MagZ);
  print_need_newline = true;  
}

void print_ahrs_RollPitchYaw() {
  Serial.printf("ahrs_roll:%+.1f\tahrs_pitch:%+.1f\tahrs_yaw:%+.1f\t",ahrs_roll,ahrs_pitch,ahrs_yaw);
  Serial.printf("yaw_mag:%+.1f\t",-atan2(MagY, MagX) * rad_to_deg);
  print_need_newline = true;
}

void print_control_PIDoutput() {
  Serial.printf("roll_PID:%+.3f\tpitch_PID:%+.3f\tyaw_PID:%+.3f\t",roll_PID,pitch_PID,yaw_PID);  
  print_need_newline = true;
}

void print_out_MotorCommands() {
  Serial.printf("out_armed:%d", out_armed);  
  for(int i=0;i<out_MOTOR_COUNT;i++) Serial.printf("m%d%%:%1.0f\t", i+1, 100*out_command[i]);
  print_need_newline = true;    
}

void print_out_ServoCommands() {
  for(int i=out_MOTOR_COUNT;i<hw_OUT_COUNT;i++) Serial.printf("s%d%%:%1.0f\t", i-out_MOTOR_COUNT+1, 100*out_command[i]);
  print_need_newline = true;  
}

void print_loop_Rate() {
  Serial.printf("loop_dt:%d\t",(int)(loop_dt * 1000000.0));
  Serial.printf("loop_rt:%d\t",(int)loop_rt);
  Serial.printf("loop_rt_imu:%d\t",(int)loop_rt_imu);
  Serial.printf("loop_cnt:%d\t",(int)loop_cnt);  
  print_need_newline = true;
}

void print_i2c_scan() {
  Serial.printf("I2C: Scanning ...\n");
  byte count = 0;
  i2c->begin();
  for (byte i = 8; i < 120; i++) {
    i2c->beginTransmission(i);          // Begin I2C transmission Address (i)
    if (i2c->endTransmission() == 0) { // Receive 0 = success (ACK response) 
      Serial.printf("I2C: Found address: 0x%02X (%d)\n",i,i);
      count++;
    }
  }
  Serial.printf("I2C: Found %d device(s)\n", count);      
}

//===============================================================================================
// HELPERS

void die(String msg) {
  int cnt = 0;
  while(1) {
    Serial.print(msg);
    Serial.printf(" [%d]\n",cnt++);
    for(int i=0;i<10;i++) {
      digitalWrite(led_PIN, HIGH);
      delay(50);
      digitalWrite(led_PIN, LOW);
      delay(50);
    }
  }
}
