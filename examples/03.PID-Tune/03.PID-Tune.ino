// This example a modified QuadcopterAdvanced.ino example, it adds PID-Tuning via RC. 
// Search for "pidtune" in this file to see the changes.

/*#########################################################################################################################

NOTICE: This program is experimental - first get Quadcopter.ino to fly before attempting this program 

###########################################################################################################################

See http://madflight.com for detailed description

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

LED State                              Meaning
---------                              -------
OFF                                    Not powered
ON                                     Startup (don't move, running gyro calibration)
Blinking long OFF short ON             DISARMED
Blinking long ON short OFF             ARMED
Blink interval longer than 1 second    imu_loop() is taking too much time
fast blinking                          Something is wrong, connect USB serial for info

MIT license
Copyright (c) 2023-2024 https://madflight.com
##########################################################################################################################*/


//========================================================================================================================//
//                                                 PINS                                                                   //
//========================================================================================================================//
//
// You have 3 options to setup the pins (gpio numbers) for the flight controller:
//
//   1) Default - Leave this section as is and see https://madflight.com for default pinout diagrams for the supported
//      processor families. Default pinouts are defined in the board header files library/src/madflight_board_default_XXX.h
// 
//   2) Header - #include the BetaFlight flight controller you want to use. See library/madflight/src for all available 
//      boards. For example: #include <madflight_board_betaflight_MTKS-MATEKH743.h>
// 
//   3) Custom - Remove /* below to enable the CUSTOM PINS section, and define own pinout.
//
//========================================================================================================================//

/*
//========================================================================================================================//
//                                              CUSTOM PINS                                                               //
//========================================================================================================================//

#define HW_BOARD_NAME "My Custom Board" //REQUIRED: Give your board a name - without a name the default pinout is loaded!!!

//Replace 'pp' with the gpio number you want to use, or comment out the #define if the pin is not used
//NOTE: Not all pins can be freely configured. Read the processor datasheet, or use the default pinout.

//LED:
#define HW_PIN_LED        pp
#define HW_LED_ON          0 //0:low is on, 1:high is on

//IMU SPI:
#define HW_PIN_SPI_MISO   pp
#define HW_PIN_SPI_MOSI   pp
#define HW_PIN_SPI_SCLK   pp
#define HW_PIN_IMU_CS     pp
#define HW_PIN_IMU_EXTI   pp //REQUIRED: IMU external interrupt pin (required for SPI and I2C sensors)

//I2C for BARO, MAG, BAT sensors (and for IMU if not using SPI IMU)
#define HW_PIN_I2C_SDA    pp
#define HW_PIN_I2C_SCL    pp

//Motor/Servo Outputs:
#define HW_OUT_COUNT      4 //number of outputs
#define HW_PIN_OUT_LIST   {pp,pp,pp,pp} //list of output pins, enter exactly HW_OUT_COUNT pins.

//Serial debug on USB Serial port (no GPIO pins)

//RC Receiver:
#define HW_PIN_RCIN_RX    pp
#define HW_PIN_RCIN_TX    pp
#define HW_PIN_RCIN_INVERTER pp //only used for STM32 targets

//GPS:
#define HW_PIN_GPS_RX     pp 
#define HW_PIN_GPS_TX     pp
#define HW_PIN_GPS_INVERTER pp //only used for STM32 targets

//Battery ADC
#define HW_PIN_BAT_V      pp
#define HW_PIN_BAT_I      pp

//Black Box SPI (for sdcard or external flash chip):
#define HW_PIN_SPI2_MISO  pp
#define HW_PIN_SPI2_MOSI  pp
#define HW_PIN_SPI2_SCLK  pp
#define HW_PIN_BB_CS      pp

//Black Box SDCARD via MMC interface:
#define HW_PIN_SDMMC_DATA pp
#define HW_PIN_SDMMC_CLK  pp
#define HW_PIN_SDMMC_CMD  pp
//*/

//RP2040 specific options
//#define HW_RP2040_SYS_CLK_KHZ 200000 //overclocking

//ESP32 specific options
//#define USE_ESP32_SOFTWIRE //use bitbang I2C (not hardware I2C) See https://github.com/espressif/esp-idf/issues/499


//========================================================================================================================//
//                                                 USER-SPECIFIED DEFINES                                                 //
//========================================================================================================================//

//--- RC RECEIVER
#define RCIN_USE  RCIN_USE_CRSF // RCIN_USE_CRSF, RCIN_USE_SBUS, RCIN_USE_DSM, RCIN_USE_PPM, RCIN_USE_PWM
//pidtune disable: #define RCIN_NUM_CHANNELS 6 //number of receiver channels (minimal 6)
#define RCIN_NUM_CHANNELS 7 //pidtune: add one channel for PID-tuning

//--- IMU SENSOR
#define IMU_USE  IMU_USE_SPI_MPU6500 // IMU_USE_SPI_MPU6500, IMU_USE_SPI_MPU9250,IMU_USE_SPI_MPU6000, IMU_USE_SPI_BMI270, IMU_USE_I2C_MPU9250, IMU_USE_I2C_MPU9150, IMU_USE_I2C_MPU6500, IMU_USE_I2C_MPU6050, IMU_USE_I2C_MPU6000
//Set sensor orientation. The label is yaw / roll (in that order) needed to rotate the sensor from it's normal position to it's mounted position.
//If not sure what is needed: use CLI 'proll' and try each setting until roll-right gives positive ahrs.roll, pitch-up gives positive ahrs.pitch, and yaw-right gives positive ahrs.yaw
#define IMU_ALIGN  IMU_ALIGN_CW90 //IMU_ALIGN_CW0, IMU_ALIGN_CW90, IMU_ALIGN_CW180, IMU_ALIGN_CW270, IMU_ALIGN_CW0FLIP, IMU_ALIGN_CW90FLIP, IMU_ALIGN_CW180FLIP, IMU_ALIGN_CW270FLIP
#define IMU_I2C_ADR  0x69 //IMU I2C address. If unknown, use CLI 'i2c'

//-- AHRS sensor fusion 
#define AHRS_USE AHRS_USE_MAHONY // AHRS_USE_MAHONY, AHRS_USE_MAHONY_BF, AHRS_USE_MADGWICK, AHRS_USE_VQF

//--- GPS
#define GPS_BAUD  115200

//--- BAROMETER SENSOR
#define BARO_USE  BARO_USE_NONE // BARO_USE_BMP280, BARO_USE_MS5611, BARO_USE_NONE
//#define BARO_I2C_ADR  0x76 //set barometer I2C address, leave commented for default address. If unknown, use CLI 'i2c'

//--- EXTERNAL MAGNETOMETER SENSOR
#define MAG_USE  MAG_USE_NONE // MAG_USE_QMC5883L, MAG_USE_NONE
//#define MAG_I2C_ADR  0x77 //set magnetometer I2C address, leave commented for default address. If unknown, use CLI 'i2c'

//--- BATTERY MONITOR
#define BAT_USE  BAT_USE_NONE // BAT_USE_INA226, BAT_USE_ADC, BAT_USE_NONE

//--- BLACKBOX LOGGER
#define BB_USE  BB_USE_NONE //BB_USE_SD spi sdcard, BB_USE_SDMMC mmc sdcard, BB_USE_NONE

//========================================================================================================================//
//                                               RC RECEIVER CONFIG                                                       //
//========================================================================================================================//

//set channels
const int rcin_cfg_thro_channel  = 1; //low pwm = zero throttle/stick back, high pwm = full throttle/stick forward
const int rcin_cfg_roll_channel  = 2; //low pwm = left, high pwm = right
const int rcin_cfg_pitch_channel = 3; //low pwm = pitch up/stick back, high pwm = pitch down/stick forward
const int rcin_cfg_yaw_channel   = 4; //low pwm = left, high pwm = right
const int rcin_cfg_arm_channel   = 5; //ARM/DISARM switch
const int rcin_cfg_aux_channel   = 6; //Fight mode - 6 position switch

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

//6 position switch on aux channel - Ardupilot switch pwm: 1165,1295,1425,1555,1685,1815 (spacing 130)
//EdgeTx 3-pos SA + 2-pos SB setup: Source:SA weight:52 offset:0, Source:SB weight:13 offset:-1 multiplex: add -OR- Source:SA Weight:26 Offset:-40 Switch:SBdown, Source:SA Weight:26 Offset:36 Switch:SBup Multiplex:Replace
int rcin_cfg_aux_min = 1165; //lowest switch pwm
int rcin_cfg_aux_max = 1815; //higest switch pwm

//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //
//========================================================================================================================//

//number of motors - out[0..out_MOTOR_COUNT-1] are motors, out[out_MOTOR_COUNT..HW_OUT_COUNT-1] are servos
const int out_MOTOR_COUNT = 4;
//name the outputs, to make code more readable
enum out_enum {MOTOR1,MOTOR2,MOTOR3,MOTOR4,SERVO1,SERVO2,SERVO3,SERVO4,SERVO5,SERVO6,SERVO7,SERVO8,SERVO9,SERVO10,SERVO11,SERVO12}; 

const uint32_t imu_sample_rate = 1000; //imu sample rate in Hz (default 1000) NOTE: not all IMU drivers support a different rate
const uint32_t baro_sample_rate = 100; //baro sample rate in Hz (default 100)

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
#include "pidtune.h"

//========================================================================================================================//
//                                                       SETUP()                                                          //
//========================================================================================================================//

void setup() {
  led.setup(HW_PIN_LED, HW_LED_ON); //Set built in LED to turn on to signal startup
  Serial.begin(115200); //start console serial

  //6 second startup delay
  for(int i=20;i>0;i--) { 
    Serial.printf("PID-Tune " MADFLIGHT_VERSION " starting %d ...\n", i);
    delay(300);
  }
  Serial.printf("Arduino library: " HW_ARDUINO_STR "\n");

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

  baro.setup(baro_sample_rate); //Barometer sample rate 100Hz
  mag.setup(); //External Magnetometer
  bat.setup(); //Battery Monitor
  bb.setup(); //Black Box
  gps_setup(); //GPS
  //gps_debug(); //uncomment to debug gps messages

  //Servos (set servos first just in case motors overwrite frequency of shared timers)
  for(int i=out_MOTOR_COUNT;i<HW_OUT_COUNT;i++) {
    out[i].begin(HW_PIN_OUT[i], 50, 1000, 2000); //Standard servo at 50Hz

    out_command[i] = 0; //keep at 0 if you are using servo outputs for motors
    out[i].writeFactor(out_command[i]); //start the PWM output to the servos
  } 

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
  if(!imu.waitNewSample()) die("IMU interrupt not firing.");

  //Calibrate for zero gyro readings, assuming vehicle not moving when powered up. Comment out to only use cfg values. (Use CLI to calibrate acc.)
  cli.calibrate_gyro();

  pidtune_setup();

  cli.welcome();

  led.enable(); //Set LED off to signal end of startup, and enable blinking by imu_loop()
}

//========================================================================================================================//
//                                                            LOOP()                                                      //
//========================================================================================================================//

void loop() {
  //if IMU uses SPI bus (not I2C bus), then read slower i2c sensors here in loop() to keep imu_loop() as fast as possible
  if (!imu.usesI2C()) i2c_sensors_update();

  gps_loop(); //update gps

  //send telemetry
  pidtune_loop();
  static uint32_t rcin_telem_ts = 0;
  static uint32_t rcin_telem_cnt = 0;
  if(millis() - rcin_telem_ts > 100) {
    rcin_telem_ts = millis();
    rcin_telem_cnt++;
    //pidtune disable: if(out_armed) rcin_telemetry_flight_mode("ARMED"); else rcin_telemetry_flight_mode("madflight"); //only first 14 char get transmitted
    rcin_telemetry_attitude(ahrs.pitch, ahrs.roll, ahrs.yaw);  
    if(rcin_telem_cnt % 10 == 0) rcin_telemetry_battery(bat.v, bat.i, bat.mah, 100);
    if(rcin_telem_cnt % 10 == 5) rcin_telemetry_gps(gps.lat, gps.lon, gps.sog/278, gps.cog/1000, (gps.alt<0 ? 0 : gps.alt/1000), gps.sat); // sog/278 is conversion from mm/s to km/h 
  }

  cli.loop(); //process CLI commands
}

//update all I2C sensors, called from loop() with SPI IMU, or called from imu_loop() with I2C IMU
void i2c_sensors_update() {
  if(bat.update()) bb.log_bat(); //update battery, and log if battery was updated. 
  if(baro.update()) bb.log_baro(); //log if pressure updated
  mag.update();
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

  //PID Controller - SELECT ONE:
  control_Angle(rcin_thro_is_low); //Stabilize on pitch/roll angle setpoint, stabilize yaw on rate setpoint  //control_Angle2(rcin_thro_is_low); //Stabilize on pitch/roll setpoint using cascaded method. Rate controller must be tuned well first!
  //control_Rate(rcin_thro_is_low); //Stabilize on rate setpoint

  //Actuator mixing
  control_Mixer(); //Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here

  //Motor output
  out_KillSwitchAndFailsafe(); //Cut all motor outputs if DISARMED or failsafe triggered.
  out_SetCommands(); //Sends command pulses to motors (only if out_armed=true) and servos

  //if IMU uses I2C bus, then get I2C sensor readings in imu_interrupt_handler() to prevent I2C bus collisions. Alternatively, put the IMU on a separate I2C bus.
  if (imu.usesI2C()) i2c_sensors_update();

  //bb.log_imu(); //full speed black box logging of IMU data, memory fills up quickly...
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

  //aux 6 position switch
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
  float error_roll = roll_des - ahrs.roll;
  integral_roll_ol += error_roll * imu.dt;
  integral_roll_ol = constrain(integral_roll_ol, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_roll = (ahrs.roll - roll_IMU_prev) / imu.dt;  
  float roll_des_ol = Kp_ro_pi_angle*error_roll + Ki_ro_pi_angle*integral_roll_ol;// - Kd_ro_pi_angle*derivative_roll;

  //Pitch
  float error_pitch = pitch_des - ahrs.pitch; 
  integral_pitch_ol += error_pitch * imu.dt;
  integral_pitch_ol = constrain(integral_pitch_ol, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  float derivative_pitch = (ahrs.pitch - pitch_IMU_prev) / imu.dt; 
  float pitch_des_ol = Kp_ro_pi_angle*error_pitch + Ki_ro_pi_angle*integral_pitch_ol;// - Kd_ro_pi_angle*derivative_pitch;

  //Apply loop gain, constrain, and LP filter for artificial damping
  float Kl = 30.0;
  roll_des_ol = Kl*roll_des_ol;
  pitch_des_ol = Kl*pitch_des_ol;
  roll_des_ol = constrain(roll_des_ol, -240.0, 240.0);
  pitch_des_ol = constrain(pitch_des_ol, -240.0, 240.0);
  roll_des_ol = (1.0 - B_loop_ro_pi)*roll_des_prev + B_loop_ro_pi*roll_des_ol;
  pitch_des_ol = (1.0 - B_loop_ro_pi)*pitch_des_prev + B_loop_ro_pi*pitch_des_ol;

  //Inner loop - PID on rate for roll & pitch
  //Roll
  error_roll = roll_des_ol - ahrs.gx;
  integral_roll_il += error_roll * imu.dt;
  integral_roll_il = constrain(integral_roll_il, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup 
  derivative_roll = (error_roll - error_roll_prev) / imu.dt;
  roll_PID = 0.01 * (Kp_ro_pi_rate*error_roll + Ki_ro_pi_rate*integral_roll_il + Kd_ro_pi_rate*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des_ol - ahrs.gy;
  integral_pitch_il += error_pitch * imu.dt;
  integral_pitch_il = constrain(integral_pitch_il, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev) / imu.dt; 
  pitch_PID = 0.01 * (Kp_ro_pi_rate*error_pitch + Ki_ro_pi_rate*integral_pitch_il + Kd_ro_pi_rate*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range
  
  //Single loop
  //Yaw
  float error_yaw = yawRate_des - ahrs.gz;
  integral_yaw += error_yaw * imu.dt;    
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_yaw = (error_yaw - error_yaw_prev) / imu.dt;  
  yaw_PID = 0.01 * (Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range
  
  //Update derivative variables
  error_roll_prev = error_roll;
  roll_IMU_prev = ahrs.roll;
  roll_des_prev = roll_des_ol;
  error_pitch_prev = error_pitch;
  pitch_IMU_prev = ahrs.pitch;
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

void out_KillSwitchAndFailsafe() {
  static bool rcin_armed_prev = true; //initial value is true: forces out_armed false on startup even if arm switch is ON

  //Change to ARMED when throttle is low and radio armed switch was flipped from disamed to armed position
  if (!out_armed && rcin_thro_is_low && rcin_armed && !rcin_armed_prev) {
    out_armed = true;
    Serial.println("OUT: ARMED");
    bb.start(); //start blackbox logging
  }

  //Change to DISARMED when radio armed switch is in disarmed position, or if radio lost connection
   if (out_armed && (!rcin_armed || !rcin.connected())) {
    out_armed = false;
    if(!rcin_armed) {
      Serial.println("OUT: DISARMED (arm switch)");
      bb.stop(); //stop blackbox logging
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

//========================================================================================================================//
//                  SETUP1() LOOP1() EXECUTING ON SECOND CORE                                                             //
//========================================================================================================================//
//Uncomment setup1() and/or loop1() to use the second core on ESP32 / RP2040 with FreeRTOS
/*
void setup1() {
  Serial.println("setup1()");
}
void loop1() {
  Serial.println("loop1()"); delay(100);
}
//*/
