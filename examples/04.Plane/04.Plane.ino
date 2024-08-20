/*#########################################################################################################################

WARNING: This program is highly experimental - not flight tested at all - it was only dry run tested!

This is just a quick first attempt to make a plane controller, it has 3 flight modes: MANUAL, ROLL and FBWA.

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

Another thing that needs to be set are the PID parameters. Set to FBWA mode and adjust the PID parameters so that the 
control surfaces react quickly, but don't oscillate, on changes in attitude.

###########################################################################################################################

See http://madflight.com for detailed description

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
Copyright (c) 2024 https://github.com/qqqlab/madflight
##########################################################################################################################*/


//========================================================================================================================//
//                                                 PINS                                                                   //
//========================================================================================================================//

/* Place an additional / at the beginning of this line to setup our own pinout below, otherwise default pins from library/src/madflight_board_default_XXX.h are used

//Example pinout for LOLIN S3 (ESP32S3) with SPI MPU-9250 module directly soldered

//LED:
#define HW_PIN_LED       -1
#define HW_LED_ON         0 //0:low is on, 1:high is on

//IMU SPI:
#define HW_PIN_SPI_MISO  15
#define HW_PIN_SPI_MOSI   5
#define HW_PIN_SPI_SCLK   4
#define HW_PIN_IMU_CS    17
#define HW_PIN_IMU_EXTI  16 //external interrupt pin

//I2C for BARO, MAG, BAT sensors and for IMU if not using SPI
#define HW_PIN_I2C_SDA   -1
#define HW_PIN_I2C_SCL   -1

//Motor/Servo Outputs:
#define HW_OUT_COUNT     6 //number of outputs
#define HW_PIN_OUT_LIST  {13,14,21,47,48,45} //list of output pins

//Serial debug on USB Serial port (no GPIO pins)

//RC Receiver:
#define HW_PIN_RCIN_RX    12
#define HW_PIN_RCIN_TX    11
#define HW_PIN_RCIN_INVERTER -1 //only used for STM32 targets

//GPS:
#define HW_PIN_GPS_RX     -1 
#define HW_PIN_GPS_TX     -1
#define HW_PIN_GPS_INVERTER -1 //only used for STM32 targets

//Battery ADC
#define HW_PIN_BAT_V      -1
#define HW_PIN_BAT_I      -1

//BlackBox SPI:
#define HW_PIN_SPI2_MISO  -1
#define HW_PIN_SPI2_MOSI  -1
#define HW_PIN_SPI2_SCLK  -1
#define HW_PIN_BB_CS      -1
//*/

//RP2040 specific options
//#define HW_RP2040_SYS_CLK_KHZ 200000 //overclocking
//#define HW_RP2040_USE_FREERTOS //enable use of FreeRTOS - experimental

//ESP32 specific options
//#define USE_ESP32_SOFTWIRE //use bitbang I2C (not hardware I2C) See https://github.com/espressif/esp-idf/issues/4999

//========================================================================================================================//
//                                                 BOARD                                                                  //
//========================================================================================================================//
// Uncomment/change the following #include to the flight controller you want to use, or leave commented out to use the
// default board pinout (madflight_board_default_*.h). See library/madflight/src for all available boards

//#include <madflight_board_betaflight_MTKS-MATEKH743.h>

//========================================================================================================================//
//                                                 HARDWARE                                                               //
//========================================================================================================================//

//--- RC RECEIVER
#define RCIN_USE  RCIN_USE_CRSF // RCIN_USE_CRSF, RCIN_USE_SBUS, RCIN_USE_DSM, RCIN_USE_PPM, RCIN_USE_PWM
#define RCIN_NUM_CHANNELS  7 //number of receiver channels (minimal 6)

//--- IMU SENSOR
#define IMU_USE  IMU_USE_SPI_MPU9250 // IMU_USE_SPI_MPU6500, IMU_USE_SPI_MPU9250,IMU_USE_SPI_MPU6000, IMU_USE_SPI_BMI270, IMU_USE_I2C_MPU9250, IMU_USE_I2C_MPU9150, IMU_USE_I2C_MPU6500, IMU_USE_I2C_MPU6050, IMU_USE_I2C_MPU6000
//Set sensor orientation. The label is yaw / roll (in that order) needed to rotate the sensor from it's normal position to it's mounted position.
//If not sure what is needed: use CLI 'proll' and try each setting until roll-right gives positive ahrs.roll, pitch-up gives positive ahrs.pitch, and yaw-right gives positive ahrs.yaw
#define IMU_ALIGN  IMU_ALIGN_CW90FLIP //IMU_ALIGN_CW0, IMU_ALIGN_CW90, IMU_ALIGN_CW180, IMU_ALIGN_CW270, IMU_ALIGN_CW0FLIP, IMU_ALIGN_CW90FLIP, IMU_ALIGN_CW180FLIP, IMU_ALIGN_CW270FLIP
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
#define BB_USE  BB_USE_NONE //BB_USE_INTFLASH internal flash, BB_USE_FLASH external flash, BB_USE_RAM ram or psram, BB_USE_NONE

//========================================================================================================================//
//                                               RC RECEIVER                                                              //
//========================================================================================================================//

//set channels (1..RCIN_NUM_CHANNELS)
const int rcin_cfg_thro_channel  = 1; //low pwm = zero throttle/stick back, high pwm = full throttle/stick forward
const int rcin_cfg_roll_channel  = 2; //low pwm = left, high pwm = right
const int rcin_cfg_pitch_channel = 3; //low pwm = pitch up/stick back, high pwm = pitch down/stick forward
const int rcin_cfg_yaw_channel   = 4; //low pwm = left, high pwm = right
const int rcin_cfg_arm_channel   = 5; //ARM/DISARM switch
const int rcin_cfg_aux_channel   = 6; //Fight mode - 6 position switch
const int rcin_cfg_flaps_channel = 7; //Flaps

//throttle pwm values
const int rcin_cfg_thro_low      = 1250; //used to set rcin_thro_is_low flag when pwm is below. Note: your craft won't arm if this is too low.
const int rcin_cfg_thro_max      = 1900;

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
//                                               OUTPUTS                                                                  //
//========================================================================================================================//

//number of motors - out[0..out_MOTOR_COUNT-1] are motors, out[out_MOTOR_COUNT..HW_OUT_COUNT-1] are servos
const int out_MOTOR_COUNT = 1;

//define outputs and their channels 1..HW_OUT_COUNT (see control_Mixer() for available outputs)
//select output name based on what the output does when pwm is high. For example: If the right aileron goes down on high 
//pwm and is connected to output channel 2 use #define OUT_RIGHT_AILERON_DOWN 2
#define OUT_MOTOR1 1 //full throttle on high pwm
#define OUT_LEFT_AILERON_UP 2 //left aileron deflects up on high pwm (and right aileron down, otherwise use two servo channels)
//#define OUT_RIGHT_AILERON_DOWN 3 //right aileron deflects down on high pwm
#define OUT_ELEVATOR_UP 3 //elevator deflects up on high pwm
#define OUT_RUDDER_LEFT 4 //rudder deflects left on high pwm
#define OUT_FLAPS_UP_HALF 5 //flaps deflect up on high pwm, but only use 0.5 to 1.0 servo range

//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //
//========================================================================================================================//

//flight modes
enum rcin_fm_enum {MANUAL, ROLL, FBWA}; //available flight modes: MANUAL send rc commands directly to motor and aileron/pitch/yaw servos, ROLL stabilize roll angle, FBWA stabilize roll/pitch angles
const char* rcin_fm_str[] = {"MANUAL", "ROLL", "FBWA"}; //flight mode names used for telemetry
rcin_fm_enum rcin_fm_map[6] {MANUAL, MANUAL, ROLL, ROLL, FBWA, FBWA}; //flightmode mapping from 6 pos switch to flight mode (simulates a 3-pos switch: MANUAL/ROLL/FBWA)

const uint32_t imu_sample_rate = 1000; //imu sample rate in Hz (default 1000) NOTE: not all IMU drivers support a different rate
const uint32_t baro_sample_rate = 100; //baro sample rate in Hz (default 100)

//Low Pass Filter cutoff frequency in Hz. Do not touch unless you know what you are doing.
float LP_acc         = 70;        //Accelerometer  (default MPU6050: 50Hz, MPU9250: 70Hz)
float LP_gyr         = 60;        //Gyro           (default MPU6050: 35Hz, MPU9250: 60Hz)
float LP_mag         = 1e10;      //Magnetometer   (default 1e10Hz, i.e. no filtering)
float LP_radio       = 400;       //Radio Input    (default 400Hz)

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

//float Kp_yaw         = 0.3;       //Yaw P-gain
//float Ki_yaw         = 0.05;      //Yaw I-gain
//float Kd_yaw         = 0.00015;   //Yaw D-gain

//========================================================================================================================//
//                              DECLARE GLOBAL VARIABLES                                                                  //
//========================================================================================================================//

//Radio communication:
int   rcin_pwm[RCIN_NUM_CHANNELS]; //filtered raw PWM values
float rcin_thro; //throttle: 0(cutoff) to 1(full);
float rcin_roll, rcin_pitch, rcin_yaw;  // roll,pitch,yaw: -1(left,down) to 1(right,up) with 0 center stick
bool  rcin_armed; //status of arm switch, true = armed
bool  rcin_thro_is_low; //status of throttle stick, true = throttle low
int   rcin_aux; // six position switch connected to aux channel, values 0-5
float rcin_flaps; //flaps 0.0:up, 1.0:full down

//PID controller output
float roll_PID = 0, pitch_PID = 0, yaw_PID = 0;

//Flight status
rcin_fm_enum rcin_fm = (rcin_fm_enum)0; //current flight mode
bool  out_armed = false; //motors will only run if this flag is true

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
    Serial.printf("Plane " MADFLIGHT_VERSION " starting %d ...\n", i);
    delay(300);
    led.toggle();
  } 
  Serial.printf("Arduino library: " HW_ARDUINO_STR "\n");
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

  baro.setup(baro_sample_rate); //Barometer sample rate 100Hz
  mag.setup(); //External Magnetometer
  bat.setup(); //Battery Monitor
  bb.setup(); //Black Box
  gps_setup(); //GPS
  //gps_debug(); //uncomment to debug gps messages

  //Servos (set servos first just in case motors overwrite frequency of shared timers)
  for(int i=out_MOTOR_COUNT;i<HW_OUT_COUNT;i++) {
    out[i].begin(HW_PIN_OUT[i], 50, 1000, 2000); //Standard servo at 50Hz

    out_command[i] = 0.5; //set to 0 if you are using servo outputs for motors
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

  cli.welcome();

  led.off(); //Set built in LED off to signal end of startup
}

//========================================================================================================================//
//                                                            LOOP()                                                      //
//========================================================================================================================//

void loop() {
  //if IMU uses SPI bus (not I2C bus), then read slower i2c sensors here in loop() to keep imu_loop() as fast as possible
  if (!imu.usesI2C()) i2c_sensors_update();

  gps_loop(); //update gps

  //send telemetry
  static uint32_t rcin_telem_ts = 0;
  static uint32_t rcin_telem_cnt = 0;
  if(millis() - rcin_telem_ts > 100) {
    rcin_telem_ts = millis();
    rcin_telem_cnt++;
    String fm_str = String(out_armed ? "*" : "") + rcin_fm_str[rcin_fm];
    rcin_telemetry_flight_mode(fm_str.c_str());  //only first 14 char get transmitted
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

  //PID Controller
  switch(rcin_fm) {
    case ROLL:
      control_ROLL(rcin_thro_is_low); //Stabilize on roll angle setpoints
      break;    
    case FBWA:
      control_FBWA(rcin_thro_is_low); //Stabilize on pitch/roll angle setpoints
      break;
    default:
      control_MANUAL();
  }

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
  rcin_armed = (rcin_cfg_arm_min <= pwm && pwm <= rcin_cfg_arm_max); //output: true/false

  //aux 6 position switch (flight mode)
  int spacing = (rcin_cfg_aux_max - rcin_cfg_aux_min) / 5;
  rcin_aux = constrain( ( rcin_pwm[rcin_cfg_aux_channel-1] - rcin_cfg_aux_min + spacing/2) / spacing, 0, 5); //output 0..5
  rcin_fm = rcin_fm_map[rcin_aux];

  //flaps
  rcin_flaps = constrain( ((float)(rcin_pwm[rcin_cfg_flaps_channel-1] - rcin_cfg_pwm_min)) / (rcin_cfg_pwm_max - rcin_cfg_pwm_min), 0.0, 1.0); //output: 0.0 to 1.0
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
  //outputs: roll_PID, pitch_PID, yaw_PID

  //desired values
  float roll_des = rcin_roll * maxRoll; //Between -maxRoll and +maxRoll
  float pitch_des = rcin_pitch * maxPitch + fbwa_pitch_offset; //Between fbwa_pitch_offset-maxPitch and fbwa_pitch_offset+maxPitch

  //state vars
  static float integral_roll, integral_pitch, error_yaw_prev, integral_yaw;

  //Zero the integrators (used to don't let integrator build if throttle is too low, or to re-start the controller)
  if(zero_integrators) {
    integral_roll = 0;
    integral_pitch = 0;
    integral_yaw = 0;
  }

  //Roll PID - stabilize desired roll angle
  float error_roll = roll_des - ahrs.roll;
  integral_roll += error_roll * imu.dt;
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_roll = ahrs.gx;
  roll_PID = Kp_roll*error_roll + Ki_roll*integral_roll - Kd_roll*derivative_roll; //nominal output -1 to 1 (can be larger)

  //Pitch PID - stabilize desired pitch angle
  float error_pitch = pitch_des - ahrs.pitch;
  integral_pitch += error_pitch * imu.dt;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_pitch = ahrs.gy; 
  pitch_PID = Kp_pitch*error_pitch + Ki_pitch*integral_pitch - Kd_pitch*derivative_pitch; //nominal output -1 to 1 (can be larger)

  //Yaw PID - passthru rcin
  yaw_PID = rcin_yaw;
  (void) integral_yaw;
  (void) error_yaw_prev;

  /*
  //TODO Yaw PID - Stabilize on zero slip, i.e. keep gravity Y component zero
  float error_yaw = 0 - ahrs.ay;
  integral_yaw += error_yaw * imu.dt;
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_yaw = (error_yaw - error_yaw_prev) / imu.dt; 
  yaw_PID = constrain(0.01 * (Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw), -1.0f, 1.0f); //Scaled by .01 to bring within -1 to 1 range

  //Update derivative variables
  error_yaw_prev = error_yaw;
  */  
}

void control_ROLL(bool zero_integrators) {
  //inputs: roll_des, pitch_des, yawRate_des
  //outputs: roll_PID, pitch_PID, yaw_PID

  //desired values
  float roll_des = rcin_roll * maxRoll; //Between -maxRoll and +maxRoll

  //state vars
  static float integral_roll;

  //Zero the integrators (used to don't let integrator build if throttle is too low, or to re-start the controller)
  if(zero_integrators) {
    integral_roll = 0;
  }

  //Roll PID - stabilize desired roll angle
  float error_roll = roll_des - ahrs.roll;
  integral_roll += error_roll * imu.dt;
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_roll = ahrs.gx;
  roll_PID = Kp_roll*error_roll + Ki_roll*integral_roll - Kd_roll*derivative_roll; //nominal output -1 to 1 (can be larger)

  //Pitch PID - passthru rcin
  pitch_PID = rcin_pitch;

  //Yaw PID - passthru rcin
  yaw_PID = rcin_yaw; 
}


void control_MANUAL() {
/* MANUAL Mode
Regular RC control, no stabilization. All RC inputs are passed through to the servo outputs.
*/  
  //pass rcin through to PID - PID values are -1 to +1, rcin values are -1 to +1
  roll_PID = rcin_roll;  //-1 = left, 1 = right
  pitch_PID = rcin_pitch; //-1 = pitch up/stick back, 1 = pitch down/stick forward
  yaw_PID = rcin_yaw; //-1 = left, 1 = right
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

    rcin_thro   0: idle throttle/stick back  1: full throttle/stick forward
    roll_PID   -1: roll left/stick left      1: roll right/stick right
    pitch_PID  -1: pitch up/stick back       1: pitch down/stick forward
    yaw_PID    -1: yaw left/stick left       1: yaw right/stick right
   */

  //Plane mixing - PID values are -1 to +1 (nominal), SERVO values are 0 to 1 (clipped by pwm class)

  //motor: full throttle on rcin_thro
  #ifdef OUT_MOTOR1 //full throttle on high pwm
    out_command[OUT_MOTOR1-1] = +rcin_thro;
  #endif
  #ifdef OUT_MOTOR1_REVERSED //reversed: idle throttle on high pwm
    out_command[OUT_MOTOR1_REVERSED-1] = 1.0 - rcin_thro;
  #endif  
  #ifdef OUT_MOTOR2 //full throttle on high pwm
    out_command[OUT_MOTOR2-1] = +rcin_thro;
  #endif  
  #ifdef OUT_MOTOR2_REVERSED //reversed: idle throttle on high pwm
    out_command[OUT_MOTOR2_REVERSED-1] = 1.0 - rcin_thro;
  #endif 

  //aileron: when roll_PID positive -> roll right -> deflect left aileron down, deflect right aileron up
  #ifdef OUT_LEFT_AILERON_DOWN //left aileron deflects down on high pwm
    out_command[OUT_LEFT_AILERON_DOWN-1] = +roll_PID/2.0 + 0.5;
  #endif
  #ifdef OUT_RIGHT_AILERON_UP //right aileron deflects up on high pwm
    out_command[OUT_RIGHT_AILERON_UP-1] = +roll_PID/2.0 + 0.5;
  #endif
  #ifdef OUT_LEFT_AILERON_UP //reversed: left aileron deflects up on high pwm
    out_command[OUT_LEFT_AILERON_UP-1] = -roll_PID/2.0 + 0.5;
  #endif
  #if defined(OUT_RIGHT_AILERON_DOWN) //reversed: right aileron deflects down on high pwm
    out_command[OUT_RIGHT_AILERON_DOWN-1] = -roll_PID/2.0 + 0.5;
  #endif

  //elevator: when pitch_PID is positive -> pitch up -> deflect elevator down 
  #ifdef OUT_ELEVATOR_DOWN //elevator deflects down on high pwm
    out_command[OUT_ELEVATOR_UP-1] = +pitch_PID/2.0 + 0.5;
  #endif
  #ifdef OUT_ELEVATOR_UP //reversed: elevator deflects up on high pwm
    out_command[OUT_ELEVATOR_UP-1] = -pitch_PID/2.0 + 0.5;
  #endif

  //rudder: when yaw_PID is positive -> yaw right -> deflect rudder right
  #ifdef OUT_RUDDER_RIGHT //rudder deflects right on high pwm 
    out_command[OUT_RUDDER_RIGHT-1] = +yaw_PID/2.0 + 0.5;
  #endif  
  #ifdef OUT_RUDDER_LEFT //reversed: rudder deflects left on high pwm
    out_command[OUT_RUDDER_LEFT-1] = -yaw_PID/2.0 + 0.5;
  #endif

  //flaps:
  #ifdef OUT_FLAPS_DOWN //flaps deflect down on high pwm
    out_command[OUT_FLAPS_DOWN-1] = +rcin_flaps;
  #endif
  #ifdef OUT_FLAPS_DOWN_HALF //flaps deflect down on high pwm (but only use servo range 0.5 to 1.0)
    out_command[OUT_FLAPS_DOWN_HALF-1] = +rcin_flaps/2.0 + 0.5;
  #endif  
  #ifdef OUT_FLAPS_UP //reversed: flaps deflect up on high pwm
    out_command[OUT_FLAPS_UP-1] = -rcin_flaps;
  #endif
  #ifdef OUT_FLAPS_UP_HALF //flaps deflect up on high pwm (but only use servo range 0.5 to 1.0)
    out_command[OUT_FLAPS_UP_HALF-1] = -rcin_flaps/2.0 + 0.5;
  #endif 

  //delta wing:
  // when roll_PID positive -> roll right -> deflect left elevon down, deflect right elevon up
  // when pitch_PID is positive -> pitch up -> deflect left elevon down, deflect right elevon down 
  #ifdef OUT_LEFT_ELEVON_DOWN //left elevon deflects down on high input
    out_command[OUT_LEFT_ELEVON_DOWN-1] = +roll_PID/2.0 +pitch_PID/2.0 + 0.5;
  #endif
  #ifdef OUT_RIGHT_ELEVON_UP //right elevon deflects up on high input
    out_command[OUT_RIGHT_ELEVON_UP-1] = +roll_PID/2.0 -pitch_PID/2.0 + 0.5;
  #endif
  #ifdef OUT_LEFT_ELEVON_UP //reversed: left elevon deflects down on high input
    out_command[OUT_LEFT_ELEVON_UP-1] = -roll_PID/2.0 -pitch_PID/2.0 + 0.5;
  #endif  
  #ifdef OUT_RIGHT_ELEVON_DOWN //reversed: right elevon deflects down on high input
    out_command[OUT_RIGHT_ELEVON_DOWN-1] = -roll_PID/2.0 +pitch_PID/2.0 + 0.5;
  #endif 

  //0.0 is zero throttle if connecting to ESC for conventional PWM, 1.0 is max throttle
  //0.5 is centered servo, 0.0 and 1.0 are servo at their extreme positions as set with SERVO_MIN and SERVO_MAX
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
