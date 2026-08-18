/*#########################################################################################################################

RC-Airplane demo program for madflight Arduino ESP32-S3 / ESP32 / RP2350 / RP2040 / STM32 Flight Controller

WARNING: This program is experimental - NOT FLIGHT TESTED - PLEASE REPORT YOUR RESULTS

###########################################################################################################################

See http://madflight.com for setup instructions

This program is an airplane controller, it has 3 flight modes: MANUAL, ROLL and FBWA.

## MANUAL Mode

Regular RC control, no stabilization. All RC inputs are passed through to the servo outputs.

## ROLL Mode

Stabilize roll angle. Pitch and yaw are under manual control.

## FBWA Fly By Wire A Mode (inspired by ArduPilot)

This is the most popular mode for assisted flying, and is the best mode for inexperienced flyers. In this mode the
plane will hold the roll and pitch specified by the control sticks. So if you hold the aileron stick hard right then the 
plane will hold its pitch level and will bank right by the angle specified in the roll limit parameter. It is not possible 
to roll the plane past the roll limit, and it is not possible to pitch the plane beyond the pitch limit settings.

Note that holding level pitch does not mean the plane will hold altitude. How much altitude a plane gains or loses at a 
particular pitch depends on its airspeed, which is primarily controlled by throttle. So to gain altitude you should raise 
the throttle, and to lose altitude you should lower the throttle.

In FBWA mode yaw is under manual control.

## Setup Procedure

 1) First edit file "config_plane.h" and setup your board, imu and radio. Upload and use CLI to verify things work as expected.

 2) IMPORTANT: Use CLI calradio, calimu and calmag to calibrate radio, gyro, accelerometer, and magnetometer.

 3) Set the OUTPUTS section below.

 4) Remove props, and connect power, don't move plane until until LED starts blinking.

 5) IMPORTANT: Do a dry run and adjust settings as needed until completed.

    Set to MANUAL and power up the plane. Move the rc controls and make sure that the aileron, elevator, and rudder move in 
    the correct direction. Arm the plane, and carefully test the motor, then disarm.
    If incorrect: modify the #define OUT_ELEVATOR_DOWN etc. statements.

    Then set to FBWA flight mode, keep the radio sticks centered, and move the plane around, to make sure that the control 
    surfaces work to oppose the move, that is: pitching the plane down should move elevator up, banking right should deflect 
    the right aileron down, left aileron up.

Another thing that needs to be set are the PID parameters. Set to ROLL or FBWA mode and adjust the PID parameters so that the 
control surfaces react quickly, but don't oscillate, on changes in attitude.

###########################################################################################################################

See http://madflight.com for detailed description and modify the USER-SPECIFIED VARIABLES section below

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

#include "config_plane.h" //Edit this header file to setup the pins, hardware, radio, etc. for madflight
#include <madflight.h>

//prototypes (for PlatformIO, not needed for Arduino IDE)
void led_Blink();
void out_KillSwitchAndFailsafe();
void out_Mixer();

//========================================================================================================================//
//                                            USER-SPECIFIED VARIABLES                                                    //
//========================================================================================================================//

// Define outputs (Change as needed, note: pin_outX names are 0-based)
// If servos move in opposite direction as they should set xxx_MULT to -1, DON'T modify the rcl_xxx parameters to achieve this
// Keep unused pins assigned, just don't connect a servo

#define MOTOR_OUTPUT 0 //use pin_out0 as motor output
#define MOTOR_MULT 1

//GOTCHA FOR RP2040/RP2350: Put motor on even GPIO number and don't use the next GPIO as servo - pin_out1 is skipped for this reason

#define AILERON_OUTPUT 2 //use pin_out2 for aileron servo (two servos on this output, or left aileron)
#define AILERON_MULT 1

#define ELEVATOR_OUTPUT 3 //use pin_out3 for elevator servo
#define ELEVATOR_MULT 1

#define RUDDER_OUTPUT 4 //use pin_out4 for rudder servo
#define RUDDER_MULT 1

#define AILERON_RIGHT_OUTPUT 5 //use pin_out5 for right aileron servo
#define AILERON_RIGHT_MULT -1 //reverse output

#define FLAPS_OUTPUT 6 //use pin_out6 for flaps servo
#define FLAPS_MULT 1
#define FLAPS_RC_CHANNEL 7 //1 based RC channel number (RC channels 1-6 used by default for Roll, Pitch, Throttle, Yaw, Arm, Flightmode)

// Uncomment ONE mixer: 

// The mix matrix maps PID [roll,pitch,yaw,throttle] to outputs [ail,ele,rud,mot]

// Regular plane
// outputs: AILERON_OUTPUT=aileron, ELEVATOR_OUTPUT=elevator, RUDDER_OUTPUT=rudder, MOTOR_OUTPUT=throttle
float const mix[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}; 

// Delta wing
// outputs: AILERON_OUTPUT=left-elevon, ELEVATOR_OUTPUT=right-elevon, RUDDER_OUTPUT=rudder, MOTOR_OUTPUT=throttle
// when pid.roll positive -> roll right -> deflect left elevon down, deflect right elevon up
// when pid.pitch is positive -> pitch up -> deflect left elevon down, deflect right elevon down 
//float const mix[4][4] = {{1, 1, 0, 0}, {-1, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}; 

// V-Tail
// outputs: AILERON_OUTPUT=aileron, ELEVATOR_OUTPUT=left-ruddervator, RUDDER_OUTPUT=right-ruddervator, MOTOR_OUTPUT=throttle
// when pid.yaw positive -> yaw right -> deflect left ruddervator down, deflect right ruddervator up
// when pid.pitch is positive -> pitch up -> deflect left ruddervator down, deflect right ruddervator down 
//float const mix[4][4] = {{1, 0, 0, 0}, {0, 1, 1, 0}, {0, 1, -1, 0}, {0, 0, 0, 1}}; 


//========================================================================================================================//
//                                                       SETUP()                                                          //
//========================================================================================================================//

void setup() {
  // pre-madflight_setup vehicle configuration
  veh.mav_type = VEH_TYPE_PLANE; //set the vehicle type for logging and mavlink

  // Setup madflight modules, start madflight RTOS tasks, Serial.begin(11520)
  madflight_setup();

  //Standard servo at 50Hz with PWM pulse 1000 - 2000 us (change values as needed)
  out.setup_servo(AILERON_OUTPUT,       50, 1000, 2000); //Aileron
  out.setup_servo(ELEVATOR_OUTPUT,      50, 1000, 2000); //Elevator
  out.setup_servo(RUDDER_OUTPUT,        50, 1000, 2000); //Rudder
  out.setup_servo(FLAPS_OUTPUT,         50, 1000, 2000); //Flaps
  out.setup_servo(AILERON_RIGHT_OUTPUT, 50, 1000, 2000); //Right Aileron

  //Motor(s)
  // Uncomment ONE line - select output type
  bool success = out.setup_motors     (1, {MOTOR_OUTPUT}, 400, 950, 2000); // Standard PWM: 400Hz, 950-2000 us
  //bool success = out.setup_motors     (1, {MOTOR_OUTPUT}, 2000, 125, 250); // Oneshot125: 2000Hz, 125-250 us
  //bool success = out.setup_dshot      (1, {MOTOR_OUTPUT}, 300);            // Dshot300
  //bool success = out.setup_dshot_bidir(1, {MOTOR_OUTPUT}, 300);            // Dshot300 Bi-Directional
  //bool success = out.setup_brushed    (1, {MOTOR_OUTPUT}, 5000);           // Brushed motors: 5000Hz with 0-100% duty cycle

  out.print(); //print servo + motor configuration

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
  //Blink LED
  led_Blink();

  //Sensor fusion: update ahr.roll, ahr.pitch, and ahr.yaw angle estimates (degrees) from IMU data
  ahr.update(); 

  //PID Controller: update pid.roll, pid.pitch and pid.yaw controller outputs
  pid.controller();

  //Updates out.arm, the output armed flag
  out_KillSwitchAndFailsafe(); //Cut all motor outputs if DISARMED or failsafe triggered.

  //Actuator mixing
  out_Mixer(); //Mixes PID outputs and sends command pulses to the motors, if out.arm == true
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
/*
  Takes pid.roll, pid.pitch, and pid.yaw computed by the PID controller and appropriately mixes them for the desired
  vehicle configuration, then sends the mixed values to the servos/motor outputs. Throttle comes unmodified (passthru)
  from radio receiver (rcl.throttle)

  PID values are -1.0 to +1.0 (nominal), throttle is 0.0 to +1.0:

      rcl.throttle    0.0: idle throttle/stick back  1.0: full throttle/stick forward
      pid.roll.sum   -1.0: roll left/stick left      1.0: roll right/stick right
      pid.pitch.sum  -1.0: pitch down/stick forward  1.0: pitch up/stick back
      pid.yaw.sum    -1.0: yaw left/stick left       1.0: yaw right/stick right

  Servo/ESC outputs are 0.0 to +1.0:

      0.0 is zero throttle if connecting to ESC for conventional PWM, 1.0 is full throttle
      0.5 is centered servo, 0.0 and 1.0 are servo at their extreme positions as set with out.setup_servo
*/

  // Apply mixer
  float ail_mix = mix[0][0] * pid.roll.sum + mix[0][1] * pid.pitch.sum + mix[0][2] * pid.yaw.sum + mix[0][3] * pid.throttle.sum; // nominal range [-1 to 1]
  float ele_mix = mix[1][0] * pid.roll.sum + mix[1][1] * pid.pitch.sum + mix[1][2] * pid.yaw.sum + mix[1][3] * pid.throttle.sum; // nominal range [-1 to 1]
  float rud_mix = mix[2][0] * pid.roll.sum + mix[2][1] * pid.pitch.sum + mix[2][2] * pid.yaw.sum + mix[2][3] * pid.throttle.sum; // nominal range [-1 to 1]
  float thr_mix = mix[3][0] * pid.roll.sum + mix[3][1] * pid.pitch.sum + mix[3][2] * pid.yaw.sum + mix[3][3] * pid.throttle.sum; // nominal range [0 to 1]

  // Motor output
  float motor_out = MOTOR_MULT * (MOTOR_MULT > 0 ?  thr_mix : thr_mix - 1.0); //apply MULT
  out.set_output(MOTOR_OUTPUT, motor_out); // send to ESC

  // Aileron output: this is both-ailerons(plane/vtail), or left-aileron(plane/vtail), or left-elevon(delta)
  float ail_out = 0.5 + (AILERON_MULT * ail_mix) / 2.0; // apply MULT and adjust scale from mixer [-1 to 1] to servo output [0 to 1]
  out.set_output(AILERON_OUTPUT, ail_out);

  // Elevator output: this is elevator(plane), or left-aileron(delta), or left-ruddervator(vtail)
  float ele_out = 0.5 + (ELEVATOR_MULT * ele_mix) / 2.0; // apply MULT and adjust scale from mixer [-1 to 1] to servo output [0 to 1]
  out.set_output(ELEVATOR_OUTPUT, ele_out);

  // Rudder output: this is rudder(plane), or unused(delta), or right-ruddervator(vtail)
  float rud_out = 0.5 + (RUDDER_MULT * pid.yaw.sum) / 2.0; // apply MULT and adjust scale from mixer [-1 to 1] to servo output [0 to 1]
  out.set_output(RUDDER_OUTPUT, rud_out);

  // Right aileron output (plane)
  float ail_right_out = 0.5 + (AILERON_RIGHT_MULT * ail_mix) / 2.0; // apply MULT and adjust scale from mixer [-1 to 1] to servo output [0 to 1]
  out.set_output(AILERON_RIGHT_OUTPUT, ail_right_out);

  // Flaps output
  float flaps_mix = constrain( ((float)(rcl.pwm[FLAPS_RC_CHANNEL - 1] - 1100)) / (1900 - 1100), 0.0, 1.0); // scale PWM value from receiver to [0.0 - 1.0]
  float flaps_out = FLAPS_MULT * (FLAPS_MULT > 0 ? flaps_mix : flaps_mix - 1.0); // apply MULT
  out.set_output(FLAPS_OUTPUT, flaps_out); // send to flaps servo
}
