//Arduino ESP32 / ESP32-S3 / RP2040 / STM32 Flight Controller
//GPL-3.0 license
//Copyright (c) 2024 madflight https://madflight.com
 
/*#########################################################################################################################
How to use this PID Tuning demo program 
=======================================

See https://madflight.com for detailed description.

This is an extension of the QuadcopterAdvanced.ino example, it adds the option to change PID (or other) variables from your
RC transmitter. The currently selected PID and it's value are reported back to the ELRS/CRSF transmitter via the 
FlightMode telemetry sensor value.

The PWM commands are:

1020 First variable
1180 Next variable
1340 Previous variable
1500 None
1660 Secrease value
1820 Increase value
1980 Reset value

Setup your RC transmitter to generate these PWM commands on channel 7 from combinations of switches/toggles/trims, 
and display the FlightMode sensor value on the transmitter.

For example: EdgeTX mixer setup for 4 toggle switches SA-SD

|Source|Weight|Offset|Switch|Multiplex|
|------|------|------|------|---------|
| MAX  |  16  |  16  |  SA  | Replace |
|  SA  |  16  |  80  |  SB  | Replace |
| MAX  | -16  | -16  |  SC  | Replace |
|  SC  | -16  | -80  |  SD  | Replace |

This sets up the commands as follows:

SA    Decrease value
SB    Increase value
SA+SB Reset value
SC    Previous variable
SD    Next variable
SC+SD First variable

Install:
add to setup(): pidtune_setup(); 
add to loop(): pidtune_loop();
disable call to rcin_telemetry_flight_mode(...) in loop()

##########################################################################################################################*/

//========================================================================================================================//
//                              PID TUNING                                                                                //
//========================================================================================================================//
const int pidtune_channel = 7; 

struct pidtune_s {
  String name;
  float *pvalue;
  float value_reset;
};

//list of tunes
const int pidtune_cnt = 6;
pidtune_s pidtune[] = {
  {"Pr",&Kp_ro_pi_angle, 0},
  {"Ir",&Ki_ro_pi_angle, 0},
  {"Dr",&Kd_ro_pi_angle, 0},
  {"Py",&Kp_yaw, 0},
  {"Iy",&Ki_yaw, 0},
  {"Dy",&Kd_yaw, 0},
};

enum pidtune_command_e {
  PIDTUNE_FIRSTVAR,   //PWM 1020
  PIDTUNE_NEXTVAR,    //PWM 1180
  PIDTUNE_PREVVAR,    //PWM 1340
  PIDTUNE_NONE,       //PWM 1500
  PIDTUNE_DECVALUE,   //PWM 1660
  PIDTUNE_INCVALUE,   //PWM 1820
  PIDTUNE_RESETVALUE, //PWM 1980
  PIDTUNE_CMDCOUNT //number of commands
};
const int pidtune_pwm_min = 1020;
const int pidtune_pwm_max = 1980;
const int pidtune_pwm_spacing = (pidtune_pwm_max-pidtune_pwm_min) / (PIDTUNE_CMDCOUNT-1);
int pidtune_idx = 0;

void pidtune_setup() {
  for(int i=0;i<pidtune_cnt;i++) {
    pidtune[i].value_reset = *pidtune[i].pvalue;
  }
  pidtune_idx = 0;
}

bool pidtune_update() {
  static pidtune_command_e cmd_last = PIDTUNE_NONE;

  //get current variable
  pidtune_s tunevar = pidtune[pidtune_idx];

  //get current command
  int pwm = rcin_pwm[pidtune_channel-1];
  pidtune_command_e cmd = (pidtune_command_e) constrain((pwm - pidtune_pwm_min + pidtune_pwm_spacing/2 ) / pidtune_pwm_spacing, 0, PIDTUNE_CMDCOUNT-1);

  //process command
  bool updated = false;
  if(cmd == PIDTUNE_FIRSTVAR) {
    pidtune_idx = 0;
    updated = true;
  } else if(cmd_last == PIDTUNE_NONE && cmd == PIDTUNE_NEXTVAR) {
    pidtune_idx++;
    if(pidtune_idx > pidtune_cnt-1) pidtune_idx = 0;
    updated = true;
  } else if(cmd_last == PIDTUNE_NONE && cmd == PIDTUNE_PREVVAR) {
    pidtune_idx--;
    if(pidtune_idx < 0) pidtune_idx = pidtune_cnt-1;
    updated = true;
  } else if(cmd_last == PIDTUNE_NONE && cmd == PIDTUNE_DECVALUE) {
    *tunevar.pvalue *= (1.0f/1.1f); //decrease by 10%
    updated = true;
  } else if(cmd_last == PIDTUNE_NONE && cmd == PIDTUNE_INCVALUE) {
    *tunevar.pvalue *= 1.1f; //increase by 10%
    updated = true;    
  } else if(cmd == PIDTUNE_RESETVALUE) {
    *tunevar.pvalue = tunevar.value_reset;
    updated = true;    
  }
  cmd_last = cmd;

  return updated;
}

String pidtune_status() {
  pidtune_s tunevar = pidtune[pidtune_idx];
  return tunevar.name + String(*tunevar.pvalue,5);
}

void pidtune_loop() {
  if(pidtune_update()) rcin_telemetry_flight_mode(pidtune_status().c_str());
}
