#include "../madflight_modules.h"
#include "msp/msp.h"
#include "cli_RclCalibrate.h"
#include "stat.h"
#include "FreeRTOS_ps.h"

//create global module instance
Cli cli;

static void cli_spinmotors() {
  //get motor indexes
  int mot_cnt = 0;
  uint8_t mots[OUT_SIZE] = {};
  for(int i = 0; i < OUT_SIZE; i++) {
    if(out.is_motor(i)) {
      mots[mot_cnt++] = i;
    }
  }

  //exit if no motors found
  if(mot_cnt == 0) {
     Serial.println("Spin motors - no motors configured, exiting");
     return;
  }

  //prompt for 'go', exit on anything else
  Serial.println("Spin motors - REMOVE PROPS - Type 'go' to continue, or enter to exit.");
  while(Serial.available()) Serial.read(); //clear input 
  char c;
  while(!Serial.available());
  c = Serial.read();
  if(c!='g') return;
  while(!Serial.available());
  c = Serial.read();
  if(c!='o') return;
  while(!Serial.available()); //get /n
  Serial.read();

  //clear input
  delay(1);
  Serial.flush();
  while(Serial.available()) Serial.read(); 

  //disable IMU interrupt
  void (*onUpdate_saved)(void) = imu.onUpdate;
  imu.onUpdate = nullptr;

  int i = -1;
  float speed = 0;
  const float maxspeed = 0.40;
  const float speedstep = maxspeed/3000; //3 second up / 3 second down
  int stage = 0;
  out.testmotor_enable(true);
  while(1) {
    switch(stage) {
    case 0: //next motor
      do {
        i++;
        if(i >= OUT_SIZE) i = 0;
      } while(!out.is_motor(i));
      Serial.printf("Spinning motor pin_out%d GPIO%d - press enter to exit\n", i, out.pin(i));
      Serial.flush();
      speed = 0;
      stage = 1;
      break;
    case 1: //spin up
      speed += speedstep;
      if(speed >= maxspeed) {
        stage = 2;
      }
      break;
    case 2: //spin down
      speed -= speedstep;
      if(speed <= 0) {
        speed = 0;
        stage = 0;
      }
      break;
    }

    //set outputs
    float val[mot_cnt] = {};
    val[i] = speed;
    for(int j = 0; j < mot_cnt; j++) {
      out.testmotor_set_output(mots[j], val[j]);
    }

    //exit on key
    delay(1);
    if(Serial.available()) break;
  }
  out.testmotor_enable(false);

  Serial.println("Spin motors - DONE");

  //enable IMU interrupt
  imu.onUpdate = onUpdate_saved;

  while(Serial.available()) Serial.read(); //clear input
}

static void cli_serial(int bus_id) {
  MF_Serial *ser = hal_get_ser_bus(bus_id);
  if(!ser) {
     Serial.printf("serial - Error: serial port %d not configured.\n", bus_id);
     return;
  }

  //disable IMU interrupt
  void (*onUpdate_saved)(void) = imu.onUpdate;
  imu.onUpdate = nullptr;

  int cnt = 0;
  Serial.println("serial - Dumping serial data, press enter to exit.");
  while(Serial.available()) Serial.read(); //clear input 
  while(!Serial.available()) {
    int d = ser->read();
    if(d>=0) {
      Serial.printf("%02X ",d);
      cnt++;
      if(cnt==32) {
        Serial.println();
        cnt=0;
      }
    }
  }
  while(Serial.available()) Serial.read(); //clear input

  Serial.println("\nserial - DONE");

  //enable IMU interrupt
  imu.onUpdate = onUpdate_saved;
}

static void cli_po() {
  Serial.printf("rcl.pwm%d:%d\t", 1, rcl.pwm[0]);
  Serial.printf("rcl.roll:%+.2f\t", rcl.roll);
  Serial.printf("ahr.gx:%+.2f\t", ahr.gx);
  Serial.printf("ahr.ax:%+.2f\t", ahr.ax);
  Serial.printf("mag.mx:%+.2f\t", mag.mx);
  Serial.printf("ahr.roll:%+.1f\t", ahr.roll);
  Serial.printf("pid.roll:%+.3f\t", pid.roll);
  Serial.printf("out.%c%d:%1.0f\t", out.type(0), 0, 100*out.get_output(0));
  Serial.printf("gps.sats:%d\t", (int)gps.sat);
  Serial.printf("imu.miss_cnt:%d\t", (int)(imu.interrupt_cnt-imu.update_cnt));
  Serial.printf("imu.upd_cnt:%d\t", (int)imu.update_cnt);
}

static void cli_ppwm() {
  for(int i=0;i<cfg.rcl_num_ch;i++) Serial.printf("pwm%d:%d\t",i+1,rcl.pwm[i]);
}

static void cli_prcl() {
  Serial.printf("rcl.throttle:%.2f\t", rcl.throttle);
  Serial.printf("roll:%+.2f\t", rcl.roll);
  Serial.printf("pitch:%+.2f\t", rcl.pitch);
  Serial.printf("yaw:%+.2f\t", rcl.yaw);
  Serial.printf("armed:%d\t", rcl.armed);
  Serial.printf("fm_sw:%d\t", rcl.flightmode_idx);
  Serial.printf("fm:%s\t", veh.flightmode_name());
  Serial.printf("connected:%d\t",rcl.connected());
  Serial.printf("upd_count:%d\t", rcl.update_count());

  static uint32_t ts_last = 0;
  static int cnt_last = 0;
  int cnt = rcl.update_count();
  uint32_t ts = micros();
  float dt = 1e-6 * (ts - ts_last);
  float hz = 0;
  if(dt>0) hz = (float)(cnt - cnt_last)/dt;
  cnt_last = cnt;
  ts_last = ts;
  Serial.printf("upd_freq:%d\t",(int)hz);
}

static void cli_pgyr() {
  Serial.printf("GYRO\tgx:%+.2f\tgy:%+.2f\tgz:%+.2f\t", ahr.gx, ahr.gy, ahr.gz);
}

static void cli_pacc() {
  Serial.printf("ACC\tax:%+.2f\tay:%+.2f\taz:%+.2f\t", ahr.ax, ahr.ay, ahr.az);
}

static void cli_pmag() {
  Serial.printf("MAG\tmx:%+.1f\tmy:%+.1f\tmz:%+.1f\ttotal:%+.1f\t", mag.mx, mag.my, mag.mz, sqrtf(mag.mx*mag.mx + mag.my*mag.my + mag.mz*mag.mz));
  float yaw = -atan2(mag.my, mag.mx) * 180 / M_PI;
  Serial.printf("compass:%+.1f\t", yaw);
}

static void cli_pahr() {
  const char* roll_str = (ahr.roll >= 0.0) ? "right" : "left";
  const char* pitch_str = (ahr.pitch >= 0.0) ? "up" : "down";
  const char* yaw_str = (ahr.yaw >= 0.0) ? "right" : "left";
  Serial.printf("AHRS\troll:%+.1f (roll %s)\tpitch:%+.1f (pitch %s)\tyaw:%+.1f (yaw %s)\t", ahr.roll, roll_str, ahr.pitch, pitch_str, ahr.yaw, yaw_str);
}

static void cli_pah() {
  Serial.printf("AHRS\troll:%+.1f\tpitch:%+.1f\tyaw:%+.1f\t", ahr.roll, ahr.pitch, ahr.yaw);
}

static void cli_ppid() {
  PidState pid_s;
  pid.topic.pull_latest(&pid_s); //get a consistent copy
  Serial.printf("PID\troll:%+.3f\t",pid_s.roll.sum);
  Serial.printf("pitch:%+.3f\t",pid_s.pitch.sum);
  Serial.printf("yaw:%+.3f\t",pid_s.yaw.sum);
  Serial.printf("throttle:%+.3f\t",pid_s.throttle.sum);
}

static void cli_ppidr() {
  PidState pid_s;
  pid.topic.pull_latest(&pid_s); //get a consistent copy
  Serial.printf("PIDROLL\tsum:%+.3f\t",pid_s.roll.sum);
  Serial.printf("P:%+.3f\t",pid_s.roll.p);
  Serial.printf("I:%+.3f\t",pid_s.roll.i);
  Serial.printf("D:%+.3f\t",pid_s.roll.d);
  Serial.printf("A:%+.3f\t",pid_s.roll.a);
  Serial.printf("B:%+.3f\t",pid_s.roll.b);
  Serial.printf("set:%+.3f\t",pid_s.roll.setpoint);
  Serial.printf("act:%+.3f\t",pid_s.roll.actual);
}

static void cli_ppidp() {
  PidState pid_s;
  pid.topic.pull_latest(&pid_s); //get a consistent copy
  Serial.printf("PIDPITCH\tsum:%+.3f\t",pid.pitch.sum);
  Serial.printf("P:%+.3f\t",pid_s.pitch.p);
  Serial.printf("I:%+.3f\t",pid_s.pitch.i);
  Serial.printf("D:%+.3f\t",pid_s.pitch.d);
  Serial.printf("A:%+.3f\t",pid_s.pitch.a);
  Serial.printf("B:%+.3f\t",pid_s.pitch.b);
  Serial.printf("set:%+.3f\t",pid_s.pitch.setpoint);
  Serial.printf("act:%+.3f\t",pid_s.pitch.actual);
}

static void cli_ppidy() {
  PidState pid_s;
  pid.topic.pull_latest(&pid_s); //get a consistent copy
  Serial.printf("PIDYAW\tsum:%+.3f\t",pid_s.yaw.sum);
  Serial.printf("P:%+.3f\t",pid_s.yaw.p);
  Serial.printf("I:%+.3f\t",pid_s.yaw.i);
  Serial.printf("D:%+.3f\t",pid_s.yaw.d);
  Serial.printf("A:%+.3f\t",pid_s.yaw.a);
  Serial.printf("B:%+.3f\t",pid_s.yaw.b);
  Serial.printf("set:%+.3f\t",pid_s.yaw.setpoint);
  Serial.printf("act:%+.3f\t",pid_s.yaw.actual);
}

static void cli_ppidt() {
  PidState pid_s;
  pid.topic.pull_latest(&pid_s); //get a consistent copy
  Serial.printf("PIDTHROTTLE\tsum:%+.3f\t",pid_s.throttle.sum);
  Serial.printf("P:%+.3f\t",pid_s.throttle.p);
  Serial.printf("I:%+.3f\t",pid_s.throttle.i);
  Serial.printf("D:%+.3f\t",pid_s.throttle.d);
  Serial.printf("A:%+.3f\t",pid_s.throttle.a);
  Serial.printf("B:%+.3f\t",pid_s.throttle.b);
  Serial.printf("set:%+.3f\t",pid_s.throttle.setpoint);
  Serial.printf("act:%+.3f\t",pid_s.throttle.actual);
}

static void cli_pout() {
  Serial.printf("OUT\tarmed:%d\t", (int)out.mode());
  for(int i = 0; i < OUT_SIZE; i++) {
    if(out.type(i)) {
      Serial.printf("%c%d:%1.0f\t", out.type(i), i, 100*out.get_output(i));
    }
  }
  for(int i = 0; i < OUT_SIZE; i++) {
    if(out.rpm(i) != -1) {
      Serial.printf("rpm%d:%d\t", i, out.rpm(i));
    }
  }
}

static void cli_pimu() {
  static uint32_t update_cnt_last = 0;
  static uint32_t ts_last = 0;
  uint32_t delta_upd = imu.update_cnt - update_cnt_last;
  update_cnt_last = imu.update_cnt;
  int miss_cnt = (int)imu.interrupt_cnt - imu.update_cnt;
  if(miss_cnt == 1) miss_cnt = 0; //ignore first miss, probably caused by interrupt_cnt updating before update_cnt
  uint32_t now = micros();
  uint32_t dt = now - ts_last;
  ts_last = now;

  int hz = imu.config.sample_rate;
  Serial.printf("IMU\tsamp_hz:%d\t", hz);

  if(dt == 0) dt = 1;
  Serial.printf("loop_hz:%.0f\t", (float)delta_upd/(dt*1e-6));

  int stat_cnt = 1;
  if(imu.stat_cnt > 0) stat_cnt = imu.stat_cnt;
  Serial.printf("latency_us:%d\t", (int)(imu.stat_latency / stat_cnt));
  Serial.printf("rt_io_us:%d\t", (int)(imu.stat_io_runtime / stat_cnt));
  Serial.printf("rt_imu_loop_us:%d\t", (int)((imu.stat_runtime - imu.stat_io_runtime) / stat_cnt));
  Serial.printf("rt_us:%d\t", (int)(imu.stat_runtime / stat_cnt));
  Serial.printf("rt%%:%d\t", (int)(imu.stat_runtime / stat_cnt) * hz / 10000);
  Serial.printf("rt_max_us:%d\t", (int)imu.stat_runtime_max);
  Serial.printf("rt_max%%:%d\t", (int)imu.stat_runtime_max * hz / 10000);
  Serial.printf("int_cnt:%d\t", (int)imu.interrupt_cnt);
  Serial.printf("miss_cnt:%d\t", (int)miss_cnt);
  imu.statReset();
}

static void cli_pbat() {
  Serial.printf("BAT\tv:%.2f\t",bat.v);
  Serial.printf("i:%+.2f\t",bat.i);
  Serial.printf("mah:%+.2f\t",bat.mah);
  Serial.printf("wh:%+.2f\t",bat.wh); 
}

static void cli_pbar() {
  Serial.printf("BAT\talt:%.2f\t", bar.alt);
  Serial.printf("press:%.1f\t", bar.press);
  Serial.printf("temp:%.2f\t", bar.temp);
  Serial.printf("agl:%+.2f\t", bar.alt - bar.ground_level);
}

static void cli_palt() {
  Serial.print("ALT\t");
  char s[100];
  alt.toString(s);
  Serial.print(s);
  Serial.printf("h:%+.2f\t", alt.getH());
  Serial.printf("v:%+.2f\t", alt.getV());
  Serial.printf("a:%+.2f\t", ahr.getAccelUp());
}

static void cli_pgps() {
  Serial.printf("GPS\ttime:%d\t", (int)gps.time);
  Serial.printf("fix:%d\t", (int)gps.fix);
  Serial.printf("sat:%d\t", (int)gps.sat);
  Serial.printf("lat:%d\t", (int)gps.lat);
  Serial.printf("lon:%d\t", (int)gps.lon);
  Serial.printf("alt:%.3f\t", (float)gps.alt/1000.0);
}

static void cli_prdr() {
  Serial.printf("RDR\tdist:%.3f\t", rdr.dist);
  Serial.printf("upd_cnt:%d\t", (int)rdr.update_cnt);  
}

static void cli_pofl() {
  Serial.printf("OFL\tdx:%.3f\t", ofl.dx);
  Serial.printf("dy:%.3f\t", ofl.dy);
  Serial.printf("x:%.3f\t", ofl.x);
  Serial.printf("y:%.3f\t", ofl.y);
  Serial.printf("upd_cnt:%d\t", (int)ofl.update_cnt);
}

struct cli_print_s {
  const char *cmd;
  const char *info;
  void (*function)(void);
};

#define CLI_PRINT_EXTERN_SIZE 10
uint8_t cli_print_extern_count = 0;
cli_print_s cli_print_extern[CLI_PRINT_EXTERN_SIZE] = {};
bool cli_print_flag_extern[CLI_PRINT_EXTERN_SIZE] = {false};
bool Cli::add_print_command(const char *cmd, const char *info, void (*function)(void)){
  if(cli_print_extern_count >= CLI_PRINT_EXTERN_SIZE) return false;
  cli_print_extern[cli_print_extern_count].cmd = cmd;
  cli_print_extern[cli_print_extern_count].info = info;
  cli_print_extern[cli_print_extern_count].function = function;
  cli_print_extern_count++;
  return true;
};

#define CLI_PRINT_FLAG_COUNT 21

static const struct cli_print_s cli_print_options[CLI_PRINT_FLAG_COUNT] = {
  {"po",     "Overview", cli_po},
  {"ppwm",   "Radio pwm (expected: 1000 to 2000)", cli_ppwm},
  {"prcl",   "Scaled radio (expected: -1 to 1)", cli_prcl},
  {"pimu",   "IMU loop timing (expected: miss% <= 1)", cli_pimu},
  {"pgyr",   "Filtered gyro (expected: -250 to 250, 0 at rest)", cli_pgyr},
  {"pacc",   "Filtered accelerometer (expected: -2 to 2; when level: x=0,y=0,z=1)", cli_pacc},
  {"pmag",   "Filtered magnetometer (expected: -300 to 300)", cli_pmag},
  {"pahr",   "AHRS roll, pitch, and yaw in human friendly format (expected: degrees, 0 when level)", cli_pahr},
  {"pah",    "AHRS roll, pitch, and yaw (expected: degrees, 0 when level)", cli_pah},
  {"ppid",   "PID output (expected: -1 to 1)", cli_ppid},
  {"ppidr",  "PID Roll output (expected: -1 to 1)", cli_ppidr},
  {"ppidp",  "PID Pitch output (expected: -1 to 1)", cli_ppidp},
  {"ppidy",  "PID Yaw output (expected: -1 to 1)", cli_ppidy},
  {"ppidt",  "PID Throttle output (expected: -1 to 1)", cli_ppidt},
  {"pout",   "Motor/servo output (expected: 0 to 1)", cli_pout},
  {"pbat",   "Battery voltage, current, Ah used and Wh used", cli_pbat},
  {"pbar",   "Barometer", cli_pbar},
  {"palt",   "Altitude estimator", cli_palt},
  {"pgps",   "GPS", cli_pgps},
  {"prdr",   "Radar", cli_prdr},
  {"pofl",   "Optical Flow", cli_pofl},
};
bool cli_print_flag[CLI_PRINT_FLAG_COUNT] = {false};

Cli::Cli() {
  cli_print_all(false);
}

void Cli::begin() {
  ser_buf_size = Serial.availableForWrite();
}

void Cli::banner() {
  if(ser_buf_size < 255) {
    Serial.printf("CLI: WARNING Serial transmit buffer (%d bytes) is small\n", ser_buf_size);
  }
}

bool Cli::update_MODE_CLI() {
  bool rv = false;
  //process chars from Serial
  int n = Serial.available(); //Note: Serial.read(&c,1) does not work on all platforms
  for(int i = 0; i < n; i++) {  
    uint8_t c = Serial.read();
    //---------------------------
    // MSP check
    //---------------------------
    //switch to MSP if we received a MSP command
    if(Msp::process_byte(c, false)) {
      cli_mode = MODE_MSP;
      return update_MODE_MSP(); //process remaining chars in serial buffer
    }

    //---------------------------
    // MAVLINK check
    //---------------------------
    //check for MAVLINK v1,v2 protocol header byte, start mavlink parser
    if((c == 0xFD || c == 0xFE) && !mavlink) {
      auto ser = &Serial;
      MF_Serial *ser_bus = new MF_SerialPtrWrapper<decltype(ser)>( ser );
      mavlink = new RclGizmoMavlink(ser_bus, -1, nullptr);
    }
    //switch to MAVLINK as soon as we received a MAVLINK message
    if(mavlink && mavlink->process_char(c) != RclGizmoMavlink::process_result_enum::NONE) {
      cli_mode = MODE_MAV;
      return update_MODE_MAV(); //process remaining chars in serial buffer
    }

    //---------------------------
    // process CLI command
    //---------------------------
    if(cmd_process_char(c)) rv = true;
  }
  //handle output for pxxx commands
  cli_print_loop();

  return rv;
}

bool Cli::update_MODE_MSP() {
  static bool last_msp_rv = false;
  bool rv = false;

  //process chars from Serial
  int n = Serial.available(); //Note: Serial.read(&c,1) does not work on all platforms
  for(int i = 0; i < n; i++) {  
    uint8_t c = Serial.read();

    //switch back to MODE_CLI if a '#' is received immediately after the last msp command
    if(last_msp_rv == true && c == '#') {
      cli_mode = MODE_CLI;
      last_msp_rv = false;
      return Cli::update_MODE_CLI();
    }

    //process MSP character
    rv = Msp::process_byte(c, true);
    last_msp_rv = rv;
  }
  return rv;
}

bool Cli::update_MODE_MAV() {
  return mavlink->update();
}

//returns true if a command was processed (even an invalid one)
bool Cli::update() {
  runtimeTrace.start();

  bool updated = false;
  switch(cli_mode) {
    case MODE_CLI: 
      updated = update_MODE_CLI();
      break;
    case MODE_MSP: 
      updated =  update_MODE_MSP();
      break;
    case MODE_MAV: 
      updated = update_MODE_MAV();
      break;
  }
  Serial.flush(); //for TinyUSB

  runtimeTrace.stop(updated);
  if(updated) updated_cnt++;
  return updated;
}

void Cli::help() {
  Serial.println(MADFLIGHT_VERSION " on " HAL_ARDUINO_STR);

  Serial.printf(
  "-- TOOLS --\n"
  "help or ?           This info\n"
  "ps                  Task list\n"
  "res                 Resources (pinout, busses)\n"
  "i2c                 I2C scan\n"
  "serial <bus_id>     Dump serial data\n"
  "spinmotors          Spin each motor\n"
  "reboot              Reboot flight controller\n"
  "-- PRINT --\n"
  "q / poff            Printing off\n"
  "pall                Print all\n"
  );
  for(int i=0;i<CLI_PRINT_FLAG_COUNT;i++) {
    Serial.print(cli_print_options[i].cmd);
    for(int j = strlen(cli_print_options[i].cmd); j < 19; j++) {
      Serial.print(' ');
    }
    Serial.print(' ');
    Serial.print(cli_print_options[i].info);
    Serial.println();
  }
  for(int i=0;i<cli_print_extern_count;i++) {
    Serial.print(cli_print_extern[i].cmd);
    for(int j = strlen(cli_print_extern[i].cmd); j < 19; j++) {
      Serial.print(' ');
    }
    Serial.print(' ');
    Serial.print(cli_print_extern[i].info);
    Serial.println();
  }  
  Serial.printf(
  "-- BLACK BOX --\n"
  "bbstart             Start logging\n"
  "bbstop              Stop logging\n"
  "bbls                List files\n"
  "bberase             Erase bb device\n"
  "bbbench             Benchmark\n"
  "bbinfo              Info\n"
  "-- PARAMETERS --\n"
  "set <filter> <val>  Set parameter(s)\n"
  "dump <filter>       List parameters\n"
  "diff <filter>       List parameter changes from default\n"
  "defaults <filter>   Reset parameters to defaults\n"
  "save                Save parameters and reboot\n"
  "-- CALIBRATE --\n"
  "calinfo             Sensor info\n"
  "calimu              Calibrate IMU error\n"
  "calmag              Calibrate magnetometer\n"
  "calradio            Calibrate RC Radio\n"
  );
}

//========================================================================================================================//
//                                          COMMAND PROCESSING                                                            //
//========================================================================================================================//
void Cli::cmd_execute_batch(const char *batch) {
  cmd_clear();
  int pos = 0;
  int c;
  while( (c = batch[pos]) ) {
    cmd_process_char(c);
    pos++;
  }
  if(c != '\n' && c != '\r') cmd_process_char('\n'); //send terminating return
  cmd_clear();
}

void Cli::cmd_clear() {
  cmdline = "";
  prev_c = 0;;
}


//returns true if a command was processed (even an invalid one)
bool Cli::cmd_process_char(char c) {
  bool rv = false;
  if ( (c=='\r' && prev_c=='\n') || (c=='\n' && prev_c=='\r') ) {
    //ignore \r\n, \n\r
  }else if ( (c=='\r' || c=='\n') ) {
    processCmd();
    rv = true;
  }else if (c == 0x08) { //backspace
    if(cmdline.length() > 0) {
      cmdline = cmdline.substring(0, cmdline.length() - 1);
    }
  }else{
    cmdline += c;
  }
  prev_c = c;
  return rv;
}

String Cli::getCmdPart(uint32_t &pos) {
  String part = "";
  while(pos < cmdline.length() && cmdline[pos] == ' ') pos++;
  while(pos < cmdline.length() && cmdline[pos] != ' ') {
    part += cmdline[pos];
    pos++;
  }
  return part;
}

void Cli::processCmd() {
  //remove comment
  int comment_pos = cmdline.indexOf('#');
  if(comment_pos >= 0) cmdline = cmdline.substring(0,comment_pos);
  //execute cmd
  uint32_t pos = 0;
  String cmd = getCmdPart(pos);
  String arg1 = getCmdPart(pos);
  String arg2 = getCmdPart(pos);
  cmd.toLowerCase();
  cmd.trim();
  cmdline = ""; //clear command line

  //Serial.println( "> " + cmd + " " + arg1 + " " + arg2 );
  this->executeCmd(cmd, arg1, arg2);
  Serial.print( "> ");
  Serial.flush();  
}

void Cli::executeCmd(String cmd, String arg1, String arg2) {
  //process external print commands
  for (int i=0;i<cli_print_extern_count;i++) {
    if (strcmp(cmd.c_str(), cli_print_extern[i].cmd) == 0) {
      cli_print_flag_extern[i] = !cli_print_flag_extern[i];
      return;
    }
  }

  //process print commands
  for (int i=0;i<CLI_PRINT_FLAG_COUNT;i++) {
    if (strcmp(cmd.c_str(), cli_print_options[i].cmd) == 0) {
      cli_print_flag[i] = !cli_print_flag[i];
      return;
    }
  }

  //call user defined commands, skip futher processing if true was returned
  if(cli_execute) {
    if(cli_execute(cmd, arg1, arg2)) return;
  }

  if (cmd == "help" || cmd == "?") {
    help();
  }else if (cmd == "i2c") {
    print_i2cScan();
  }else if (cmd == "reboot") {
    hal_reboot();
  }else if (cmd == "q" || cmd == "poff") {
    cli_print_all(false);
  }else if (cmd == "pall") {
    cli_print_all(true);
  }else if (cmd == "bbstart") {
    bbx.start();
  }else if (cmd == "bbstop") {
    bbx.stop();
  }else if (cmd == "bbls") {
    bbx.dir();
  }else if (cmd == "bberase") {
    bbx.erase();
  }else if (cmd == "bbinfo") {
    bbx.info();
  }else if (cmd == "bbbench") {
    bbx.bench();
  }else if (cmd == "set") {
    cfg.cli_set_param(arg1, arg2);
  }else if (cmd == "dump") {
    cfg.cli_dump(arg1.c_str());
  }else if (cmd == "diff") {
    cfg.cli_diff(arg1.c_str());
  }else if (cmd == "defaults") {
    cfg.cli_defaults(arg1.c_str());
    if(arg1 != "") cfg.cli_dump(arg1.c_str());
    Serial.println("Parameters reset to defaults, type 'save' to save... ");
  }else if (cmd == "save") {
    cfg.cli_save();
  }else if (cmd == "calinfo") {
    cli_print_all(false);
    calibrate_info(arg1.toInt());
  }else if (cmd == "calimu") {
    cli_print_all(false);
    calibrate_IMU();
  }else if (cmd == "calmag") {
    mag.cli_calibrate();
  }else if (cmd == "calradio") {
    cli_print_all(false);
    RclCalibrate::calibrate();
  }else if (cmd == "ps") {
    ps();
  }else if (cmd == "serial") {
    cli_serial(arg1.toInt());
  }else if (cmd == "spinmotors") {
    cli_spinmotors();
  }else if (cmd == "res") {
    print_resources();
  }else if (cmd != "") {
    Serial.println("ERROR Unknown command - Type help for help");
  }
}

//========================================================================================================================//
//                                          HELPERS                                                                       //
//========================================================================================================================//
void Cli::print_i2cScan() {
  for(int bus_i=0;bus_i<4;bus_i++) {
    MF_I2C *i2c = hal_get_i2c_bus(bus_i);
    if(i2c) {
      //set clock speed to 100k
      uint32_t clock = i2c->getClock();
      i2c->setClock(100000);
      //do scan
      Serial.printf("I2C: Scanning i2c_bus:%d - ", bus_i);
      int count = 0;
      for (byte i = 1; i < 128; i++) {
        i2c->beginTransmission(i);          // Begin I2C transmission Address (i)
        if (i2c->endTransmission() == 0) {  // Receive 0 = success (ACK response) 
          Serial.printf("0x%02X(%d) ", i, i);
          count++;
        }
      }
      Serial.printf("- Found %d device(s)\n", count);
      //restore original clock speed
      i2c->setClock(clock);
    }
  }
}

//========================================================================================================================//
//                                          CALIBRATION FUNCTIONS                                                         //
//========================================================================================================================//
void Cli::calibrate_gyro() {
  Serial.println("Calibrating gyro, don't move vehicle, this takes a couple of seconds...");
  calibrate_IMU2(true);
}

void Cli::calibrate_IMU() {
  Serial.println("Calibrating IMU, don't move vehicle, this takes a couple of seconds...");
  calibrate_IMU2(false);
}

//Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
void Cli::calibrate_IMU2(bool gyro_only) {
  auto imu_sub = MsgSubscription<ImuState>("calimu", &imu.topic);
  auto bar_sub = MsgSubscription<BarState>("calimu", &bar.topic);
  ImuState imu_s;
  BarState bar_s;

  //Read IMU values, and average the readings
  const int timeout = 3000;
  Stat alt, a[3], g[3];
  const char axisname[3] = {'x','y','z'};
  float *acal = &cfg.imu_cal_ax; //current a calibration
  float *gcal = &cfg.imu_cal_gx; //current g calibration
  uint32_t ts = millis();
  while(millis() - ts < timeout) {
    if(bar_sub.pull_next(&bar_s)) {
      alt.append(bar_s.alt);
    }
    if(imu_sub.pull_next(&imu_s)) {
      imu.convert_to_raw(&imu_s.gx, &imu_s.ax);
      a[0].append(imu_s.ax);
      a[1].append(imu_s.ay);
      a[2].append(imu_s.az);
      g[0].append(imu_s.gx);
      g[1].append(imu_s.gy);
      g[2].append(imu_s.gz); 
    }
  }

  //save ground level
  if(alt.n > 0) {
    bar.ground_level = alt.mean();
    Serial.printf("BAR: Ground level: %.3fm (%d samples, stdev: %.3fm)\n", alt.mean(), alt.n, alt.std());
  }

  float aoff[3], goff[3]; //measured offsets
  for(int axis = 0; axis < 3; axis++) {
    aoff[axis] = a[axis].mean();
    //remove gravitation
    if(aoff[axis] > +0.8) aoff[axis] -= 1.0;
    if(aoff[axis] < -0.8) aoff[axis] += 1.0;
    goff[axis] = g[axis].mean();
  }

  for(int axis = 0; axis < 3; axis++) {
    Serial.printf("set imu_cal_g%c %+f #config was %+f\n", axisname[axis], goff[axis], gcal[axis]);
  }

  bool apply_gyro = true;
  
  if (gyro_only) {
    //only apply reasonable gyro errors
    float gtol = 10; //in deg/s
    apply_gyro = ( 
      -gtol < goff[0] && goff[0] < gtol  &&  
      -gtol < goff[1] && goff[1] < gtol  &&  
      -gtol < goff[2] && goff[2] < gtol 
    );
  }else{
    for(int axis = 0; axis < 3; axis++) {
      Serial.printf("set imu_cal_a%c %+f #config was %+f\n", axisname[axis], aoff[axis], acal[axis]);
    }
  }
/*
    //only apply reasonable acc errors
    float atol = 0.1;
    float aztol = 0.2;
    apply_acc = ( -atol < axerr && axerr < atol  &&  -atol < ayerr && ayerr < atol  &&  -aztol < azerr && azerr < aztol );
*/
  
  if (apply_gyro) {
    for(int axis = 0; axis < 3; axis++) {
      gcal[axis] = goff[axis];
    }
  }else{
     Serial.println("=== Not applying gyro correction, out of tolerance ===");
  }

  if (!gyro_only) {
    for(int axis = 0; axis < 3; axis++) {
      acal[axis] = aoff[axis];
    }
  }
  
  Serial.println("Type 'save' to save these values to flash");
}

void Cli::calibrate_info(int seconds) {
  bool report_spikes = (seconds >= 0);
  if(seconds < 0) seconds = -seconds;
  if(seconds == 0) seconds = 3;
  Serial.printf("Gathering sensor statistics, please wait %d seconds ...\n\n", seconds);

  //keep up to 1000 samples of history
  const int hist = 1000;
  Stat ax(hist), ay(hist), az(hist);
  Stat gx(hist), gy(hist), gz(hist);
  Stat sp(hist), sa(hist), st(hist);
  Stat mx(hist), my(hist), mz(hist);

  float bp_last = 0;
  float ba_last = 0;
  float bt_last = 0;

  float mx_last = 0;
  float my_last = 0;
  float mz_last = 0;

  ImuState imu_s;
  BarState bar_s;
  MagState mag_s;
  auto imu_sub = MsgSubscription<ImuState>("calinfo", &imu.topic);
  auto bar_sub = MsgSubscription<BarState>("calinfo", &bar.topic);
  auto mag_sub = MsgSubscription<MagState>("calinfo", &mag.topic);

  uint32_t ts = micros();
  while((uint32_t)micros() - ts < (uint32_t)1000000*seconds) {
    if(imu_sub.pull_next(&imu_s)) {
      ax.append(imu_s.ax);
      ay.append(imu_s.ay);
      az.append(imu_s.az);
      gx.append(imu_s.gx);
      gy.append(imu_s.gy);
      gz.append(imu_s.gz);
    }

    if(bar_sub.pull_next(&bar_s) && ((bar_s.press != bp_last) || (bar_s.alt != ba_last) || (bar_s.temp != bt_last))) {
      //only record if at least one value is changed
      sp.append(bar_s.press);
      sa.append(bar_s.alt);
      st.append(bar_s.temp);
      bp_last = bar_s.press;
      ba_last = bar_s.alt;
      bt_last = bar_s.temp;
    }

    if(mag_sub.pull_next(&mag_s) && ((mag_s.mx != mx_last) || (mag_s.my != my_last) || (mag_s.mz != mz_last))) {
      //only record if at least one value is changed
      mx.append(mag_s.mx);
      my.append(mag_s.my);
      mz.append(mag_s.mz);
      mx_last = mag_s.mx;
      my_last = mag_s.my;
      mz_last = mag_s.mz;
    }
  } 

  if(report_spikes) {
    Serial.printf("### SENSOR SPIKE REPORT - " MADFLIGHT_VERSION " ###\n\n");
    Serial.printf("=== %s Gyro %s ===\n", imu.name(), (imu.config.uses_i2c?"I2C":"SPI"));
    gx.print_spikes("gx[deg/s]     ");
    gy.print_spikes("gy[deg/s]     ");
    gz.print_spikes("gz[deg/s]     ");
    Serial.printf("=== %s Accelerometer %s ===\n", imu.name(), (imu.config.uses_i2c?"I2C":"SPI"));
    ax.print_spikes("ax[g]         ");
    ay.print_spikes("ay[g]         ");
    az.print_spikes("az[g]         ");
    if(sp.n > 0) {
      Serial.printf("=== %s Barometer I2C ===\n", bar.name());
      sa.print_spikes("Altitude[m]   ");
      sp.print_spikes("Pressure[Pa]  ");
      st.print_spikes("Temperature[C]");
    }
    if(mx.n > 0) {
      Serial.printf("=== %s Magnetometer I2C ===\n", mag.name());
      mx.print_spikes("mx[uT]        ");
      my.print_spikes("my[uT]        ");
      mz.print_spikes("mz[uT]        ");
    }
    Serial.println();
  }

  Serial.printf("### SENSOR STATISTICS REPORT - " MADFLIGHT_VERSION " ###\n\n");
  Serial.printf("=== %s Gyro %s ===\n", imu.name(), (imu.config.uses_i2c ? "I2C" : "SPI"));
  gx.print("gx[deg/s]     ", seconds);
  gy.print("gy[deg/s]     ", seconds);
  gz.print("gz[deg/s]     ", seconds);
  Serial.printf("=== %s Accelerometer %s ===\n", imu.name(), (imu.config.uses_i2c ? "I2C" : "SPI"));
  ax.print("ax[g]         ", seconds);
  ay.print("ay[g]         ", seconds);
  az.print("az[g]         ", seconds);
  if(sp.n > 0) {
    Serial.printf("=== %s Barometer I2C ===\n", bar.name());
    sa.print("Altitude[m]   ", seconds);
    sp.print("Pressure[Pa]  ", seconds);
    st.print("Temperature[C]", seconds);
  }
  if(mx.n > 0) {
    float f = sqrt(mx.mean() * mx.mean() + my.mean() * my.mean() + mz.mean() * mz.mean());
    Serial.printf("=== %s Magnetometer I2C - Field Strength: %.2f uT===\n", mag.name(), f);
    mx.print("mx[uT]        ", seconds);
    my.print("my[uT]        ", seconds);
    mz.print("mz[uT]        ", seconds);
  }

  Serial.println("\ncalinfo completed\n");
}

//========================================================================================================================//
//                                                PRINT FUNCTIONS                                                         //
//========================================================================================================================//
void Cli::cli_print_all(bool val) {
  for(int i=0;i<cli_print_extern_count;i++) cli_print_flag_extern[i] = val;
  for(int i=0;i<CLI_PRINT_FLAG_COUNT;i++) cli_print_flag[i] = val;
}

void Cli::cli_print_loop() {
  uint32_t cli_print_interval = 100000; //Print data at cli_print_interval microseconds
  if (micros() - cli_print_time > cli_print_interval) {
    cli_print_time = micros();
    bool cli_print_need_newline = false;
    //Serial.printf("loop_time:%d\t",loop_time); //print loop time stamp
    for(int i=0;i<cli_print_extern_count;i++) {
      if(cli_print_flag_extern[i]) {
        cli_print_extern[i].function();
        cli_print_need_newline = true;
      }
    }
    for (int i=0;i<CLI_PRINT_FLAG_COUNT;i++) {
      if (cli_print_flag[i]) {
        cli_print_options[i].function();
        cli_print_need_newline = true;
      }
    }
    if (cli_print_need_newline) Serial.println();
    imu.stat_runtime_max = 0; //reset maximum runtime
  }
}

void Cli::ps() {
  MsgBroker::top();
  RuntimeTraceGroup::print();
  freertos_ps();
  Serial.println();
  hal_meminfo();
}

void Cli::print_resources() {
  // MEMINFO 
  hal_meminfo();

  // DMA, PIO
  hal_print_resources();

  //serial, i2c, and spi busses
  hal_print_businfo();

  //i2c clock speeds
  for(int i=0;i<2;i++) {
    MF_I2C* i2c = hal_get_i2c_bus(i);
    if(i2c) {
      Serial.printf("I2C: bus:%d clock:%d\n", i, (int)i2c->getClock());
    }
  }

  //pinout sorted by gpio number
  cfg.printPins();
}
