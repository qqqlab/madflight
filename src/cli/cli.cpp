#include "cli.h"
#include "../madflight_modules.h"

#include "msp/msp.h"
#include "cli_RclCalibrate.h"

#include "stat.h"
#include "FreeRTOS_ps.h"

//create global module instance
Cli cli;

static void cli_spinmotors() {
  int mot_cnt = 0;
  for(int i=0;i<OUT_SIZE;i++) {
    if(out.getType(i) == 'M') mot_cnt++;
  }
  if(mot_cnt==0) {
     Serial.println("Spin motors - no motors configured, exiting");
     return;
  }
  Serial.println("Spin motors - REMOVE PROPS - Type 'go' to continue, or enter to exit.");
  while(Serial.available()) Serial.read(); //clear input 
  char c;
  while(!Serial.available());
  c = Serial.read();
  if(c!='g') return;
  while(!Serial.available());
  c = Serial.read();
  if(c!='o') return;

  while(Serial.available()) Serial.read(); //clear input

  //disable IMU interrupt
  void (*onUpdate_saved)(void) = imu.onUpdate;
  imu.onUpdate = nullptr;

  out.armed = true;
  int i = -1;
  float speed = 0;
  const float maxspeed = 0.40;
  const float speedstep = maxspeed/3000; //3 second up / 3 second down
  int stage = 0;
  while(!Serial.available()) {
    switch(stage) {
    case 0: //next motor
      do {
        i++;
        if(i>=OUT_SIZE) i = 0;
      } while(out.getType(i) != 'M');
      Serial.printf("Spinning motor pin_out%d - press enter to exit\n", i);
      speed = 0;
      out.set(i, speed);
      stage = 1;
      break;
    case 1: //spin up
      speed += speedstep;
      if(speed<maxspeed) {
        out.set(i, speed);
      }else{
        stage = 2;
      }
      break;
    case 2: //spin down
      speed -= speedstep;
      if(speed>0) {
        out.set(i, speed);
      }else{
        speed = 0;
        out.set(i, speed);
        stage = 0;
      }
      break;
    }
    delay(1);
  }
  out.armed = false;

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
  Serial.printf("ahr.mx:%+.2f\t", ahr.mx);
  Serial.printf("ahr.roll:%+.1f\t", ahr.roll);
  Serial.printf("pid.roll:%+.3f\t", pid.roll);
  Serial.printf("out.%c%d%%:%1.0f\t", out.getType(0), 0, 100*out.get(0));
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
  Serial.printf("flightmode:%d\t", rcl.flightmode);
  Serial.printf("upd_count:%d\t", rcl.update_count());
  Serial.printf("connected:%d\t",rcl.connected());
}

static void cli_pgyr() {
  Serial.printf("gx:%+.2f\tgy:%+.2f\tgz:%+.2f\t", ahr.gx, ahr.gy, ahr.gz);
}

static void cli_pacc() {
  Serial.printf("ax:%+.2f\tay:%+.2f\taz:%+.2f\t", ahr.ax, ahr.ay, ahr.az);
}

static void cli_pmag() {
  Serial.printf("mx:%+.2f\tmy:%+.2f\tmz:%+.2f\t", ahr.mx, ahr.my, ahr.mz); 
}

static void cli_pahr() {
  const char* roll_str = (ahr.roll >= 0.0) ? "right" : "left";
  const char* pitch_str = (ahr.pitch >= 0.0) ? "up" : "down";
  const char* yaw_str = (ahr.yaw >= 0.0) ? "right" : "left";
  Serial.printf("roll:%+.1f (roll %s)\tpitch:%+.1f (pitch %s)\tyaw:%+.1f (yaw %s)\t", ahr.roll, roll_str, ahr.pitch, pitch_str, ahr.yaw, yaw_str);
}

static void cli_pah() {
  Serial.printf("roll:%+.1f\tpitch:%+.1f\tyaw:%+.1f\t", ahr.roll, ahr.pitch, ahr.yaw);
}

static void cli_ppid() {
  Serial.printf("pid.roll:%+.3f\t",pid.roll);
  Serial.printf("pitch:%+.3f\t",pid.pitch);
  Serial.printf("yaw:%+.3f\t",pid.yaw);
}

static void cli_pout() {
  Serial.printf("out.armed:%d\t", out.armed);
  for(int i=0;i<OUT_SIZE;i++) {
    if(out.getType(i)) {
      Serial.printf("%c%d%%:%1.0f\t", out.getType(i), i, 100*out.get(i));
      if(out.eperiodEnabled[i]) {
        Serial.printf("rpm%d:%d\t", i, out.rpm(i));
      }
    }
  }
}

static void cli_pimu() {
  static uint32_t update_cnt_last = 0;
  static uint32_t ts_last = 0;
  uint32_t delta_upd = imu.update_cnt - update_cnt_last;
  update_cnt_last = imu.update_cnt;
  int miss_cnt = (int)imu.interrupt_cnt - imu.update_cnt;
  if(miss_cnt == 1) miss_cnt = 0; //ignore first miss, probably caused by imu_loop still running
  uint32_t now = micros();
  uint32_t dt = now - ts_last;
  ts_last = now;

  int hz = imu.config.sample_rate;
  Serial.printf("samp_hz:%d\t", hz);

  if(dt == 0) dt = 1;
  Serial.printf("imu_loop_hz:%.0f\t", (float)delta_upd/(dt*1e-6));

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
  Serial.printf("bat.v:%.2f\t",bat.v);
  Serial.printf("bat.i:%+.2f\t",bat.i);
  Serial.printf("bat.mah:%+.2f\t",bat.mah);
  Serial.printf("bat.wh:%+.2f\t",bat.wh); 
}

static void cli_pbar() {
  Serial.printf("bar.alt:%.2f\t", bar.alt);
  Serial.printf("press:%.1f\t", bar.press);
  Serial.printf("temp:%.2f\t", bar.temp);
}

static void cli_pgps() {
  Serial.printf("gps.time:%d\t", (int)gps.time);
  Serial.printf("fix:%d\t", (int)gps.fix);
  Serial.printf("sat:%d\t", (int)gps.sat);
  Serial.printf("lat:%d\t", (int)gps.lat);
  Serial.printf("lon:%d\t", (int)gps.lon);
  Serial.printf("alt:%.3f\t", (float)gps.alt/1000.0);
}

static void cli_palt() {
  char s[100];
  alt.toString(s);
  Serial.print(s);
  Serial.printf("bar.alt:%.2f\t", bar.alt);
  Serial.printf("ahr.aup:%.2f\t", ahr.getAccelUp());
}

static void cli_prdr() {
  Serial.printf("rdr.dist:%.3f\t", rdr.dist);
  Serial.printf("upd_cnt:%d\t", (int)rdr.update_cnt);  
}

static void cli_pofl() {
  Serial.printf("ofl.dx:%.3f\t", ofl.dx);
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

#define CLI_PRINT_FLAG_COUNT 17

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
  {"pout",   "Motor/servo output (expected: 0 to 1)", cli_pout},
  {"pbat",   "Battery voltage, current, Ah used and Wh used", cli_pbat},
  {"pbar",   "Barometer", cli_pbar},
  {"palt",   "Altitude estimator", cli_palt},
  {"pgps",   "GPS", cli_pgps},
  {"prdr",   "Radar", cli_prdr},
  {"pofl",   "Optical Flow", cli_pofl},
};
bool cli_print_flag[CLI_PRINT_FLAG_COUNT] = {false};



void Cli::setup() {
  cli_print_all(false);
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
    if(Msp::process_byte(c)) {
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

    //switch back to MODE_CLI if a single '#' is received directrly after the last msp command
    if(last_msp_rv == true && c == '#') {
      cli_mode = MODE_CLI;
      last_msp_rv = false;
      return Cli::update_MODE_CLI();
    }

    //process MSP character
    rv = Msp::process_byte(c);
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
  return updated;
}

void Cli::begin() {
  Serial.println("CLI: Command Line Interface Started - Type help for help");
}

void Cli::help() {
  Serial.println(MADFLIGHT_VERSION " on " HAL_ARDUINO_STR);

  Serial.printf(
  "-- TOOLS --\n"
  "help or ?           This info\n"
  "ps                  Task list\n"
  "i2c                 I2C scan\n"
  "serial <bus_id>     Dump serial data\n"
  "spinmotors          Spin each motor\n"
  "reboot              Reboot flight controller\n"
  "-- PRINT --\n"
  "poff                Printing off\n"
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
  "-- CONFIG --\n"
  "set <name> <value>  Set config parameter\n"
  "dump <filter>       List config\n"
  "diff <filter>       List config changes from default\n"
  "save                Save config and reboot\n"
  "defaults            Reset to defaults and reboot\n"
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

  Serial.println( "> " + cmd + " " + arg1 + " " + arg2 );
  this->executeCmd(cmd, arg1, arg2);
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

  if (cmd=="help" || cmd=="?") {
    help();
  }else if (cmd == "i2c") {
    print_i2cScan();
  }else if (cmd == "reboot") {
    hal_reboot();
  }else if (cmd == "poff") {
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
    cfg.setParam(arg1, arg2);
  }else if (cmd == "dump") {
    cfg.cli_dump(arg1.c_str());
  }else if (cmd == "diff") {
    cfg.cli_diff(arg1.c_str());
  }else if (cmd == "defaults") {
    Serial.println("Resetting to defaults and rebooting, please wait... ");
    cfg.clear();
    cfg.writeToEeprom();
    delay(1000);
    hal_reboot();
  }else if (cmd == "save") {
    Serial.println("Saving and rebooting, please wait... ");
    cfg.writeToEeprom();
    delay(1000);
    hal_reboot();
  }else if (cmd == "calinfo") {
    cli_print_all(false);
    calibrate_info(arg1.toInt());
  }else if (cmd == "calimu") {
    cli_print_all(false);
    calibrate_IMU();
  }else if (cmd == "calmag") {
    calibrate_Magnetometer();
  }else if (cmd == "calradio") {
    cli_print_all(false);
    RclCalibrate::calibrate();
  }else if (cmd == "ps") {
    ps();
  }else if (cmd == "serial") {
    cli_serial(arg1.toInt());
  }else if (cmd == "spinmotors") {
    cli_spinmotors();
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
  //Read IMU values, and average the readings
  int cnt = 3000;
  float axerr = 0;
  float ayerr = 0;
  float azerr = 0;
  float gxerr = 0;
  float gyerr = 0;
  float gzerr = 0;
  int bar_cnt = 0;
  float bar_alt = 0;
  for(int i=0; i<cnt; i++) {
    if(bar.update()) {
      bar_cnt++;
      bar_alt += bar.alt;
    }
    imu.waitNewSample();
    axerr += imu.ax;
    ayerr += imu.ay;
    azerr += imu.az;
    gxerr += imu.gx;
    gyerr += imu.gy;
    gzerr += imu.gz;
  }
  axerr /= cnt;
  ayerr /= cnt;
  azerr /= cnt;
  gxerr /= cnt;
  gyerr /= cnt;
  gzerr /= cnt;

  //remove gravitation
  azerr -= 1.0;

  //save ground level
  if(bar_cnt > 0) {
    bar.ground_level = bar_alt / bar_cnt;
    Serial.printf("BAR: Ground level: %f m (%d samples)\n", bar.ground_level, bar_cnt);
  }

  Serial.printf("set imu_cal_gx %+f #config was %+f\n", gxerr, cfg.imu_cal_gx);
  Serial.printf("set imu_cal_gy %+f #config was %+f\n", gyerr, cfg.imu_cal_gy);
  Serial.printf("set imu_cal_gz %+f #config was %+f\n", gzerr, cfg.imu_cal_gz);

  bool apply_gyro = true;
  
  if (gyro_only) {
    //only apply reasonable gyro errors
    float gtol = 10;
    apply_gyro = ( -gtol < gxerr && gxerr < gtol  &&  -gtol < gyerr && gyerr < gtol  &&  -gtol < gzerr && gzerr < gtol );
  }else{
    Serial.printf("set imu_cal_ax %+f #config was %+f\n", axerr, cfg.imu_cal_ax);
    Serial.printf("set imu_cal_ay %+f #config was %+f\n", ayerr, cfg.imu_cal_ay);
    Serial.printf("set imu_cal_az %+f #config was %+f\n", azerr, cfg.imu_cal_az);
  }
/*
    //only apply reasonable acc errors
    float atol = 0.1;
    float aztol = 0.2;
    apply_acc = ( -atol < axerr && axerr < atol  &&  -atol < ayerr && ayerr < atol  &&  -aztol < azerr && azerr < aztol );
*/
  
  if (apply_gyro) {
    cfg.imu_cal_gx = gxerr;
    cfg.imu_cal_gy = gyerr;
    cfg.imu_cal_gz = gzerr;
  }else{
     Serial.println("=== Not applying gyro correction, out of tolerance ===");
  }

  if (!gyro_only) {
    cfg.imu_cal_ax = axerr;
    cfg.imu_cal_ay = ayerr;
    cfg.imu_cal_az = azerr;
  }
  
  Serial.println("Use 'save' to save these values to flash");
}

void Cli::calibrate_Magnetometer() {
  float bias[3], scale[3];

  if (mag.installed()) {
    Serial.print("EXT ");
  }else if (imu.config.has_mag) {
    Serial.print("IMU ");
  }
  Serial.println("Magnetometer calibration. Rotate the IMU about all axes until complete.");
  if ( _calibrate_Magnetometer(bias, scale) ) {
    Serial.println("Calibration Successful!");
    Serial.printf("set mag_cal_x  %+f #config %+f\n", bias[0], cfg.mag_cal_x);
    Serial.printf("set mag_cal_y  %+f #config %+f\n", bias[1], cfg.mag_cal_y);
    Serial.printf("set mag_cal_z  %+f #config %+f\n", bias[2], cfg.mag_cal_z);
    Serial.printf("set mag_cal_sx %+f #config %+f\n", scale[0], cfg.mag_cal_sx);
    Serial.printf("set mag_cal_sy %+f #config %+f\n", scale[1], cfg.mag_cal_sy);
    Serial.printf("set mag_cal_sz %+f #config %+f\n", scale[2], cfg.mag_cal_sz);
    Serial.println("Note: use 'save' to save these values to flash");
    Serial.println(" ");
    Serial.println("If you are having trouble with your attitude estimate at a new flying location, repeat this process as needed.");
    cfg.mag_cal_x = bias[0];
    cfg.mag_cal_y = bias[1];
    cfg.mag_cal_z = bias[2];
    cfg.mag_cal_sx = scale[0];
    cfg.mag_cal_sy = scale[1];
    cfg.mag_cal_sz = scale[2];
  }
  else {
    Serial.println("ERROR: No magnetometer");
  }
}

//get a reading from the magnetometer
bool Cli::_calibrate_Magnetometer_ReadMag(float *m) {
  mag.update();
  m[0] = mag.mx;
  m[1] = mag.my;
  m[2] = mag.mz;
  return true;
}

// finds bias and scale factor calibration for the magnetometer, the sensor should be rotated in a figure 8 motion until complete
// Note: Earth's field ranges between approximately 25 and 65 uT. (Europe & USA: 45-55 uT, inclination 50-70 degrees)
bool Cli::_calibrate_Magnetometer(float bias[3], float scale[3]) 
{
  const int sample_interval = 10000; //in us
  const int maxCounts = 1000; //sample for at least 10 seconds @ 100Hz
  const float deltaThresh = 0.3f; //uT
  const float B_coeff = 0.125;

  float mlast[3] = {0};
  float m[3] = {0};
  int counter;
  float m_filt[3];
  float m_max[3];
  float m_min[3];

  //exit if no mag present
  if(!mag.installed()) return false;

  // get starting set of data
  for(int i=0;i<50;i++) {
    _calibrate_Magnetometer_ReadMag(mlast);
    delayMicroseconds(sample_interval);
    _calibrate_Magnetometer_ReadMag(m);
    delayMicroseconds(sample_interval);
    if ( abs(m[0] - mlast[0]) < 20 && abs(m[1] - mlast[1]) && abs(m[2] - mlast[2]) && m[0] != 0  && m[1] != 0 && m[2] != 0) break;
  }
  for(int i=0;i<3;i++) mlast[i] = m[i];
  
  //save starting data
  for(int i=0;i<3;i++) {
    m_max[i] = m[i];
    m_min[i] = m[i];
    m_filt[i] = m[i];
  }

  // collect data to find max / min in each channel
  // sample counter times, restart sampling when a min/max changed at least deltaThresh uT
  uint32_t start_time = millis()-1000;
  counter = 0;
  uint32_t sample_time = micros();
  while (counter < maxCounts) {
    while(micros() - sample_time < sample_interval); //sample at 100Hz
    sample_time = micros();
    _calibrate_Magnetometer_ReadMag(m);
    if ( abs(m[0] - mlast[0]) < 20 && abs(m[1] - mlast[1]) && abs(m[2] - mlast[2]) && m[0] != 0  && m[1] != 0 && m[2] != 0) {
      for(int i=0;i<3;i++) mlast[i] = m[i];
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
    }
    
    //print progress
    if (millis() - start_time > 1000) {
      start_time = millis();
      Serial.printf("cnt:%d\txmin:%+.2f\txmax:%+.2f\tymin:%+.2f\tymax:%+.2f\tzmin:%+.2f\tzmax:%+.2f\n", counter, m_min[0], m_max[0], m_min[1], m_max[1], m_min[2], m_max[2]);
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

  return true;
}


void Cli::calibrate_info(int seconds) {
  bool report_spikes = (seconds >= 0);
  if(seconds < 0) seconds = -seconds;
  if(seconds == 0) seconds = 3;
  Serial.printf("Gathering sensor statistics, please wait %d seconds ...\n\n", seconds);

  Stat ax(1000),ay(1000),az(1000);
  Stat gx(1000),gy(1000),gz(1000);
  Stat sp(1000),sa(1000),st(1000);
  Stat mx(1000),my(1000),mz(1000);
  uint32_t last_cnt = imu.update_cnt;
  uint32_t ts = micros();

  float bp_last = 0;
  float ba_last = 0;
  float bt_last = 0;

  float mx_last = 0;
  float my_last = 0;
  float mz_last = 0;

  while((uint32_t)micros() - ts < (uint32_t)1000000*seconds) {
    if(last_cnt != imu.update_cnt) {
      ax.append(imu.ax);
      ay.append(imu.ay);
      az.append(imu.az);
      gx.append(imu.gx);
      gy.append(imu.gy);
      gz.append(imu.gz);
      last_cnt = imu.update_cnt;  
    }
    if(bar.installed() && bar.update() && ((bar.press != bp_last) || (bar.alt != ba_last) || (bar.temp != bt_last))) {
      //only record if at least one value is changed
      sp.append(bar.press);
      sa.append(bar.alt);
      st.append(bar.temp);
      bp_last = bar.press;
      ba_last = bar.alt;
      bt_last = bar.temp;
    }
    if(mag.installed() && mag.update() && ((mag.mx != mx_last) || (mag.my != my_last) || (mag.mz != mz_last))) {
      //only record if at least one value is changed
      mx.append(mag.mx);
      my.append(mag.my);
      mz.append(mag.mz);
      mx_last = mag.mx;
      my_last = mag.my;
      mz_last = mag.mz;
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
  hal_print_resources();
  freertos_ps();
  RuntimeTraceGroup::print();
}
