#include "cli.h"

//include all module interfaces
#include "../ahr/ahr.h"
#include "../alt/alt.h"
#include "../bar/bar.h"
#include "../bat/bat.h"
#include "../cfg/cfg.h"
#include "../gps/gps.h"
#include "../hal/hal.h"
#include "../imu/imu.h"
#include "../led/led.h"
#include "../bbx/bbx.h"
#include "../mag/mag.h"
#include "../out/out.h"
#include "../pid/pid.h"
#include "../rcl/rcl.h"
#include "../rdr/rdr.h"
#include "../veh/veh.h"

#include "cli_RclCalibrate.h"

#include "stat.h"
#include "FreeRTOS_ps.h"

//create global module instance
Cli cli;

static void cli_print_overview() {
  Serial.printf("rcl.pwm%d:%d\t", 1, rcl.pwm[0]);
  Serial.printf("rcl.roll:%+.2f\t", rcl.roll);
  Serial.printf("ahr.gx:%+.2f\t", ahr.gx);
  Serial.printf("ahr.ax:%+.2f\t", ahr.ax);
  Serial.printf("ahr.mx:%+.2f\t", ahr.mx);
  Serial.printf("ahr.roll:%+.1f\t", ahr.roll);
  Serial.printf("PID.roll:%+.3f\t", PIDroll.PID);
  Serial.printf("out.%c%d%%:%1.0f\t", out.getType(0), 0, 100*out.get(0));
  Serial.printf("gps.sats:%d\t", (int)gps.sat);
  Serial.printf("imu.miss_cnt:%d\t", (int)(imu.interrupt_cnt-imu.update_cnt));
  Serial.printf("imu.upd_cnt:%d\t", (int)imu.update_cnt);
}

static void cli_print_rcl_RadioPWM() {
  Serial.printf("rcl_con:%d\t",rcl.connected());
  for(int i=0;i<cfg.rcl_num_ch;i++) Serial.printf("pwm%d:%d\t",i+1,rcl.pwm[i]);
}

static void cli_print_rcl_RadioScaled() {
  Serial.printf("rcl.throttle:%.2f\t", rcl.throttle);
  Serial.printf("roll:%+.2f\t", rcl.roll);
  Serial.printf("pitch:%+.2f\t", rcl.pitch);
  Serial.printf("yaw:%+.2f\t", rcl.yaw);
  Serial.printf("armed:%d\t", rcl.armed);
  Serial.printf("flightmode:%d\t", rcl.flightmode);
}

static void cli_print_imu_GyroData() {
  Serial.printf("gx:%+.2f\tgy:%+.2f\tgz:%+.2f\t", ahr.gx, ahr.gy, ahr.gz);
}

static void cli_print_imu_AccData() {
  Serial.printf("ax:%+.2f\tay:%+.2f\taz:%+.2f\t", ahr.ax, ahr.ay, ahr.az);
}

static void cli_print_imu_MagData() {
  Serial.printf("mx:%+.2f\tmy:%+.2f\tmz:%+.2f\t", ahr.mx, ahr.my, ahr.mz); 
}

static void cli_print_imu_Quaternion() {
  if (imu.hasSensorFusion()) {
    Serial.printf("w:%+.2f\tq1:%+.2f\tq2:%+.2f\tq3:%+.2f\t", imu.q[0], imu.q[1], imu.q[2], imu.q[3]);
  } else {
    Serial.printf("IMU does not have sensor fusion.");
  }
}

static void cli_print_ahr_RollPitchYaw_verbose() {
    // from ahr.h:
  // roll in degrees: -180 to 180, roll right is +
  // pitch in degrees: -90 to 90, pitch up is +
  // yaw in degrees: -180 to 180, yaw right is positive
  const char* roll_str = (ahr.roll >= 0.0) ? "right" : "left"; // roll right is clockwise, is it?
  const char* pitch_str = (ahr.pitch >= 0.0) ? "up" : "down";
  const char* yaw_str = (ahr.yaw >= 0.0) ? "right" : "left";
  Serial.printf("roll:%+.1f (roll %s)\tpitch:%+.1f (pitch %s)\tyaw:%+.1f (yaw %s)\t", ahr.roll, roll_str, ahr.pitch, pitch_str, ahr.yaw, yaw_str);
}

static void cli_print_ahr_RollPitchYaw() {
  Serial.printf("roll:%+.1f\tpitch:%+.1f\tyaw:%+.1f\t", ahr.roll, ahr.pitch, ahr.yaw);
}

static void cli_print_ahr_Quaternion() {
  Serial.printf("w:%+.2f\tq1:%+.2f\tq2:%+.2f\tq3:%+.2f", ahr.q[0], ahr.q[1], ahr.q[2], ahr.q[3]);
}

static void cli_print_control_PIDoutput() {
  Serial.printf("PID.roll:%+.3f\t",PIDroll.PID);
  Serial.printf("pitch:%+.3f\t",PIDpitch.PID);
  Serial.printf("yaw:%+.3f\t",PIDyaw.PID);
}

static void cli_print_out_Command() {
  Serial.printf("out.armed:%d\t", out.armed);
  for(int i=0;i<OUT_SIZE;i++) {
    if(out.getType(i)) {
      Serial.printf("%c%d%%:%1.0f\t", out.getType(i), i, 100*out.get(i));
    }
  }
}

static void cli_print_imu_Rate() {
  static uint32_t interrupt_cnt_last = 0;
  static uint32_t update_cnt_last = 0;
  static uint32_t ts_last = 0;
  uint32_t delta_int = imu.interrupt_cnt - interrupt_cnt_last;
  interrupt_cnt_last = imu.interrupt_cnt;
  uint32_t delta_upd = imu.update_cnt - update_cnt_last;
  update_cnt_last = imu.update_cnt;
  uint32_t now = micros();
  uint32_t dt = now - ts_last;
  ts_last = now;

  int hz = 0;
  if(imu.getSamplePeriod() > 0) hz = 1000000 / imu.getSamplePeriod();
  Serial.printf("samp_hz:%d\t", hz);

  if(dt == 0) dt = 1;
  Serial.printf("intr_hz:%.0f\t", (float)delta_int/(dt*1e-6));
  Serial.printf("loop_hz:%.0f\t", (float)delta_upd/(dt*1e-6));

  int miss = 0;
  if(delta_int > 0) miss = (100 - (100 * delta_upd) / delta_int);
  Serial.printf("miss%%:%d\t", (miss<0?0:miss));

  int stat_cnt = 1;
  if(imu.stat_cnt > 0) stat_cnt = imu.stat_cnt;
  //Serial.printf("stat_cnt:%d\t", (int)(stat_cnt));
  Serial.printf("latency_us:%d\t", (int)(imu.stat_latency / stat_cnt));
  Serial.printf("rt_io_us:%d\t", (int)(imu.stat_io_runtime / stat_cnt));
  Serial.printf("rt_imu_loop_us:%d\t", (int)((imu.stat_runtime - imu.stat_io_runtime) / stat_cnt));
  Serial.printf("rt_us:%d\t", (int)(imu.stat_runtime / stat_cnt));
  Serial.printf("rt_max_us:%d\t", (int)imu.stat_runtime_max);
  Serial.printf("int_cnt:%d\t", (int)imu.interrupt_cnt);
  Serial.printf("upd_cnt:%d\t", (int)imu.update_cnt);
  Serial.printf("miss_cnt:%d\t", (int)(imu.interrupt_cnt - imu.update_cnt));
  imu.statReset();
}

static void cli_print_bat() {
  Serial.printf("bat.v:%.2f\t",bat.v);
  Serial.printf("bat.i:%+.2f\t",bat.i);
  Serial.printf("bat.mah:%+.2f\t",bat.mah);
  Serial.printf("bat.wh:%+.2f\t",bat.wh); 
}

static void cli_print_bar() {
  Serial.printf("bar.alt:%.2f\t", bar.alt);
  Serial.printf("press:%.1f\t", bar.press);
  Serial.printf("temp:%.2f\t", bar.temp);
}

static void cli_print_gps() {
  Serial.printf("gps.time:%d\t", (int)gps.time);
  Serial.printf("fix:%d\t", (int)gps.fix);
  //Serial.printf("date:%d\t", (int)gps.date);
  Serial.printf("sat:%d\t", (int)gps.sat);
  Serial.printf("lat:%d\t", (int)gps.lat);
  Serial.printf("lon:%d\t", (int)gps.lon);
  Serial.printf("alt:%.3f\t", (float)gps.alt/1000.0);
}

static void cli_print_alt() {
  char s[100];
  alt.toString(s);
  Serial.print(s);
  Serial.printf("bar.alt:%.2f\t", bar.alt);
  Serial.printf("ahr.aup:%.2f\t", ahr.getAccelUp());
}

static void cli_print_rdr() {
  Serial.printf("rdr.dist:%d\t", rdr.dist);
}

struct cli_print_s {
  const char *cmd;
  const char *info;
  void (*function)(void);
};

#define CLI_PRINT_FLAG_COUNT 18
bool cli_print_flag[CLI_PRINT_FLAG_COUNT] = {false};

static const struct cli_print_s cli_print_options[CLI_PRINT_FLAG_COUNT] = {
  {"po",     "Overview", cli_print_overview},
  {"ppwm",   "Radio pwm (expected: 1000 to 2000)", cli_print_rcl_RadioPWM},
  {"prcl",   "Scaled radio (expected: -1 to 1)", cli_print_rcl_RadioScaled},
  {"pimu",   "IMU loop timing (expected: miss% <= 1)", cli_print_imu_Rate},
  {"pgyr",   "Filtered gyro (expected: -250 to 250, 0 at rest)", cli_print_imu_GyroData},
  {"pacc",   "Filtered accelerometer (expected: -2 to 2; when level: x=0,y=0,z=1)", cli_print_imu_AccData},
  {"pmag",   "Filtered magnetometer (expected: -300 to 300)", cli_print_imu_MagData},
  {"pquat",  "IMU quaternion data", cli_print_imu_Quaternion},
  {"pahr",   "AHRS roll, pitch, and yaw in human friendly format (expected: degrees, 0 when level)", cli_print_ahr_RollPitchYaw_verbose},
  {"pah",    "AHRS roll, pitch, and yaw in less verbose format (expected: degrees, 0 when level)", cli_print_ahr_RollPitchYaw},
  {"pahq",   "AHRS quaternion data", cli_print_ahr_Quaternion},
  {"ppid",   "PID output (expected: -1 to 1)", cli_print_control_PIDoutput},
  {"pout",   "Motor/servo output (expected: 0 to 1)", cli_print_out_Command},
  {"pbat",   "Battery voltage, current, Ah used and Wh used", cli_print_bat},
  {"pbar",   "Barometer", cli_print_bar},
  {"palt",   "Altitude estimator", cli_print_alt},
  {"pgps",   "GPS", cli_print_gps},
  {"prdr",   "Radar", cli_print_rdr},
};




void Cli::setup() {
  cli_print_all(false);
}

//returns true if a command was processed (even an invalid one)
bool Cli::update() {
  if(mavlink) {
    return mavlink->update();
  }else{
    //process chars from Serial
    bool rv = false;
    while(Serial.available()) {
      uint8_t c = Serial.read();
      if(c == 0xFD || c == 0xFE) { //mavlink v1,v2 protocol header byte
        auto ser = &Serial;
        MF_Serial *ser_bus = new MF_SerialPtrWrapper<decltype(ser)>( ser );
        mavlink = new RclGizmoMavlink(ser_bus, 115200, nullptr);
        return false;
      }
      if(cmd_process_char(c)) rv = true;
    }

    //handle output for pxxx commands
    cli_print_loop();

    return rv;
  }
}

void Cli::begin() {
  Serial.println("CLI: Command Line Interface Started - Type help for help");
}

void Cli::help() {
  Serial.printf(
  "-- INFO & TOOLS --\n"
  "help or ? This info\n"
  "ps        Task list\n"
  "i2c       I2C scan\n"
  "reboot    Reboot flight controller\n"
  "-- PRINT --\n"
  "poff      Printing off\n"
  "pall      Print all\n"
  );
  for(int i=0;i<CLI_PRINT_FLAG_COUNT;i++) {
    Serial.print(cli_print_options[i].cmd);
    for(int j = strlen(cli_print_options[i].cmd); j < 9; j++) {
      Serial.print(' ');
    }
    Serial.print(' ');
    Serial.print(cli_print_options[i].info);
    Serial.println();
  }
  Serial.printf(
  "-- BLACK BOX --\n"
  "bbstart   Start logging\n"
  "bbstop    Stop logging\n"
  "bbls      List files\n"
  "bberase   Erase bb device\n"
  "bbbench   Benchmark\n"
  "bbinfo    Info\n"
  "-- CONFIG --\n"
  "set <name> <value>  Set config parameter\n"
  "dump <filter>       List config\n"
  "diff <filter>       List config changes from default d\n"
  "save                Save config and reboot\n"
  "defaults            Reset to defaults and reboot\n"
  "-- CALIBRATE --\n"
  "calinfo   Sensor info\n"
  "calimu    Calibrate IMU error\n"
  "calmag    Calibrate magnetometer\n"
  "calradio  Calibrate RC Radio\n"
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
    freertos_ps();
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
    }
  }
}

//========================================================================================================================//
//                                          CALIBRATION FUNCTIONS                                                         //
//========================================================================================================================//

void Cli::calibrate_gyro() {
  Serial.println("Calibrating gyro, don't move vehicle, this takes a few seconds...");
  calibrate_IMU2(true);
}

void Cli::calibrate_IMU() {
  Serial.println("Calibrating IMU, don't move vehicle, this takes a few seconds...");
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
  for(int i=0; i<cnt; i++) {
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

  if (imu.hasSensorFusion()) {
    Serial.println("Calibrating IMU sensor fusion, don't move vehicle, this takes a few seconds...");

    // Wait for a fresh sample to ensure imu.q[] is up-to-date
    imu.waitNewSample(); // Ensure fresh sample from ISR before using imu.q[]

    // Compute quaternion bias from IMU fusion quaternion
    float q_dmp[4] = {0};
    const int q_cnt = 3000;
    for (int i = 0; i < q_cnt; i++) {
      imu.waitNewSample();
      for (int j = 0; j < 4; j++) {
        q_dmp[j] += imu.q[j];
      }
    }

    // Normalize the quaternion
    float norm = sqrt(q_dmp[0]*q_dmp[0] + q_dmp[1]*q_dmp[1] + q_dmp[2]*q_dmp[2] + q_dmp[3]*q_dmp[3]);
    for (int i = 0; i < 4; i++) q_dmp[i] /= norm;

    // Store inverse as q_bias
    cfg.imu_cal_q0_bias =  q_dmp[0];
    cfg.imu_cal_q1_bias = -q_dmp[1];
    cfg.imu_cal_q2_bias = -q_dmp[2];
    cfg.imu_cal_q3_bias = -q_dmp[3];

    Serial.printf("set imu_cal_q0_bias %+f\n", cfg.imu_cal_q0_bias);
    Serial.printf("set imu_cal_q1_bias %+f\n", cfg.imu_cal_q1_bias);
    Serial.printf("set imu_cal_q2_bias %+f\n", cfg.imu_cal_q2_bias);
    Serial.printf("set imu_cal_q3_bias %+f\n", cfg.imu_cal_q3_bias);
  }

  Serial.println("Use 'save' to save these values to flash");
}

void Cli::calibrate_Magnetometer() {
  float bias[3], scale[3];

  if (mag.installed()) {
    Serial.print("EXT ");
  }else if (imu.hasMag()) {
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



//get a reading from the external or imu magnetometer
bool Cli::_calibrate_Magnetometer_ReadMag(float *m) {
  if (mag.installed()) {
    mag.update();
    m[0] = mag.x;
    m[1] = mag.y;
    m[2] = mag.z;
  }else{
    if(!imu.hasMag()) return false;
    imu.waitNewSample();
    m[0] = imu.mx;
    m[1] = imu.my;
    m[2] = imu.mz;
  }
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
  if(!_calibrate_Magnetometer_ReadMag(m)) return false;

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
  if(seconds<=0) seconds = 3;
  Serial.printf("Gathering sensor statistics, please wait %d seconds ...\n", seconds);

  Stat ax,ay,az,gx,gy,gz;
  Stat sp,sa,st;
  Stat mx,my,mz;
  uint32_t last_cnt = imu.update_cnt;
  uint32_t ts = millis();

  while((uint32_t)millis() - ts < (uint32_t)1000*seconds) {
    if(last_cnt != imu.update_cnt) {
      ax.append(imu.ax);
      ay.append(imu.ay);
      az.append(imu.az);
      gx.append(imu.gx);
      gy.append(imu.gy);
      gz.append(imu.gz);
      last_cnt = imu.update_cnt;
    }
    if(bar.installed() && bar.update()) {
      sp.append(bar.press);
      sa.append(bar.alt);
      st.append(bar.temp);
    }
    if(mag.installed() && mag.update()) {
      mx.append(mag.x);
      my.append(mag.y);
      mz.append(mag.z);
    }
  } 

  Serial.println("=== Gyro ===");
  gx.print("gx[deg/s]     ");
  gy.print("gy[deg/s]     ");
  gz.print("gz[deg/s]     ");
  Serial.println("=== Accelerometer ===");
  ax.print("ax[g]         ");
  ay.print("ay[g]         ");
  az.print("az[g]         ");
  if(bar.installed()) {
    Serial.println("=== Barometer ===");
    sa.print("Altitude[m]   ");
    sp.print("Pressure[Pa]  ");
    st.print("Temperature[C]");
  }
  if(mag.installed()) {
    Serial.println("=== Magnetometer (external) ===");
    mx.print("mx[uT]        ");
    my.print("my[uT]        ");
    mz.print("mz[uT]        ");
  }
}

//========================================================================================================================//
//                                                PRINT FUNCTIONS                                                         //
//========================================================================================================================//

void Cli::cli_print_all(bool val) {
  for(int i=0;i<CLI_PRINT_FLAG_COUNT;i++) cli_print_flag[i] = val;
}

void Cli::cli_print_loop() {
  uint32_t cli_print_interval = 100000; //Print data at cli_print_interval microseconds
  if (micros() - cli_print_time > cli_print_interval) {
    cli_print_time = micros();
    bool cli_print_need_newline = false;
    //Serial.printf("loop_time:%d\t",loop_time); //print loop time stamp
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
