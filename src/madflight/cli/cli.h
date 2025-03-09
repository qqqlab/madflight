/*==========================================================================================
cli.h - madflight Command Line Interface 

MIT License

Copyright (c) 2024-2025 https://madflight.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
===========================================================================================*/

#pragma once

//include all module interfaces
#include "../ahrs/ahrs_interface.h"
#include "../alt/alt_interface.h"
#include "../baro/baro_interface.h"
#include "../bat/bat_interface.h"
#include "../bb/bb_interface.h"
#include "../cfg/cfg_interface.h"
//#include "../cli/cli_interface.h"
#include "../gps/gps_interface.h"
#include "../imu/imu_interface.h"
#include "../led/led_interface.h"
#include "../mag/mag_interface.h"
#include "../out/out_interface.h"
#include "../pid/pid_interface.h"
#include "../rcin/rcin_interface.h"
#include "../veh/veh_interface.h"

#include "stat.h"
#include "FreeRTOS_ps.h"


//cli command extension, return true if command was processed
extern bool cli_execute(String cmd, String arg1, String arg2) __attribute__((weak));

/* example cli command extension
bool cli_execute(String cmd, String arg1, String arg2) {
  if(cmd == "mycommand") {
    Serial.println("ASDFASDFASDF");
    return true;
  }
  return false;
}
*/

void cli_print_overview() {
  Serial.printf("rcin.pwm%d:%d\t", 1, rcin.pwm[0]);
  Serial.printf("rcin.roll:%+.2f\t", rcin.roll);
  Serial.printf("ahrs.gx:%+.2f\t", ahrs.gx);
  Serial.printf("ahrs.ax:%+.2f\t", ahrs.ax);
  Serial.printf("ahrs.mx:%+.2f\t", ahrs.mx);
  Serial.printf("ahrs.roll:%+.1f\t", ahrs.roll);
  Serial.printf("PID.roll:%+.3f\t", PIDroll.PID);
  Serial.printf("out.%c%d%%:%1.0f\t", out.getType(0), 0, 100*out.get(0));
  Serial.printf("gps.sats:%d\t", (int)gps.sat);
  Serial.printf("imu.miss_cnt:%d\t", (int)(imu.interrupt_cnt-imu.update_cnt));
  Serial.printf("imu.upd_cnt:%d\t", (int)imu.update_cnt);
}

void cli_print_rcin_RadioPWM() {
  Serial.printf("rcin_con:%d\t",rcin.connected());
  for(int i=0;i<RCIN_NUM_CHANNELS;i++) Serial.printf("pwm%d:%d\t",i+1,rcin.pwm[i]);
}

void cli_print_rcin_RadioScaled() {
  Serial.printf("rcin.throttle:%.2f\t", rcin.throttle);
  Serial.printf("roll:%+.2f\t", rcin.roll);
  Serial.printf("pitch:%+.2f\t", rcin.pitch);
  Serial.printf("yaw:%+.2f\t", rcin.yaw);
  Serial.printf("arm:%d\t", rcin.arm);
  Serial.printf("flightmode:%d\t", rcin.flightmode);
}

void cli_print_imu_GyroData() {
  Serial.printf("gx:%+.2f\tgy:%+.2f\tgz:%+.2f\t", ahrs.gx, ahrs.gy, ahrs.gz);
}

void cli_print_imu_AccData() {
  Serial.printf("ax:%+.2f\tay:%+.2f\taz:%+.2f\t", ahrs.ax, ahrs.ay, ahrs.az);
}

void cli_print_imu_MagData() {
  Serial.printf("mx:%+.2f\tmy:%+.2f\tmz:%+.2f\t", ahrs.mx, ahrs.my, ahrs.mz); 
}

void cli_print_ahrs_RollPitchYaw() {
  Serial.printf("roll:%+.1f\tpitch:%+.1f\tyaw:%+.1f\t", ahrs.roll, ahrs.pitch, ahrs.yaw);
}

void cli_print_control_PIDoutput() {
  Serial.printf("PID.roll:%+.3f\t",PIDroll.PID);
  Serial.printf("pitch:%+.3f\t",PIDpitch.PID);
  Serial.printf("yaw:%+.3f\t",PIDyaw.PID);
}

void cli_print_out_Command() {
  Serial.printf("out.armed:%d\t", out.armed);
  for(int i=0;i<HW_OUT_COUNT;i++) {
    if(out.getType(i) != 'X') {
      Serial.printf("%c%d%%:%1.0f\t", out.getType(i), i, 100*out.get(i));
    }
  }
}

void cli_print_imu_Rate() {
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
  //Serial.printf("imu%%:%d\t", (int)(100 * (imu.stat_runtime_max - imu.stat_latency) / imu.getSamplePeriod()));
  Serial.printf("samp_hz:%d\t", (int)(1000000/imu.getSamplePeriod()));
  Serial.printf("intr_hz:%.0f\t", (float)delta_int/(dt*1e-6));
  Serial.printf("loop_hz:%.0f\t", (float)delta_upd/(dt*1e-6));
  int miss = (100 - (100 * delta_upd) / delta_int);
  Serial.printf("miss%%:%d\t", (miss<0?0:miss));
  //Serial.printf("stat_cnt:%d\t", (int)(imu.stat_cnt));
  Serial.printf("latency_us:%d\t", (int)(imu.stat_latency/imu.stat_cnt));
  Serial.printf("rt_io_us:%d\t", (int)(imu.stat_io_runtime/imu.stat_cnt));
  Serial.printf("rt_imu_loop_us:%d\t", (int)((imu.stat_runtime - imu.stat_io_runtime)/imu.stat_cnt));
  Serial.printf("rt_us:%d\t", (int)(imu.stat_runtime/imu.stat_cnt));
  Serial.printf("rt_max_us:%d\t", (int)imu.stat_runtime_max);
  Serial.printf("int_cnt:%d\t", (int)imu.interrupt_cnt);
  Serial.printf("upd_cnt:%d\t", (int)imu.update_cnt);
  Serial.printf("miss_cnt:%d\t", (int)(imu.interrupt_cnt-imu.update_cnt));
  imu.statReset();
}

void cli_print_bat() {
  Serial.printf("bat.v:%.2f\t",bat.v);
  Serial.printf("bat.i:%+.2f\t",bat.i);
  Serial.printf("bat.mah:%+.2f\t",bat.mah);
  Serial.printf("bat.wh:%+.2f\t",bat.wh); 
}

static void cli_print_baro() {
  Serial.printf("baro.alt:%.2f\t", baro.alt);
  Serial.printf("press:%.1f\t", baro.press);
  Serial.printf("temp:%.2f\t", baro.temp);
}

void cli_print_gps() {
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
  Serial.printf("baro.alt:%.2f\t", baro.alt);
  Serial.printf("ahrs.aup:%.2f\t", ahrs.getAccelUp());
}

struct cli_print_s {
  const char *cmd;
  const char *info;
  void (*function)(void);
};

#define CLI_PRINT_FLAG_COUNT 14
bool cli_print_flag[CLI_PRINT_FLAG_COUNT] = {false};

static const struct cli_print_s cli_print_options[] = {
  {"po",     "Overview", cli_print_overview},
  {"ppwm",   "Radio pwm (expected: 1000 to 2000)", cli_print_rcin_RadioPWM},
  {"pradio", "Scaled radio (expected: -1 to 1)", cli_print_rcin_RadioScaled},
  {"pimu",   "IMU loop timing (expected: imu%% < 50)", cli_print_imu_Rate},
  {"pgyro",  "Filtered gyro (expected: -250 to 250, 0 at rest)", cli_print_imu_GyroData},
  {"pacc",   "Filtered accelerometer (expected: -2 to 2; x,y 0 when level, z 1 when level)", cli_print_imu_AccData},
  {"pmag",   "Filtered magnetometer (expected: -300 to 300)", cli_print_imu_MagData},
  {"pahrs",  "AHRS roll, pitch, and yaw (expected: degrees, 0 when level)", cli_print_ahrs_RollPitchYaw},
  {"ppid",   "PID output (expected: -1 to 1)", cli_print_control_PIDoutput},
  {"pout",   "Motor/servo output (expected: 0 to 1)", cli_print_out_Command},
  {"pbat",   "Battery voltage, current, Ah used and Wh used", cli_print_bat},
  {"pbaro",  "Barometer", cli_print_baro},
  {"palt",   "Altitude estimator", cli_print_alt},
  {"pgps",   "GPS", cli_print_gps},
};


class CLI {
public:

  void setup() {
    cli_print_all(false);
  }

  //returns true if a command was processed (even an invalid one)
  bool loop() {
    //process chars from Serial
    bool rv = false;
    while(Serial.available()) {
      if(cmd_process_char(Serial.read())) rv = true;
    }

    //handle output for pxxx commands
    cli_print_loop();

    return rv;
  }

  void welcome() {
    Serial.println("CLI:  Command Line Interface Started - Type help for help");
  }

  void help() {
    Serial.printf(
    "-- INFO & TOOLS --\n"
    "help or ? This info\n"
    "board     Board info and pinout\n"
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
    "set [name] [value]\n"
    "clist     List config\n"
    "cclear    Clear config\n"
    "cwrite    Write config to flash\n"
    "cread     Read config to flash\n"
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

private:

  String cmdline = "";
  char prev_c = 0;

public:
  void cmd_execute_batch(const char *batch) {
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

  void cmd_clear() {
    cmdline = "";
    prev_c = 0;;
  }

private:
  //returns true if a command was processed (even an invalid one)
  bool cmd_process_char(char c) {
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

  String getCmdPart(uint32_t &pos) {
    String part = "";
    while(pos < cmdline.length() && cmdline[pos] == ' ') pos++;
    while(pos < cmdline.length() && cmdline[pos] != ' ') {
      part += cmdline[pos];
      pos++;
    }
    return part;
  }

  void processCmd() {
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

public:
  void executeCmd(String cmd, String arg1 = "", String arg2 = "") {
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
    }else if (cmd == "board") {
      print_boardInfo();
    }else if (cmd == "i2c") {
      print_i2cScan();
    }else if (cmd == "reboot") {
      hal_reboot();
    }else if (cmd == "poff") {
      cli_print_all(false);
    }else if (cmd == "pall") {
      cli_print_all(true);
    }else if (cmd == "bbstart") {
      bb.start();
    }else if (cmd == "bbstop") {
      bb.stop();
    }else if (cmd == "bbls") {
      bb.dir();
    }else if (cmd == "bberase") {
      bb.erase();
    }else if (cmd == "bbinfo") {
      bb.info();
    }else if (cmd == "bbbench") {
      bb.bench();
    }else if (cmd == "set") {
      cfg.set(arg1, arg2);
    }else if (cmd == "clist") {
      cfg.list();
    }else if (cmd == "cclear") {
      cfg.clear();
      Serial.println("Config cleared, use 'cwrite' to write to flash");
    }else if (cmd == "cwrite") {
      Serial.println("writing, please wait... ");
      cfg.write();
      Serial.println("cwrite completed");
    }else if (cmd == "cread") {
      cli_print_all(false);
      cfg.read();
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
      rcin.calibrate();
    }else if (cmd == "ps") {
      freertos_ps();
    }else if (cmd != "") {
      Serial.println("ERROR Unknown command - Type help for help");
    }

  }


//========================================================================================================================//
//                                          HELPERS                                                                       //
//========================================================================================================================//

public:

  void print_boardInfo() {
    Serial.print("HW:");
    #ifdef HW_MCU
      Serial.print(" MCU=" HW_MCU);
    #endif
    Serial.print(" BOARD_NAME=" HW_BOARD_NAME);
    Serial.println();
    
    //LED
    #ifdef HW_PIN_LED
      Serial.printf("HW_PIN_LED:            %d\n",HW_PIN_LED);
    #endif
    //#ifdef HW_LED_ON
    //  Serial.printf("HW_LED_ON:             %d\n",HW_LED_ON);
    //#endif

    //IMU SPI:
    #ifdef HW_PIN_SPI_MISO
      Serial.printf("HW_PIN_SPI_MISO:       %d\n",HW_PIN_SPI_MISO);
    #endif
    #ifdef HW_PIN_SPI_MOSI
      Serial.printf("HW_PIN_SPI_MOSI:       %d\n",HW_PIN_SPI_MOSI);
    #endif
    #ifdef HW_PIN_SPI_SCLK
      Serial.printf("HW_PIN_SPI_SCLK:       %d\n",HW_PIN_SPI_SCLK);
    #endif
    #ifdef HW_PIN_IMU_CS
      Serial.printf("HW_PIN_IMU_CS:         %d\n",HW_PIN_IMU_CS);
    #endif
    #ifdef HW_PIN_IMU_EXTI
      Serial.printf("HW_PIN_IMU_EXTI:       %d\n",HW_PIN_IMU_EXTI);
    #endif

    //I2C for BARO, MAG, BAT sensors (and for IMU if not using SPI IMU)
    #ifdef HW_PIN_I2C_SDA
      Serial.printf("HW_PIN_I2C_SDA:        %d\n",HW_PIN_I2C_SDA);
    #endif
    #ifdef HW_PIN_I2C_SCL
      Serial.printf("HW_PIN_I2C_SCL:        %d\n",HW_PIN_I2C_SCL);
    #endif

    //Motor/Servo Outputs:
    #ifdef HW_OUT_COUNT
      Serial.printf("HW_PIN_OUT[%d]:         %d", HW_OUT_COUNT, HW_PIN_OUT[0]);
      for(int i=1; i<HW_OUT_COUNT; i++) Serial.printf(",%d", HW_PIN_OUT[i]);
      Serial.println();
    #endif

    //Serial debug on USB Serial port (no GPIO pins)

    //RC Receiver:
    #ifdef HW_PIN_RCIN_RX
      Serial.printf("HW_PIN_RCIN_RX:        %d\n",HW_PIN_RCIN_RX);
    #endif
    #ifdef HW_PIN_RCIN_TX
      Serial.printf("HW_PIN_RCIN_TX:        %d\n",HW_PIN_RCIN_TX);
    #endif
    #ifdef HW_PIN_RCIN_INVERTER
      Serial.printf("HW_PIN_RCIN_INVERTER:  %d\n",HW_PIN_RCIN_INVERTER);
    #endif

    //GPS:
    #ifdef HW_PIN_GPS_RX
      Serial.printf("HW_PIN_GPS_RX:         %d\n",HW_PIN_GPS_RX);
    #endif
    #ifdef HW_PIN_GPS_TX
      Serial.printf("HW_PIN_GPS_TX:         %d\n",HW_PIN_GPS_TX);
    #endif
    #ifdef HW_PIN_GPS_INVERTER
      Serial.printf("HW_PIN_GPS_INVERTER:   %d\n",HW_PIN_GPS_INVERTER);
    #endif

    //Battery ADC
    #ifdef HW_PIN_BAT_V
      Serial.printf("HW_PIN_BAT_V:          %d\n",HW_PIN_BAT_V);
    #endif
    #ifdef HW_PIN_BAT_I
      Serial.printf("HW_PIN_BAT_I:          %d\n",HW_PIN_BAT_I);
    #endif

    //Black Box SPI (for sdcard or external flash chip):
    #ifdef HW_PIN_SPI2_MISO
      Serial.printf("HW_PIN_SPI2_MISO:      %d\n",HW_PIN_SPI2_MISO);
    #endif
    #ifdef HW_PIN_SPI2_MOSI
      Serial.printf("HW_PIN_SPI2_MOSI:      %d\n",HW_PIN_SPI2_MOSI);
    #endif
    #ifdef HW_PIN_SPI2_SCLK
      Serial.printf("HW_PIN_SPI2_SCLK:      %d\n",HW_PIN_SPI2_SCLK);
    #endif
    #ifdef HW_PIN_BB_CS
      Serial.printf("HW_PIN_BB_CS:          %d\n",HW_PIN_BB_CS);
    #endif

    //Black Box SDCARD via MMC interface:
    #ifdef HW_PIN_SDMMC_DATA
      Serial.printf("HW_PIN_SDMMC_DATA:     %d\n",HW_PIN_SDMMC_DATA);
    #endif
    #ifdef HW_PIN_SDMMC_CLK
      Serial.printf("HW_PIN_SDMMC_CLK:      %d\n",HW_PIN_SDMMC_CLK);
    #endif
    #ifdef HW_PIN_SDMMC_CMD
      Serial.printf("HW_PIN_SDMMC_CMD:      %d\n",HW_PIN_SDMMC_CMD);
    #endif

    Serial.flush();
  }

  void print_i2cScan() {
    Serial.printf("I2C:  Scanning ...\n");
    byte count = 0;
    //mf_i2c->begin();
    for (byte i = 8; i < 120; i++) {
      mf_i2c->beginTransmission(i);          // Begin I2C transmission Address (i)
      if (mf_i2c->endTransmission() == 0) { // Receive 0 = success (ACK response) 
        Serial.printf("I2C:  Found address: 0x%02X (%d)\n",i,i);
        count++;
      }
    }
    Serial.printf("I2C:  Found %d device(s)\n", count);
  }

//========================================================================================================================//
//                                          CALIBRATION FUNCTIONS                                                         //
//========================================================================================================================//

public:

  void calibrate_gyro() {
    Serial.println("Calibrating gyro, don't move vehicle, this takes a couple of seconds...");
    calibrate_IMU2(true);
  }

  void calibrate_IMU() {
    Serial.println("Calibrating IMU, don't move vehicle, this takes a couple of seconds...");
    calibrate_IMU2(false);
  }

  //Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
  void calibrate_IMU2(bool gyro_only = false) {
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
    
    Serial.println("Use CLI 'cwrite' to write these values to flash");
  }

  void calibrate_Magnetometer() {
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
      Serial.println("Note: use CLI 'cwrite' to write these values to flash");
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

private:

  //get a reading from the external or imu magnetometer
  bool _calibrate_Magnetometer_ReadMag(float *m) {
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
  bool _calibrate_Magnetometer(float bias[3], float scale[3]) 
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


  void calibrate_info(int seconds = 0) {
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
      if(baro.installed() && baro.update()) {
        sp.append(baro.press);
        sa.append(baro.alt);
        st.append(baro.temp);
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
    if(baro.installed()) {
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

private:

  uint32_t cli_print_time = 0;



  void cli_print_all(bool val) {
    for(int i=0;i<CLI_PRINT_FLAG_COUNT;i++) cli_print_flag[i] = val;
  }

  void cli_print_loop() {
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

};


CLI cli;