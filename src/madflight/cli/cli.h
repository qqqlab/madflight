//CLI Command Line Interface for madflight

#pragma once

void cli_print_overview() {
  Serial.printf("CH%d:%d\t", 1, rcin_pwm[0]);  
  Serial.printf("rcin_roll:%+.2f\t", rcin_roll);
  Serial.printf("gx:%+.2f\t", ahrs.gx);
  Serial.printf("ax:%+.2f\t", ahrs.ax);
  Serial.printf("mx:%+.2f\t", ahrs.mx);
  Serial.printf("ahrs.roll:%+.1f\t", ahrs.roll);
  Serial.printf("roll_PID:%+.3f\t", roll_PID);  
  Serial.printf("m%d%%:%1.0f\t", 1, 100*out_command[0]);
  Serial.printf("sats:%d\t", (int)gps.sat);
  Serial.printf("imu%%:%d\t", (int)(100 * imu.runtime_tot_max / imu.getSamplePeriod()));
  Serial.printf("imu_cnt:%d\t", (int)imu.update_cnt);
}

void cli_print_rcin_RadioPWM() {
  Serial.printf("rcin_con:%d\t",rcin.connected());
  for(int i=0;i<RCIN_NUM_CHANNELS;i++) Serial.printf("pwm%d:%d\t",i+1,rcin_pwm[i]);
}

void cli_print_rcin_RadioScaled() {
  Serial.printf("rcin_thro:%.2f\t", rcin_thro);
  Serial.printf("rcin_roll:%+.2f\t", rcin_roll);
  Serial.printf("rcin_pitch:%+.2f\t", rcin_pitch);
  Serial.printf("rcin_yaw:%+.2f\t", rcin_yaw);
  Serial.printf("rcin_arm:%d\t", rcin_armed);
  Serial.printf("rcin_aux:%d\t", rcin_aux);
  Serial.printf("out_armed:%d\t", out_armed);
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
  Serial.printf("yaw_mag:%+.1f\t",-atan2(ahrs.my, ahrs.mx) * Ahrs::rad_to_deg);
}

void cli_print_control_PIDoutput() {
  Serial.printf("roll_PID:%+.3f\tpitch_PID:%+.3f\tyaw_PID:%+.3f\t",roll_PID,pitch_PID,yaw_PID);
}

void cli_print_out_MotorCommands() {
  Serial.printf("out_armed:%d\t", out_armed);
  for(int i=0;i<out_MOTOR_COUNT;i++) Serial.printf("m%d%%:%1.0f\t", i+1, 100*out_command[i]);
}

void cli_print_out_ServoCommands() {
  for(int i=out_MOTOR_COUNT;i<HW_OUT_COUNT;i++) Serial.printf("s%d%%:%1.0f\t", i-out_MOTOR_COUNT+1, 100*out_command[i]);
}

void cli_print_imu_Rate() {
  static uint32_t update_cnt_last = 0;
  Serial.printf("imu%%:%d\t", (int)(100 * imu.runtime_tot_max / imu.getSamplePeriod()));
  Serial.printf("period:%d\t", (int)imu.getSamplePeriod());
  Serial.printf("dt:%d\t", (int)(imu.dt * 1000000.0));
  Serial.printf("rt:%d\t", (int)imu.runtime_tot_max);
  Serial.printf("rt_int:%d\t", (int)imu.runtime_int);
  Serial.printf("rt_bus:%d\t", (int)imu.runtime_bus);
  Serial.printf("rt_fus:%d\t", (int)(imu.runtime_tot_max - imu.runtime_bus - imu.runtime_int));
  Serial.printf("overruns:%d\t", (int)(imu.overrun_cnt));
  Serial.printf("cnt:%d\t", (int)imu.update_cnt);
  Serial.printf("loops:%d\t", (int)(imu.update_cnt - update_cnt_last));
  update_cnt_last = imu.update_cnt;
}

void cli_print_bat() {
  Serial.printf("bat.v:%.2f\t",bat.v);
  Serial.printf("bat.i:%+.2f\t",bat.i);
  Serial.printf("bat.mah:%+.2f\t",bat.mah);
  Serial.printf("bat.wh:%+.2f\t",bat.wh); 
}

void cli_print_baro() {
  Serial.printf("baro.alt:%.2f\t", baro.alt);
  Serial.printf("vz:%.1f\t", baro.vz);
  Serial.printf("press:%.1f\t", baro.press);
  Serial.printf("temp:%.2f\t", baro.temp);
}

struct cli_print_s {
  String cmd;
  String info;
  void (*function)(void);
};


#define CLI_PRINT_FLAG_COUNT 13
bool cli_print_flag[CLI_PRINT_FLAG_COUNT] = {false};

struct cli_print_s cli_print_options[] = {
  {"po", "Overview: pwm1, rcin_roll, gyroX, accX, magX, ahrs.roll, pid_roll, motor1, imu%", cli_print_overview},
  {"ppwm", "Radio pwm (expected: 1000 to 2000)", cli_print_rcin_RadioPWM},
  {"pradio", "Scaled radio (expected: -1 to 1)", cli_print_rcin_RadioScaled},
  {"pimu", "IMU loop timing (expected: imu%% < 50)", cli_print_imu_Rate},
  {"pgyro", "Filtered gyro (expected: -250 to 250, 0 at rest)", cli_print_imu_GyroData},
  {"pacc", "Filtered accelerometer (expected: -2 to 2; x,y 0 when level, z 1 when level)", cli_print_imu_AccData},
  {"pmag", "Filtered magnetometer (expected: -300 to 300)", cli_print_imu_MagData},
  {"proll", "AHRS roll, pitch, and yaw (expected: degrees, 0 when level)", cli_print_ahrs_RollPitchYaw},
  {"ppid", "PID output (expected: -1 to 1)", cli_print_control_PIDoutput},
  {"pmot", "Motor output (expected: 0 to 1)", cli_print_out_MotorCommands},
  {"pservo", "Servo output (expected: 0 to 1)", cli_print_out_ServoCommands},
  {"pbat", "Battery voltage, current, Ah used and Wh used", cli_print_bat},
  {"pbaro", "Barometer", cli_print_baro},
};


class CLI {
public:

  void setup() {
    cli_print_all(false);
  }

  //returns true if a command was processed (even an invalid one)
  bool loop() {
    static char prev_c = 0;
    bool rv = false;
    while(Serial.available()) {
      char c = Serial.read();
      if ( (c=='\r' && prev_c!='\n') || (c=='\n' && prev_c!='\n') ) { //accept \n, \r, \r\n, \n\r as end of command
        processCmd();
        rv = true;
      }else{
        cmdline += c;
      }
      prev_c = c;
    }
    
    //handle output for pxxx commands
    cli_print_loop();
    
    return rv;
  }

  void welcome() {
    Serial.println("CLI: Command Line Interface Started - Type help for help");
  }

  void help() {
    Serial.printf(
    "-- INFO & TOOLS --\n"
    "help or ? This info\n"
    "board     Board info and pinout\n"
    "i2c       I2C scan\n"
    "reboot    Reboot flight controller\n"
    "-- PRINT --\n"
    "poff      Printing off\n"
    "pall      Print all\n"
    );
    for(int i=0;i<CLI_PRINT_FLAG_COUNT;i++) {
      Serial.print(cli_print_options[i].cmd);
      for(int j=cli_print_options[i].cmd.length();j<9;j++) Serial.print(' ');
      Serial.print(' ');
      Serial.print(cli_print_options[i].info);
      Serial.println();
    }
    Serial.printf(
    "-- BLACK BOX --\n"
    "bbstart   Start logging\n"
    "bbstop    Stop logging\n"
    "bbls      List files\n"
    "bbdump n  Dump file n in CSV format\n"
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
    "calimu    Calibrate IMU error\n"
    "calmag    Calibrate magnetometer\n"
    );
  }

private:

  String cmdline = "";

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
      if (cmd == cli_print_options[i].cmd) {
        cli_print_flag[i] = !cli_print_flag[i];
        return;
      }
    }

    if (cmd=="help" || cmd=="?") {
      help();
    }else if (cmd == "board") {
      print_boardInfo();
    }else if (cmd == "i2c") {
      print_i2cScan();
    }else if (cmd == "reboot") {
      hw_reboot();
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
    }else if (cmd == "bbdump") {
      bb.csvDump(arg1.toInt());
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
      cfg.read();
    }else if (cmd == "calimu") {
      calibrate_IMU();
    }else if (cmd == "calmag") {
      calibrate_Magnetometer();
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
    
    Serial.printf("HW_PIN: LED=%d", HW_PIN_LED);
    Serial.printf(" SPI_MOSI=%d MISO=%d SCLK=%d", HW_PIN_SPI_MOSI, HW_PIN_SPI_MISO, HW_PIN_SPI_SCLK);
    Serial.printf(" IMU_CS=%d",HW_PIN_IMU_CS);  
    Serial.printf(" IMU_EXTI=%d",HW_PIN_IMU_EXTI);
    Serial.printf(" I2C_SDA=%d SCL=%d", HW_PIN_I2C_SDA, HW_PIN_I2C_SCL);
    Serial.printf(" OUT[%d]=%d", HW_OUT_COUNT, HW_PIN_OUT[0]);  
    for(int i=1; i<HW_OUT_COUNT; i++) Serial.printf(",%d", HW_PIN_OUT[i]);  
    Serial.printf(" RCIN_RX=%d TX=%d", HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);
    Serial.printf(" GPS_RX=%d TX=%d", HW_PIN_GPS_RX, HW_PIN_GPS_TX);
    Serial.println();
  }

  void print_i2cScan() {
    Serial.printf("I2C: Scanning ...\n");
    byte count = 0;
    //i2c->begin();
    for (byte i = 8; i < 120; i++) {
      i2c->beginTransmission(i);          // Begin I2C transmission Address (i)
      if (i2c->endTransmission() == 0) { // Receive 0 = success (ACK response) 
        Serial.printf("I2C: Found address: 0x%02X (%d)\n",i,i);
        count++;
      }
    }
    Serial.printf("I2C: Found %d device(s)\n", count);
  }

//========================================================================================================================//
//                                          CALIBRATION FUNCTIONS                                                         //
//========================================================================================================================//

public:

  void calibrate_gyro() {
    Serial.println("Calibrating gyro, this takes a couple of seconds...");
    calibrate_IMU2(true);
  }

  void calibrate_IMU() {
    Serial.println("Calibrating IMU, this takes a couple of seconds...");
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

/*
  void calibrate_ESCs() { //TODO
    //DESCRIPTION: Used in void setup() to allow standard ESC calibration procedure with the radio to take place.
    //  Simulates the void loop(), but only for the purpose of providing throttle pass through to the motors, so that you can
    //  power up with throttle at full, let ESCs begin arming sequence, and lower throttle to zero. This function should only be
    //  uncommented when performing an ESC calibration.

    uint32_t ts = micros();
    while (true) {
      while ( (micros() - ts) < (1000000U / loop_freq) ); //Keeps loop sample rate constant. (Waste time until sample time has passed.)
      ts = micros();

      led_SwitchON(true); //LED on to indicate we are not in main loop

      rcin_GetCommands(); //Pulls current available radio commands
      rcin_Failsafe(); //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup
      rcin_Normalize(); //Convert raw commands to normalized values based on saturated control limits
      
      //set all motors
      for(int i=0;i<out_MOTOR_COUNT;i++) out_command[i] = rcin_thro;
    
      //out_KillSwitch(); //Don't update motor outputs to 0 if disarmed
      out_SetCommands(); //Sends command pulses to each motor pin
      
      //printRadioData(); //Radio pwm values (expected: 1000 to 2000)
    }
  }
*/

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
      imu.runtime_tot_max = 0; //reset maximum runtime
    }
  }

};


CLI cli;