//CLI Command Line Interface for madflight

#pragma once

class CLI {

public:

  void setup() {
    print_all(false);
  }

  //returns true if a command was processed (even an invalid one)
  bool loop() {
    static char prev_c = 0;
    bool rv = false;
    while(Serial.available()) {
      char c = Serial.read();
      if( (c=='\r' && prev_c!='\n') || (c=='\n' && prev_c!='\n') ) { //accept \n, \r, \r\n, \n\r as end of command
        processCmd();
        rv = true;
      }else{
        cmdline += c;
      }
      prev_c = c;
    }
    
    //handle output for pxxx commands
    print_loop();
    
    return rv;
  }

  void welcome() {
    Serial.println("CLI Command Line Interface Started - Type help for help");
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
    "po        Overview: pwm1, rcin_roll, gyroX, accX, magX, ahrs_roll, pid_roll, motor1, loop_rt\n"
    "ppwm      Radio pwm (expected: 1000 to 2000)\n"
    "pradio    Scaled radio (expected: -1 to 1)\n"
    "pgyro     Filtered gyro (expected: -250 to 250, 0 at rest)\n"
    "pacc      Filtered accelerometer (expected: -2 to 2; x,y 0 when level, z 1 when level)\n"
    "pmag      Filtered magnetometer (expected: -300 to 300)\n"
    "proll     AHRS roll, pitch, and yaw (expected: degrees, 0 when level)\n"
    "ppid      PID output (expected: -1 to 1)\n"
    "pmot      Motor output (expected: 0 to 1)\n"
    "pservo    Servo output (expected: 0 to 1)\n"
    "ploop     Loop timing in microseconds (expected: 1000000 / loop_freq)\n"
    "pbat      Battery voltage, current, Ah used and Wh used\n"
    "-- BLACK BOX --\n"
    "bbdump    Dump CSV format\n"
    "bbstart   Start logging\n"
    "bbstop    Stop logging\n"
    "bberase   Erase bb device\n"
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

    Serial.println( "> " + cmd + " " + arg1 + " " + arg2 );

    if(cmd=="help" || cmd=="?") {
      help();
    }else if(cmd == "board") {
      print_boardInfo();
    }else if(cmd == "i2c") {
      print_i2cScan();
    }else if(cmd == "reboot") {
      hw_reboot();
    }else if(cmd == "poff") {
      print_all(false);
    }else if(cmd == "pall") {
      print_all(true); 
    }else if(cmd == "po") {
      print_flag[0] = !print_flag[0];
    }else if(cmd == "ppwm") {
      print_flag[1] = !print_flag[1];
    }else if(cmd == "pradio") {
      print_flag[2] = !print_flag[2];
    }else if(cmd == "pgyro") {
      print_flag[3] = !print_flag[3];
    }else if(cmd == "pacc") {
      print_flag[4] = !print_flag[4];
    }else if(cmd == "pmag") {
      print_flag[5] = !print_flag[5];
    }else if(cmd == "proll") {
      print_flag[6] = !print_flag[6];
    }else if(cmd == "ppid") {
      print_flag[7] = !print_flag[7];
    }else if(cmd == "pmot") {
      print_flag[8] = !print_flag[8];
    }else if(cmd == "pservo") {
      print_flag[9] = !print_flag[9];
    }else if(cmd == "ploop") {
      print_flag[10] = !print_flag[10];
    }else if(cmd == "pbat") {
      print_flag[11] = !print_flag[11];
    }else if(cmd == "bbdump") {
      bb.csvDump();
    }else if(cmd == "bbstart") {
      bb.start();
    }else if(cmd == "bbstop") {
      bb.stop();
    }else if(cmd == "bberase") {
      bb.erase();
    }else if(cmd == "set") {
      cfg.set(arg1, arg2);
    }else if(cmd == "clist") {
      cfg.list();
    }else if(cmd == "cclear") {
      cfg.clear();
      Serial.println("Config cleared, use 'cwrite' to write to flash");
    }else if(cmd == "cwrite") {
      Serial.println("writing, please wait... ");
      cfg.write();
      Serial.println("cwrite completed");
    }else if(cmd == "cread") {
      cfg.read();
    }else if(cmd == "calimu") {
      calibrate_IMU();
    }else if(cmd == "calmag") {
      calibrate_Magnetometer();
    }else if(cmd != "") {
      Serial.println("ERROR Unknown command - Type help for help");
    }
    cmdline = "";
  }


//========================================================================================================================//
//                                          HELPERS                                                                       //
//========================================================================================================================//

public:

  void print_boardInfo() {
    Serial.println("HW_BOARD_NAME=" HW_BOARD_NAME);
    #ifdef HW_MCU
      Serial.println("HW_MCU=" HW_MCU);
    #endif  
    #ifdef USE_IMU_POLLING
      Serial.println("USE_IMU_POLLING");
    #endif

    Serial.printf("HW_PIN_LED=%d\n", HW_PIN_LED);
    Serial.printf("HW_PIN_SPI_MOSI=%d MISO=%d SCLK=%d\n", HW_PIN_SPI_MOSI, HW_PIN_SPI_MISO, HW_PIN_SPI_SCLK);
    Serial.printf("HW_PIN_IMU_CS=%d\n",HW_PIN_IMU_CS);  
    Serial.printf("HW_PIN_IMU_EXTI=%d\n",HW_PIN_IMU_EXTI);
    Serial.printf("HW_PIN_I2C_SDA=%d SCL=%d\n", HW_PIN_I2C_SDA, HW_PIN_I2C_SCL);
    Serial.printf("HW_PIN_OUT[%d]=%d", HW_OUT_COUNT, HW_PIN_OUT[0]);  for(int i=1; i<HW_OUT_COUNT; i++) Serial.printf(",%d", HW_PIN_OUT[i]);  Serial.println();
    Serial.printf("HW_PIN_RCIN_RX=%d TX=%d\n", HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);
    Serial.printf("HW_PIN_GPS_RX=%d TX=%d\n", HW_PIN_GPS_RX, HW_PIN_GPS_TX);
  }

  void print_i2cScan() {
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
    imu_loop_enable = false; //disable running of imu_loop()

    //Read IMU values, and average the readings
    int cnt = 3000;
    float axerr = 0;
    float ayerr = 0;
    float azerr = 0;
    float gxerr = 0;
    float gyerr = 0;
    float gzerr = 0;
    for(int i=0; i<cnt; i++) {
      imu.update();
      axerr += imu.ax;
      ayerr += imu.ay;
      azerr += imu.az;
      gxerr += imu.gx;
      gyerr += imu.gy;
      gzerr += imu.gz;
      delayMicroseconds(1000000/loop_freq);
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
    
    if(gyro_only) {
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

    imu_loop_enable = true; //enable running of imu_loop()
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

    Serial.println("Magnetometer calibration. Rotate the IMU about all axes until complete.");
    if( _calibrate_Magnetometer(bias, scale) ) {
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

    while(1); //Halt code so it won't enter main loop until this function commented out
  }

private:

  //get a reading from the external or imu magnetometer
  bool _calibrate_Magnetometer_ReadMag(float *m) {
    if(mag.installed()) {
      mag.update();
      m[0] = mag.x;
      m[1] = mag.y;
      m[2] = mag.z;
    }else{
      if(!imu.hasMag()) return false;
      imu.update();
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
    const int maxCounts = 1000; //sample for at least 10 seconds @ 100Hz
    const float deltaThresh = 0.3f; //uT
    const float B_coeff = 0.125;

    float m[3] = {0};
    int counter;
    float m_filt[3];
    float m_max[3];
    float m_min[3];

    // get a starting set of data (and test if mag is present)
    if(!_calibrate_Magnetometer_ReadMag(m)) return false;
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
      _calibrate_Magnetometer_ReadMag(m);
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

    return true;
  }


//========================================================================================================================//
//                                                PRINT FUNCTIONS                                                         //
//========================================================================================================================//

public:

  uint32_t print_time = 0;
  bool print_need_newline = false;
#define CLI_PRINT_FLAG_COUNT 12
  bool print_flag[CLI_PRINT_FLAG_COUNT];

  void print_all(bool val) {
    for(int i=0;i<CLI_PRINT_FLAG_COUNT;i++) print_flag[i] = val;
  }

  void print_loop() {
    //Debugging - Print data at print_interval microseconds, uncomment line(s) for troubleshooting
    uint32_t print_interval = 100000;
    if (micros() - print_time > print_interval) {
      print_time = micros();
      print_need_newline = false;
      //Serial.printf("loop_time:%d\t",loop_time); //print loop time stamp
      if(print_flag[0])  print_overview(); //prints: pwm1, rcin_roll, gyroX, accX, magX, ahrs_roll, pid_roll, motor1, loop_rt
      if(print_flag[1])  print_rcin_RadioPWM();     //Prints radio pwm values (expected: 1000 to 2000)
      if(print_flag[2])  print_rcin_RadioScaled();     //Prints scaled radio values (expected: -1 to 1)    
      if(print_flag[3])  print_imu_GyroData();      //Prints filtered gyro data direct from IMU (expected: -250 to 250, 0 at rest)
      if(print_flag[4])  print_imu_AccData();     //Prints filtered accelerometer data direct from IMU (expected: -2 to 2; x,y 0 when level, z 1 when level)
      if(print_flag[5])  print_imu_MagData();       //Prints filtered magnetometer data direct from IMU (expected: -300 to 300)
      if(print_flag[6])  print_ahrs_RollPitchYaw();  //Prints roll, pitch, and yaw angles in degrees from ahrs_Madgwick filter (expected: degrees, 0 when level)
      if(print_flag[7])  print_control_PIDoutput();     //Prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1 to 1)
      if(print_flag[8])  print_out_MotorCommands(); //Prints the values being written to the motors (expected: 0 to 1)
      if(print_flag[9])  print_out_ServoCommands(); //Prints the values being written to the servos (expected: 0 to 1)
      if(print_flag[10]) print_loop_Rate();      //Prints the time between loops in microseconds (expected: 1000000 / loop_freq)
      if(print_flag[11]) print_bat(); //Prints battery voltage, current, Ah used and Wh used
      //Serial.printf("press:%.1f\ttemp:%.2f\t",baro.press_pa, baro.temp_c); //Prints barometer data
      if(print_need_newline) Serial.println();
      loop_rt = 0; //reset maximum
    }
  }

  void print_overview() {
    Serial.printf("CH%d:%d\t",1,rcin_pwm[0]);  
    Serial.printf("rcin_roll:%+.2f\t",rcin_roll);
    Serial.printf("gx:%+.2f\t",GyroX);
    Serial.printf("ax:%+.2f\t",AccX);
    Serial.printf("mx:%+.2f\t",MagX);
    Serial.printf("ahrs_roll:%+.1f\t",ahrs_roll);
    Serial.printf("roll_PID:%+.3f\t",roll_PID);  
    Serial.printf("m%d%%:%1.0f\t", 1, 100*out_command[0]);
    Serial.printf("sats:%d\t",(int)gps.sat);
    Serial.printf("loop_rt:%d\t",(int)loop_rt);
    Serial.printf("loop_cnt:%d\t",(int)loop_cnt); 
    print_need_newline = true;    
  }

  void print_rcin_RadioPWM() {
    Serial.printf("rcin_con:%d\t",rcin.connected());
    for(int i=0;i<RCIN_NUM_CHANNELS;i++) Serial.printf("pwm%d:%d\t",i+1,rcin_pwm[i]);
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
    Serial.printf("gx:%+.2f\tgy:%+.2f\tgz:%+.2f\t",GyroX,GyroY,GyroZ);
    print_need_newline = true;
  }

  void print_imu_AccData() {
    Serial.printf("ax:%+.2f\tay:%+.2f\taz:%+.2f\t",AccX,AccY,AccZ);
    print_need_newline = true;
  }

  void print_imu_MagData() {
    Serial.printf("mx:%+.2f\tmy:%+.2f\tmz:%+.2f\t",MagX,MagY,MagZ);
    print_need_newline = true;  
  }

  void print_ahrs_RollPitchYaw() {
    Serial.printf("roll:%+.1f\tpitch:%+.1f\tyaw:%+.1f\t",ahrs_roll,ahrs_pitch,ahrs_yaw);
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
    for(int i=out_MOTOR_COUNT;i<HW_OUT_COUNT;i++) Serial.printf("s%d%%:%1.0f\t", i-out_MOTOR_COUNT+1, 100*out_command[i]);
    print_need_newline = true;  
  }

  void print_loop_Rate() {
    static uint32_t loop_cnt_last = 0;
    Serial.printf("loop_dt:%d\t",(int)(loop_dt * 1000000.0));
    Serial.printf("loop_rt:%d\t",(int)loop_rt);
    Serial.printf("loop_rt_imu:%d\t",(int)loop_rt_imu);
    Serial.printf("loop_cnt:%d\t",(int)loop_cnt);  
    Serial.printf("loops:%d\t",(int)(loop_cnt - loop_cnt_last));  
    loop_cnt_last = loop_cnt;
    print_need_newline = true;
  }

  void print_bat() {
    Serial.printf("bat.v:%.2f\t",bat.v);
    Serial.printf("bat.i:%+.2f\t",bat.i);
    Serial.printf("bat.mah:%+.2f\t",bat.mah);
    Serial.printf("bat.wh:%+.2f\t",bat.wh); 
    print_need_newline = true;  
  }


};


CLI cli;