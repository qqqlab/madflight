
#pragma once



class CLI {

public:

  void setup() {
    print_off();
  }

  void loop() {
    while(Serial.available()) {
      char c = Serial.read();
      if(c=='\r' || c=='\n') {
        processCmd();
      }else{
        cmdline += c;
      }
    }
    
    print_loop();
  }

  void pinout() {
    Serial.printf("HW_PIN_LED=%d\n", HW_PIN_LED);
    Serial.printf("HW_PIN_SPI_MOSI=%d MISO=%d SCLK=%d\n", HW_PIN_SPI_MOSI, HW_PIN_SPI_MISO, HW_PIN_SPI_SCLK);
    Serial.printf("HW_PIN_IMU_CS=%d\n",HW_PIN_IMU_CS);  
    Serial.printf("HW_PIN_IMU_EXTI=%d\n",HW_PIN_IMU_EXTI);
    Serial.printf("HW_PIN_I2C_SDA=%d SCL=%d\n", HW_PIN_I2C_SDA, HW_PIN_I2C_SCL);
    Serial.printf("HW_PIN_OUT[%d]=%d", HW_OUT_COUNT, HW_PIN_OUT[0]);  for(int i=1; i<HW_OUT_COUNT; i++) Serial.printf(",%d", HW_PIN_OUT[i]);  Serial.println();
    Serial.printf("HW_PIN_RCIN_RX=%d TX=%d\n", HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);
    Serial.printf("HW_PIN_GPS_RX=%d TX=%d\n", HW_PIN_GPS_RX, HW_PIN_GPS_TX);
  }

  void i2c_scan() {
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

  void welcome() {
    Serial.println("CLI Command Line Interface Started - Type help for help");
  }

  void help() {
    Serial.printf(
    "--info--\n"
    "help or ? This info\n"
    "pinout    Pinout\n"
    "i2c       I2C scan\n"
    "--print--\n"
    "poff      Printing off\n"
    "po        Overview: pwm1, rcin_roll, gyroX, accX, magX, ahrs_roll, pid_roll, motor1, loop_rt\n"
    "ppwm      Radio pwm (expected: 1000 to 2000)\n"
    "pradio    Scaled radio (expected: -1 to 1)\n"
    "pgyro     Filtered gyro (expected: -250 to 250, 0 at rest)\n"
    "pacc      Filtered accelerometer (expected: -2 to 2; x,y 0 when level, z 1 when level)\n"
    "pmag      Filtered magnetometer (expected: -300 to 300)\n"
    "prpy      AHRS roll, pitch, and yaw (expected: degrees, 0 when level)\n"
    "ppid      PID output (expected: -1 to 1)\n"
    "pmot      Motor output (expected: 0 to 1)\n"
    "pservo    Servo output (expected: 0 to 1)\n"
    "ploop     Loop timing in microseconds (expected: 1000000 / loop_freq)\n"
    "pbat      Battery voltage, current, Ah used and Wh used\n"
    "--black box--\n"
    "bbdump\n"
    "bbstart\n"
    "bbstop\n"
    "bberase\n"
    );
  }
  
private:

  String cmdline = "";

  void processCmd() {
    String cmd = cmdline;
    //for(int i=0;i<cmdline.length();i++) {
    //}
    cmd.toLowerCase();
    cmd.trim();
    
    if(cmd=="help" || cmd=="?") {
      help();
    }else if(cmd == "pinout") {
      pinout();
    }else if(cmd == "i2c") {
      i2c_scan();
    }else if(cmd == "poff") {
      print_off();
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
    }else if(cmd == "prpy") {
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
    }else if(cmd != "") {
      Serial.println("ERROR Unknown command - Type help for help");
    }
    cmdline = "";
  }

//========================================================================================================================//
//                                                PRINT FUNCTIONS                                                         //
//========================================================================================================================//

  uint32_t print_time = 0;
  bool print_need_newline = false;
#define CLI_PRINT_FLAG_COUNT 12
  bool print_flag[CLI_PRINT_FLAG_COUNT];

  void print_off() {
    for(int i=0;i<CLI_PRINT_FLAG_COUNT;i++) print_flag[i] = false;
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