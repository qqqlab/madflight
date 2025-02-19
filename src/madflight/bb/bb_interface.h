#pragma once

class BlackBox {
  public:
    //loggers
    virtual void log_baro() {}
    virtual void log_bat() {}
    virtual void log_gps() {}
    virtual void log_imu() {}
    virtual void log_mode(uint8_t fm, const char* name) {(void)fm;(void)name;}
    virtual void log_msg(const char* msg) {(void)msg;}
    virtual void log_parm(const char* name, float value, float default_value) {(void)name;(void)value;(void)default_value;}
    virtual void log_pid() {}
    virtual void log_att() {}
    virtual void log_ahrs() {}
    virtual void log_sys() {}

    //Blackbox Interface
    virtual void setup() {} //setup blackbox
    virtual void start() {} //start logging (create new file)
    virtual void stop()  {} //stop logging (closes file)
    virtual void erase() {} //erase all log files
    virtual void dir()   {} //list log files
    virtual void bench() {} //benchmark read/write to blackbox
    virtual void info()  {} //blackbox info (memory size, free space, etc.)
};

extern BlackBox &bb;
