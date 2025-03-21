/*==========================================================================================
MIT License

Copyright (c) 2023-2025 https://madflight.com

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

class BlackBox {
  public:
    //loggers
    virtual void log_bar() {}
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

extern BlackBox &bbx;
