#pragma once

class PID {
  public:
    float PID = 0; //PID output value
};

extern PID PIDroll;
extern PID PIDpitch;
extern PID PIDyaw;
