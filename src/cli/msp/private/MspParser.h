#pragma once

#include "Msp.h"

class MspParser
{
  public:
    MspParser();
    void parse(char c, MspMessage& msg);
};

