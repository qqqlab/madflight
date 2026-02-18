#pragma once

#include <stdint.h>

class Msp {
  public:
    static bool process_byte(uint8_t c, bool msp_connected); //returns true if command was processed
};
