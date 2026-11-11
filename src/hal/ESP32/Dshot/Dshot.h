#include "../../../tbx/MF_printf.h"
#include <stdint.h>

class Dshot {
public:
  bool setup( int* pins, uint8_t cnt, int freq_khz = 300) {
    MF_printf("OUT: ERROR Dshot not implemented\n"); 
    return false;
  }

  void set_throttle( uint16_t* throttle) {}
};
