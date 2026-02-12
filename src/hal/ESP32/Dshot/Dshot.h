class Dshot {
public:
  bool setup( int* pins, uint8_t cnt, int freq_khz = 300) {
    Serial.printf("OUT: ERROR Dshot not implemented\n"); 
    return false;
  }

  void set_throttle( uint16_t* throttle) {}
};
