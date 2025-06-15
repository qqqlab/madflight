class Dshot {
public:
  bool setup( int* pins, uint8_t cnt, int freq_khz = 300) {
    Serial.printf("ERROR: Dshot not implemented"); 
    return false;
  }

  void set_throttle( uint16_t* throttle) {}
};
