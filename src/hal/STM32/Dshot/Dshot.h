class Dshot {
public:
  bool setup( int* pins, uint8_t cnt, int freq_khz = 300) {
    (void) pins;
    (void) cnt;
    (void) freq_khz;
    Serial.printf("ERROR: Dshot not implemented"); 
    return false;
  }

  void set_throttle( uint16_t* throttle) {
    (void) throttle;
  }
};
