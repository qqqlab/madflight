class DshotBidir {
public:
  bool setup( int* pins, uint8_t cnt, int freq_khz = 300) {
    (void) pins;
    (void) cnt;
    (void) freq_khz;
    Serial.printf("ERROR: DshotBidir not implemented");
    return false;
  }

  void set_throttle( uint16_t* throttle)  {
    (void) throttle;
  }

  //get erpm values, call before set_throttle. Returns negative values on error.
  void get_erpm( int* erpm) {
    (void) erpm;
  }
};
