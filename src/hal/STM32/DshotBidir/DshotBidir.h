class DshotBidir {
public:
  bool setup( int* pins, uint8_t cnt, int freq_khz = 300) {
    Serial.printf("ERROR: DshotBidir not implemented");
    return false;
  }

  void set_throttle( uint16_t* throttle) {}

  //get erpm values, call before set_throttle. Returns negative values on error.
  void get_erpm( int* erpm) {}
};
