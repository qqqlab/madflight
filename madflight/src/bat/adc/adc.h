extern float bat_i;// = 0; //Battery current (A)
extern float bat_v;// = 0; //battery voltage (V)
extern float bat_mah;// = 0; //battery usage (Ah)
extern float bat_wh;// = 0; //battery usage (Wh)
extern float bat_factor_v;// = 8.04/13951; //voltage conversion factor, set this to 1 and enable print_bat(), then enter here: Actual Volt / bat_v ADC reading
extern float bat_factor_i;// = 1.0/847; //current conversion factor, set this to 1 and enable print_bat(), then enter here: Actual Amperes / bat_i ADC reading

void bat_setup() {
  pinMode(HW_PIN_BAT_V, INPUT);
  pinMode(HW_PIN_BAT_I, INPUT);
  analogReadResolution(16);
}

void bat_loop() {
  static uint32_t ts = micros();
  uint32_t now = micros();
  if(now - ts >= 10000) {
    uint32_t dt = now - ts;
    float dt_h = dt / 3600e6;
    ts = now;
    bat_v = bat_factor_v * analogRead(HW_PIN_BAT_V);    
    bat_i = bat_factor_i * analogRead(HW_PIN_BAT_I);
    bat_mah += bat_i * dt_h * 1000;
    bat_wh += bat_v * bat_i * dt_h;
  }
}