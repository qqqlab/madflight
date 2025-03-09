//test program for rcin.h - used with madflight 1.1.3

//--- RC RECEIVER
#define RCIN_USE  RCIN_USE_CRSF //RCIN_USE_CRSF, RCIN_USE_SBUS, RCIN_USE_DSM, RCIN_USE_PPM, RCIN_USE_PWM
#define RCIN_NUM_CHANNELS 6 //number of receiver channels (minimal 6)

/* Override default pins

//ESP32 WeMos LOLIN32-Lite
//RC Receiver:
#define HW_PIN_RCIN_RX    16
#define HW_PIN_RCIN_TX     4
#define HW_PIN_RCIN_INVERTER -1 //only used for STM32 targets
//*/

#define MF_TEST MF_TEST_RCIN // | MF_TEST_CLI
#include <madflight.h>


void setup() {
  Serial.begin(115200);
  while(!Serial);

  //cli.print_boardInfo(); //print board info and pinout
  hal_setup(); //hardware specific setup for spi and Wire (see hw_xxx.h)

  rcin.setup(); //Initialize radio communication. Set correct USE_RCIN_xxx user specified defines above. Note: rcin_Setup() function is defined in rcin.h, but normally no changes needed there.

  Serial.printf("HW_PIN_RCIN_RX:%d HW_PIN_RCIN_TX:%d\n", (int)HW_PIN_RCIN_RX, (int)HW_PIN_RCIN_TX );
}

void loop() {
  static uint32_t ts_last = 0;
  bool got_new_data = rcin.update();

  if(got_new_data) {
    uint32_t ts = micros();
    Serial.printf("dt:%d ", (int)(ts-ts_last));
    for(int i=0;i<RCIN_NUM_CHANNELS;i++) Serial.printf("pwm%d:%d ", i, rcin.pwm[i]);
    Serial.println();
    ts_last = ts;
  }

  delay(1);
}

