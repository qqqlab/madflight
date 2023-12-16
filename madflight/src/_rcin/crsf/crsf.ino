/*==========================================================================================
CRSF/ELRS receiver demo program

Connect receiver TX to HW_PIN_RCIN_RX, receiver RX to HW_PIN_RCIN_TX
==========================================================================================*/

#include "crsf.h"
CRSF crsf;

#if defined ARDUINO_ARCH_ESP32
    const int HW_PIN_RCIN_RX  = 35;
    const int HW_PIN_RCIN_TX  = 32;
    HardwareSerial *rcin_Serial = &Serial1; //&Serial1 or &Serial2 (&Serial is used for debugging)
    void hw_setup()
    {
      rcin_Serial->setPins(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);
    }
#elif defined ARDUINO_ARCH_RP2040
    const int HW_PIN_RCIN_RX  = 1; //uart0: 1(default), 5, 13, 17   uart1: 5, 9(default)
    const int HW_PIN_RCIN_TX  = 0; //uart0: 0(default), 4, 12, 16   uart1: 4, 8(default)
    SerialUART *rcin_Serial = new SerialUART(uart0, HW_PIN_RCIN_TX, HW_PIN_RCIN_RX); //uart0 or uart1
    void hw_setup() 
    { 
    }
#else 
    #error "Unknown hardware architecture"
#endif

uint32_t print_ts;

void setup() {
    Serial.begin(115200);
    while(!Serial);
    Serial.println("CRSF DEMO");
    
    hw_setup();
    rcin_Serial->begin(CRSF_BAUD);

    print_ts = millis();
}

void loop() {
    if(rcin_Serial->available()) {
        int c = rcin_Serial->read();
        //print received data
        if(c == CRSF_ADDRESS_FLIGHT_CONTROLLER) Serial.printf("\nreceived: ");
        Serial.printf("%02x ",c);
        if(crsf.update(c)) {
            //print decoded rc data
            Serial.print(" decoded RC: ");
            for(int i=0;i<16;i++) Serial.printf("ch%d:%d ",i,crsf.channel[i]);
            print_ts = millis();
        }
    }

    //print connection status if nothing received for 1 second
    if(millis() - print_ts > 1000) {
       Serial.printf("is_connected:%d ",crsf.is_connected());
       for(int i=0;i<16;i++) Serial.printf("ch%d:%d ",i,crsf.channel[i]);
       Serial.println();
       print_ts = millis();
    }
}