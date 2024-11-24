//=================================================================================================
// LED
//=================================================================================================

#include "../interface.h"

/* INTERFACE
class Led {
  public:
    virtual void setup() = 0;
    virtual void set(bool set_on) = 0;
    void on();
    void off();
    void toggle();
    void blink(int times);
    void enable();
  protected:
    bool state = false;
    int pin = -1;
    uint8_t led_on_value = 0;
    bool enabled = false;
};

extern Led &led;
*/

void Led::on() {
  set(true);
}

void Led::off() {
  set(false);
}

void Led::toggle() {
  set(!state);
}

void Led::blink(int times) {
  for(int i=0;i<times;i++) {
    off();
    delay(500);
    on();
    delay(500);
  }
  off();
  delay(2000);
}

void Led::enable() {
  enabled = true;
  off();
}

//=================================================================================================
// None or undefined
//=================================================================================================
#ifndef HW_PIN_LED

class LedNone : public Led {
  public:
    void setup() {}

    void set(bool set_on) {}
};

LedNone led_instance;

//=================================================================================================
// Single LED
//=================================================================================================
#else
  
#ifndef HW_LED_ON
  #define HW_LED_ON 0
#endif
class LedSingle : public Led {
  public:
    void setup() {
      this->pin = HW_PIN_LED;
      this->led_on_value = HW_LED_ON;
      if(pin<0) return;
      pinMode(pin, OUTPUT);
      enabled = true;
      on();
      enabled = false;
    }

    void set(bool set_on) {
      if(!enabled || pin<0) return;
      //if(led_state != set_on) Serial.printf("led_SwitchON(%d)\n", set_on);
      state = set_on;
      digitalWrite( pin, (set_on ? led_on_value : !led_on_value) );
    }
};

LedSingle led_instance;

//=================================================================================================
#endif

Led &led = led_instance;