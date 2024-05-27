//=================================================================================================
// LED
//=================================================================================================

#include "../interface.h"

/* INTERFACE
class Led {
  public:
    virtual void setup(int pin, uint8_t led_on_value) = 0;
    virtual void set(bool set_on) = 0;
    void on();
    void off();
    void toggle();
    void blink(int times);
  protected:
    bool state = false;
    int pin = -1;
    uint8_t led_on_value = 0;
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


class LedSingle : public Led {
  public:
    void setup(int pin, uint8_t led_on_value) {
      this->pin = pin;
      this->led_on_value = led_on_value;
      if(pin<0) return;
      pinMode(pin, OUTPUT);
      on();
    }

    void set(bool set_on) {
      if(pin<0) return;
      //if(led_state != set_on) Serial.printf("led_SwitchON(%d)\n", set_on);
      state = set_on;
      digitalWrite( pin, (set_on ? led_on_value : !led_on_value) );
    }
};

LedSingle led_instance;

Led &led = led_instance;