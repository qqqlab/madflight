#pragma once

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
