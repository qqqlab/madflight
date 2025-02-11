#pragma once

//blocking
int autobaud(int pin, int timeout = 1000, int minpulses = 100);

//async
void autobaud_begin(int pin);
int autobaud_get_baud(int minpulses = 100);
void autobaud_end();
