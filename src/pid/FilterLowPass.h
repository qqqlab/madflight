/*==========================================================================================
MIT License

Copyright (c) 2025 https://madflight.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
===========================================================================================*/

#pragma once

#include <math.h>

#ifndef constrain
  #define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
#endif

class FilterLowPass {
  public:
    float output = 0;      // Filter output value
    float alpha = 0;       // Filter alpha constant for sample frequency
    float cutoff_freq = 0; // Cutoff frequency [Hz]

    FilterLowPass() {}

    FilterLowPass(float sample_freq, float cutoff_freq, float input = 0) {
        begin(sample_freq, cutoff_freq, input);
    }

    void begin(float sample_freq, float cutoff_freq, float input = 0) {
        this->cutoff_freq = cutoff_freq;
        alpha = freq_to_alpha(cutoff_freq, sample_freq);
        output = input;
    }

    float update(float input) {
        output += alpha * (input - output);
        return output;
    }

    float update(float input, float dt) {
        float rc = 1.0f / (M_2_PI * cutoff_freq);
        float alpha_dt = dt / (dt + rc);
        output += alpha_dt * (input - output);
        return output;
    }

    static float freq_to_alpha(float cutoff_freq, float sample_freq) {
        float dt = 1.0f / sample_freq;
        float rc = 1.0f / (M_2_PI * cutoff_freq);
        return dt / (dt + rc);
    }
};
