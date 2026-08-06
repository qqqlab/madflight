/*==========================================================================================
MIT License

Copyright (c) 2026 https://madflight.com

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
#include "../cfg/cfg.h"

/* moved to param.yaml.h
enum class MF_FilterType {
  mf_NONE,
  mf_PT1,
  mf_PT2,
  mf_PT3,
  mf_BIQUAD,
  mf_NOTCH,
};
*/

class MF_Filter {
public:
    virtual float apply(float input) = 0;
    virtual MF_FilterType get_type() = 0;

    static bool setup(MF_Filter* &f, MF_FilterType typ, float f_sample, float para1, float para2);
private:
    virtual bool init(float f_sample, float para1, float para2) = 0;
    static MF_Filter* create(MF_FilterType typ);
};

class MF_FilterNone : public MF_Filter {
public:
    MF_FilterType get_type() override;
    bool init(float f_sample, float f_cut, float dummy = 0) override;
    float apply(float input) override;
};

class MF_FilterLowPT1 : public MF_Filter {
private:
    // State variables
    float s0 = 0;
    // Coefficients
    float k = 1; //default to no filtering
public:
    MF_FilterType get_type() override;
    bool init(float f_sample, float f_cut, float dummy = 0) override;
    float apply(float input) override;
};

class MF_FilterLowPT2 : public MF_Filter {
private:
    // State variables
    float s0 = 0;
    float s1 = 0;
    // Coefficients
    float k = 1; //default to no filtering
public:
    MF_FilterType get_type() override;
    bool init(float f_sample, float f_cut, float dummy = 0) override;
    float apply(float input) override;
};

class MF_FilterLowPT3 : public MF_Filter {
private:
    // State variables
    float s0 = 0;
    float s1 = 0;
    float s2 = 0;
    // Coefficients
    float k = 1; //default to no filtering
public:
    MF_FilterType get_type() override;
    bool init(float f_sample, float f_cut, float dummy = 0) override;
    float apply(float input) override;
};

//see https://synthengineer.com/blog/zero-delay-feedback-filters
class MF_FilterLowBiquad : public MF_Filter {
private:
    // State variables
    float s1 = 0; // integrator 1 s0 (BP)
    float s2 = 0; // integrator 2 s0 (LP)
    // Coefficients
    float D = 0;  // precomputed denominator
    float g = 0;  // tan(pi * fc / fs)
    float gD = 0; // g * D
public:
    MF_FilterType get_type() override;
    bool init(float f_sample, float f_cut, float Q = 0) override;
    float apply(float input) override;
};

//see https://synthengineer.com/blog/zero-delay-feedback-filters
class MF_FilterNotchBiquad : public MF_Filter {
private:
    // State variables
    float s1k = 0; // integrator 1 (BP * k)
    float s2 = 0;  // integrator 2 (LP)
    // Coefficients
    float D = 0;   // precomputed denominator
    float gDk = 0; // g*D*k
    float g_k = 0; // g/k
public:
    MF_FilterType get_type() override;
    bool init(float f_sample, float f_cut, float Q = 0) override;
    float apply(float input) override;
};
