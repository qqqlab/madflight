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

#include "MF_Filter.h"
#include <math.h>


//=================================================================
// MF_Filter
//=================================================================

MF_Filter* MF_Filter::create(MF_FilterType typ) {
    switch (typ) {
    case MF_FilterType::mf_NONE:
        return new MF_FilterNone();
    case MF_FilterType::mf_PT1:
        return new MF_FilterLowPT1();
    case MF_FilterType::mf_PT2:
        return new MF_FilterLowPT2();
    case MF_FilterType::mf_PT3:
        return new MF_FilterLowPT3();
    case MF_FilterType::mf_BIQUAD:
        return new MF_FilterLowBiquad();
    case MF_FilterType::mf_NOTCH:
        return new MF_FilterNotchBiquad();
    default:
        //should not get here
        return new MF_FilterNone();
    }
}

bool MF_Filter::setup(MF_Filter* &f, MF_FilterType typ, float f_sample, float para1, float para2) {
    if(!f || f->get_type() != typ) {
        delete f;
        f = MF_Filter::create(typ);
    }
    return f->init(f_sample, para1, para2);
}

//=================================================================
// MF_FilterNone
//=================================================================

MF_FilterType MF_FilterNone::get_type() {
    return MF_FilterType::mf_NONE;
}

bool MF_FilterNone::init(float f_sample, float f_cut, float dummy) {
    (void)dummy;
    (void)f_cut;
    (void)f_sample;
    return true;
}

float MF_FilterNone::apply(float input) {
    return input;
}


//=================================================================
// MF_FilterLowPT1
//=================================================================

// PTn cutoff correction = 1 / sqrt(2^(1/n) - 1)
#define CUTOFF_CORRECTION_PT2 1.553773974f
#define CUTOFF_CORRECTION_PT3 1.961459177f

MF_FilterType MF_FilterLowPT1::get_type() {
    return MF_FilterType::mf_PT1;
}

static float pt_calc_k(float f_cut, float f_sample) {
    //default to no filtering when parameters are incorrect
    if(f_sample <= 0 || f_cut < 0) return 1; 

    //approximation
    //float omega = 2.0f * M_PI * f_cut / f_sample;
    //return omega / (omega + 1.0f);

    return 1 - exp(-2.0f * M_PI * f_cut / f_sample); //exact
}

bool MF_FilterLowPT1::init(float f_sample, float f_cut, float dummy) {
    (void)dummy;
    if(f_cut >= f_sample || f_cut <= 0 || f_sample <= 0) {
        //default to no filtering
        k = 1; 
        s0 = 0;
        return false;
    }else{
        k = pt_calc_k(f_cut, f_sample);
        s0 = 0;
        return true;
    }
}

float MF_FilterLowPT1::apply(float input) {
    s0 = s0 + k * (input - s0);
    return s0;
}

//=================================================================
// MF_FilterLowPT2
//=================================================================

MF_FilterType MF_FilterLowPT2::get_type() {
    return MF_FilterType::mf_PT2;
}

bool MF_FilterLowPT2::init(float f_sample, float f_cut, float dummy) {
    (void)dummy;
    if(f_cut >= f_sample || f_cut <= 0 || f_sample <= 0) {
        //default to no filtering
        k = 1; 
        s0 = 0;
        s1 = 0;
        return false;
    }else{
        // shift f_cut to satisfy -3dB cutoff condition
        k = pt_calc_k(f_cut * CUTOFF_CORRECTION_PT2, f_sample);
        s0 = 0;
        s1 = 0;
        return true;
    }
}

float MF_FilterLowPT2::apply(float input) {
    s0 = s0 + k * (input - s0);
    s1 = s1 + k * (s0 - s1);
    return s1;
}


//=================================================================
// MF_FilterLowPT3
//=================================================================

MF_FilterType MF_FilterLowPT3::get_type() {
    return MF_FilterType::mf_PT3;
}

bool MF_FilterLowPT3::init(float f_sample, float f_cut, float dummy) {
    (void)dummy;
    if(f_cut >= f_sample || f_cut <= 0 || f_sample <= 0) {
        //default to no filtering
        k = 1; 
        s0 = 0;
        s1 = 0;
        s2 = 0;
        return false;
    }else{
        // shift f_cut to satisfy -3dB cutoff condition
        k = pt_calc_k(f_cut * CUTOFF_CORRECTION_PT3, f_sample);
        s0 = 0;
        s1 = 0;
        s2 = 0;
        return true;
    }
}

float MF_FilterLowPT3::apply(float input) {
    s0 = s0 + k * (input - s0);
    s1 = s1 + k * (s0 - s1);
    s2 = s2 + k * (s1 - s2);
    return s2;
}


//=================================================================
// MF_FilterLowBiquad
//=================================================================

MF_FilterType MF_FilterLowBiquad::get_type() {
    return MF_FilterType::mf_BIQUAD;
}

bool MF_FilterLowBiquad::init(float f_sample, float f_cut, float Q) {
    if(f_cut >= f_sample || f_cut <= 0 || f_sample <= 0) {
        return false;
    }else{
        const float k = (Q > 0 ? 1 / Q : 1.41421356237f); // default Q = 1/sqrt(2)
        g = tan(M_PI * f_cut / f_sample); // Pre-warp frequency to the Nyquist limit
        D = 1.0f / (1.0f + g * (g + k)); // Linear system denominator
        gD = g * D;
        // zero initial s0
        s2 = 0.0f;
        s1 = 0.0f;
        return true;
    }
}

// Computes a SVF MF_Filter in TPT form on a sample
// optimized version: 5 multiplications, 5 additions vs 7,6 on-opt
float MF_FilterLowBiquad::apply(float input) {
    const float bp = D * s1 + gD * (input - s2); //bandpass
    const float lp = g * bp + s2; //lowpass
    s1 = 2.0f * bp - s1;
    s2 = 2.0f * lp - s2;
    return lp; // Returns the lowpass node
}

//=================================================================
// MF_FilterNotchBiquad
//=================================================================

MF_FilterType MF_FilterNotchBiquad::get_type() {
    return MF_FilterType::mf_NOTCH;
}

bool MF_FilterNotchBiquad::init(float f_sample, float f_notch, float Q) {
    if(f_notch <= 0 || f_sample <= 0) {
        return false;
    }else{
        float dt = 1 / f_sample;
        const float g = tan(M_PI * f_notch * dt);
        const float k = (Q > 0 ? 1 / Q : 1.41421356237f); // default Q = 1/sqrt(2)
        D = 1.0f / (1.0f + g * (g + k));
        gDk = g * D * k;
        g_k  = g * Q; // Division replaced by multiplication
        // zero initial samples
        s2 = 0.0f;
        s1k = 0.0f;
        return true;
    }
}

// Computes a SVF MF_Filter in TPT form on a sample
// optimized version: 5 multiplications, 6 additions vs 7,8 non-opt
float MF_FilterNotchBiquad::apply(float input) {
    const float bpk = D * s1k + gDk * (input - s2); // This is:  BP * k)
    const float lp = g_k * bpk + s2;       // Equivalent to: g * BP + s2
    // Update scaled states
    s1k = 2.0f * bpk - s1k;
    s2 = 2.0f * lp - s2;
    return input - bpk;
}


//=================================================================
// MF_FilterLowBiquad + MF_FilterNotchBiquad non-optimized
//=================================================================
#if 0 //non-optimized versions

//see https://synthengineer.com/blog/zero-delay-feedback-filters
class MF_FilterLowBiquad : public MF_Filter {
public:
    // State variables
    float s1 = 0; // integrator 1 s0 (BP)
    float s2 = 0; // integrator 2 s0 (LP)
    // Coefficients
    float D = 0; // precomputed denominator
    float g = 0; // tan(pi * fc / fs)
    float k = 0; // 1 / Q

    MF_FilterType get_type() override {
        return MF_FilterType::mf_BIQUAD;
    }

    bool init(float f_sample, float f_cut, float Q) override {
        if(f_cut >= f_sample || f_cut <= 0 || f_sample <= 0) {
            return false;
        }else{
            k = (Q > 0 ? 1 / Q : 1.41421356237f); // default Q = 1/sqrt(2)
            g = tan(M_PI * f_cut / f_sample); // Pre-warp frequency to the Nyquist limit
            D = 1.0f / (1.0f + g * (g + k)); // Linear system denominator
            // zero initial s0
            s2 = 0.0f;
            s1 = 0.0f;
            return true;
        }
    }

    // Computes a SVF MF_Filter in TPT form on a sample
    // 7 multiplications, 7 additions
    float apply(float x) override {
        // Solve the algebraic loop
        float hp = (x - k * s1 - s2 - g * s1) * D;

        // Compute outputs from integrators
        float bp = g * hp + s1;
        float lp = g * bp + s2;

        // Update states
        s1 = 2.0f * bp - s1;
        s2 = 2.0f * lp - s2;

        return lp; // Returns the lowpass node
    }
};

class MF_FilterNotchBiquad : public MF_Filter {
public:
    // State variables
    float s1 = 0; // integrator 1 s0 (BP)
    float s2 = 0; // integrator 2 s0 (LP)
    // Coefficients
    float D = 0; // precomputed denominator
    float g = 0; // tan(pi * fc / fs)
    float k = 0; // 1 / Q

    MF_FilterType get_type() override {
        return MF_FilterType::NotchBiquad;
    }

    bool init(float f_sample, float f_cut, float Q) override {
        if(f_cut >= f_sample || f_cut <= 0 || f_sample <= 0) {
            return false;
        }else{
            k = (Q > 0 ? 1 / Q : 1.41421356237f); // default Q = 1/sqrt(2) 
            g = tan(M_PI * f_cut / f_sample); // Pre-warp frequency to the Nyquist limit
            D = 1.0f / (1.0f + g * (g + k)); // Linear system denominator
            k = 1.0f / Q;
            // zero initial s0
            s2 = 0.0f;
            s1 = 0.0f;
            return true;
        }
    }

    // Computes a SVF MF_Filter in TPT form on a sample
    // 7 multiplications, 8 additions
    float apply(float x) override {
        // Solve the algebraic loop
        float hp = (x - k * s1 - s2 - g * s1) * D;

        // Compute outputs from integrators
        float bp = g * hp + s1;
        float lp = g * bp + s2;

        // Update states
        s1 = 2.0f * bp - s1;
        s2 = 2.0f * lp - s2;

        return hp + lp;
    }
};
#endif // #if 0 non-optimized versions
