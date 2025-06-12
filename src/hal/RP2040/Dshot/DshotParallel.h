/*==========================================================================================
MIT License

Copyright (c) 2025 https://github.com/qqqlab

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
==========================================================================================*/

/* DSHOT PROTOCOL

Available modes: Dshot150, Dshot300, Dshot600, Dshot1200. The number is the bit rate in kHz.

Frame Structure

11 bit throttle: 2048 possible values. 0 is reserved for disarmed. 1-47 are reserved for special commands. Leaving 48 to 2047 (2000 steps) for the actual throttle value
1 bit telemetry request - if this is set, telemetry data is sent back via a separate channel
4 bit CRC: (Cyclic Redundancy) Check to validate data (throttle and telemetry request bit)

Resulting in a frame with the following structure:

SSSSSSSSSSSTCCCC (16 bits, MSB transmitted first)

Pause (output low) between frames: 21 bits recommended, 2 minimum

Bit Structure

A bit period (6.667 us for Dshot150) is split in 8 equal parts. A zero-bit is encoded as 3H 5L, and a one-bit is encoded as 6H 2L.

*/


#pragma once

#include "dshot_parallel.pio.h"

class DshotParallel {
private:
    bool setup_done = false;
    uint8_t pin_count;
    PIO pio;
    uint sm;
    uint offset;
    uint32_t write_ts = 0; //last write timestamp

public:
    uint32_t interval_us = 0; //minimal interval between two writes in us (16 bits transmission + 21 bits pause)

    ~DshotParallel() {
        end();
    }

    bool begin(int pin, uint8_t pin_count, int rate_khz) {
        end();

        if(pin < 0 || pin_count < 1 || pin_count > 8 || rate_khz < 0) return false;

        this->pin_count = pin_count;
        this->interval_us = (16 + 21) * 1000 / rate_khz; 

        // This will find a free pio and state machine for our program and load it for us
        // We use pio_claim_free_sm_and_add_program_for_gpio_range (for_gpio_range variant)
        // so we will get a PIO instance suitable for addressing gpios >= 32 if needed and supported by the hardware
        bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&dshot_parallel_program, &pio, &sm, &offset, pin, pin_count, true);
        if(!success) return false;

        dshot_parallel_program_init(pio, sm, offset, pin, pin_count, rate_khz * 1000);

        setup_done = true;
        return true;
    }

    void end() {
        if(setup_done) {
            setup_done = false;

            // This will free resources and unload our program
            pio_remove_program_and_unclaim_sm(&dshot_parallel_program, pio, sm, offset);
        }
    }

    //set throttle for all ESCs
    bool set_throttle(uint16_t *throttle) {
        uint16_t values[8];
        for(int i = 0; i < pin_count; i++) {
          values[i] = ( throttle[i] == 0 ? 0 : throttle[i] + 48);
        }
        return write(values);
    }

    //only write if last write was more than interval_us in the past
    bool write(uint16_t *values) {
        if(micros() - write_ts < interval_us) return false;
        if(!write_raw(values)) return false;
        write_ts = micros();
        return true;
    }

    //immediately write, don't check interval
    bool write_raw(uint16_t *values) {
        if(!setup_done) return false;

        //convert values into frames (i.e. shift bits, add T=0 no telemetry bit, add CRC)
        uint16_t frames[8];
        for(int ch = 0; ch < pin_count; ch++) {
            uint16_t f = values[ch];
            if(f > 2047) f = 2047; //clip to max throttle
            f <<= 5; //shift and add T=0 no telemetry bit
            uint16_t crc = (f >> 4) ^ (f >> 8) ^ (f >> 12);
            f |= (crc & 0x0F); //add CRC
            frames[ch] = f;
        }

        //convert frames into fifo stream
        uint32_t d[4] = {};
        for(int bit = 15; bit >= 0; bit--) {
            uint32_t word = (15 - bit) / 4;
            uint32_t shift = ((15 - bit) % 4) * 8; 
            for(int ch=0; ch < pin_count; ch++) {
                uint32_t chmask = (1u << ch);
                if( frames[ch] & (1u << bit) ) {
                    d[word] |= (chmask << shift);
                }
            }
        }

        //send the frames to the PIO
        pio_sm_set_enabled(pio, sm, false); //disable PIO while filling fifos
        pio_sm_clear_fifos(pio, sm); //just to be sure
        for(int i = 0; i < 4; i++) {
            pio_sm_put(pio, sm, d[i]);
        }
        pio_sm_set_enabled(pio, sm, true); //enable PIO to send our frames in parallel

        return true;
    }

};