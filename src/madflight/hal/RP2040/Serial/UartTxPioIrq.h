/*==========================================================================================
UartTxPioIrq.h - High Performance RP2 Buffered TX PIO UART Driver

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
#pragma once

#include "uart_tx.pio.h"
#include "SerialRingBuf.h"

#define UARTTXPIOIRQ_NUM 6

//prototypes
class UartTxPioIrq;
void _UartTxPioIrq_irq_handler(void);

//global instances (first non-null, then null entries)
UartTxPioIrq* _UartTxPioIrq_instances[UARTTXPIOIRQ_NUM] = {};

class UartTxPioIrq {
    friend void _UartTxPioIrq_irq_handler(void);
private:
    bool setup_done = false;
    uint8_t pin = 0xFF;
    uint16_t buflen = 1;

    //Ringbugger vars
    SerialRingBuf rb;
    uint8_t *rb_buf = nullptr;

    //PIO vars
    PIO pio = nullptr;
    uint sm;
    uint offset;
    uint32_t int0_mask = 0; //pre-calculated inte0/ints0 mask value

    //IRQ handler for this instance
    void __not_in_flash_func(_irq_handler)(void) {
        if(pio->ints0 & int0_mask) {
            while( (pio->fstat & (1u << (PIO_FSTAT_TXFULL_LSB + sm))) == 0) { //same as !pio_sm_is_tx_fifo_full(pio, sm) without overhead
                uint8_t c;
                if(!rb.pop(&c)) {
                    //no more data, disable interrupt
                    hw_clear_bits(&pio->inte0, int0_mask); //Atomically clear the specified bits to 0 in a HW register (same as pio_set_irq0_source_enabled without overhead)
                    return;
                }
                pio->txf[sm] = c; //same as pio_sm_put without overhead               
            }
        }
    }

    bool panic(const char* s) {
        end();
        Serial.println(s);
        return false;
    }

public:
    UartTxPioIrq(uint8_t pin, uint16_t buflen = 256) {
        this->pin = pin;
        if(buflen < 1) buflen = 1;
        this->buflen = buflen;
    }

    ~UartTxPioIrq() {
        end();
    }

    bool begin(uint32_t baud) {
        //check pin
        if(pin == 0xFF) return false;

        //check baud
        if(baud > clock_get_hz(clk_sys) / 8) return panic("PANIC: UartTxPioIrq baud rate too high"); //max baud = f_cpu/8 >= 10M 
        if(baud < clock_get_hz(clk_sys) / 8 / 65536) return panic("PANIC: UartTxPioIrq baud rate too low"); //min baud = f_cpu/8/65536 <= 286 @ 150Mhz
            
        //only change baud if setup was completed    
        if(setup_done) {
            uart_tx_program_init(pio, sm, offset, pin, baud);
            return true;
        }

        //ringbuf
        rb_buf = new uint8_t[buflen + 1];
        if(!rb_buf) {
            return panic("PANIC: UartTxPioIrq out of memory");
        }
        rb.begin(rb_buf, buflen + 1);

        // This will find a free pio and state machine for our program and load it for us
        // We use pio_claim_free_sm_and_add_program_for_gpio_range (for_gpio_range variant)
        // so we will get a PIO instance suitable for addressing gpios >= 32 if needed and supported by the hardware
        bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&uart_tx_program, &pio, &sm, &offset, pin, 1, true);
        if(!success) {
            return panic("PANIC: UartTxPioIrq no free sm");
        }

        uart_tx_program_init(pio, sm, offset, pin, baud);

        //disable state machine tx_fifo_not_full interrupt
        //int0_mask is the pre-calculated inte0/ints0 mask value
        switch(sm) {
            case 0: int0_mask = 1 << pis_sm0_tx_fifo_not_full; break;
            case 1: int0_mask = 1 << pis_sm1_tx_fifo_not_full; break;
            case 2: int0_mask = 1 << pis_sm2_tx_fifo_not_full; break;
            case 3: int0_mask = 1 << pis_sm3_tx_fifo_not_full; break;
            default:
              return panic("PANIC: UartTxPioIrq invalid PIO SM ID\n");
        }
        hw_clear_bits(&pio->inte0, int0_mask); //start with interrupt disabled

        //enable interrupt
        const uint irqn = 0; //0 for PIOx_IRQ_0 or 1 for PIOx_IRQ_1 etc where x is the PIO number
        const int pio_irq = pio_get_irq_num(pio, irqn); //get IRQ for a PIO hardware instance
        irq_add_shared_handler(pio_irq, _UartTxPioIrq_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
        irq_set_enabled(pio_irq, true);

        //register instance
        int i;
        for(i = 0; i < UARTTXPIOIRQ_NUM; i++) {
            if(!_UartTxPioIrq_instances[i]) _UartTxPioIrq_instances[i] = this;
            break;
        }
        if(i >= UARTTXPIOIRQ_NUM) {
            return panic("PANIC: UartTxPioIrq too many instances");
        }

        setup_done = true;
        return true;    
    }

    void end() {
        setup_done = false; //NOTE: potential race condition when other thread is writing at this time

        //don't disable IRQ, it might be in use by something else. Removing the instance will disable the handler.

        //remove UartTxPioIrq instance (and reorder list to keep non-nulls first)
        for(int i = 0; i < UARTTXPIOIRQ_NUM; i++) {
            if(_UartTxPioIrq_instances[i] == this) {
                //find last non-null entry or 0
                int j;
                for(j = UARTTXPIOIRQ_NUM-1; j > 0; j--) {
                    if(_UartTxPioIrq_instances[i]) break;
                }
                //replace instance with last entry
                _UartTxPioIrq_instances[i] = _UartTxPioIrq_instances[j];
                //remove last entry
                _UartTxPioIrq_instances[j] = nullptr;
                break;
            }
        }

        // This will free resources and unload our pio program
        if(pio) {
            pio_remove_program_and_unclaim_sm(&uart_tx_program, pio, sm, offset);
            pio = nullptr;
        }

        //delete ringbuffer buffer
        delete[] rb_buf;
    }

    //write as much fits in ringbuffer, then write blocking
    int write_blocking(const uint8_t *buf, int len) {
        if(!setup_done) return 0;

        //write as much as possible into ringbuffer
        int i = rb.push(buf, len);
        hw_set_bits(&pio->inte0, int0_mask); //enable interrupt

        //write blocking
        for(; i < len; i++) {
            while(rb.push(buf[i]) == 0) {
                hw_set_bits(&pio->inte0, int0_mask); //re-enable interrupt (was potentially disabled by irq handler)
                tight_loop_contents();
            }
        }
        return i;
    }

    //write all or nothing
    int write(const uint8_t *buf, int len) {
        if(!setup_done) return 0;

        if(rb.free_space() < len) return 0; //the calling thread should be the only writer thread, so rb.free_space() not decrease after this call

        rb.push(buf, len);
        hw_set_bits(&pio->inte0, int0_mask); //enable interrupt
        return len;
    }

    int availableForWrite() {
        if(!setup_done) return 0;
        return rb.free_space();
    }    
};

void __not_in_flash_func(_UartTxPioIrq_irq_handler)(void) {
    //_UartTxPioIrq_instances is ordered, exit on first null
    for(uint8_t i = 0; i < UARTTXPIOIRQ_NUM; i++) {
      if(_UartTxPioIrq_instances[i]) {
        _UartTxPioIrq_instances[i]->_irq_handler();
      }else{
        break;
      }
    }
}