/*==========================================================================================
UartRxPioIrq.h - High Performance RP2 Buffered RX PIO UART Driver

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

#include "../pio_registry.h"

#include "uart_rx.pio.h"
#include "SerialRingBuf.h"

#define UARTRXPIOIRQ_NUM 6

//prototypes
class UartRxPioIrq;
void _UartRxPioIrq_irq_handler(void);

//global instances (first non-null, then null entries)
UartRxPioIrq* _UartRxPioIrq_instances[UARTRXPIOIRQ_NUM] = {};

class UartRxPioIrq {
    friend void _UartRxPioIrq_irq_handler(void);
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
    uint irq_index;
    uint fstat_mask; //pre-calculated fstat mask value
    io_rw_8 *rxfifo_shift; //pre-calculated address for 8-bit read from the uppermost byte of the FIFO, as data is left-justified

    //IRQ handler for this instance
    void __not_in_flash_func(_irq_handler)(void) {
        while( (pio->fstat & fstat_mask) == 0 ) { // same as !pio_sm_is_rx_fifo_empty(pio, sm) without overhead
            uint8_t c = (uint8_t)*rxfifo_shift;
            rb.push(c);
        }
    }

    bool panic(const char* s) {
        end();
        Serial.println(s);
        return false;
    }

public:
    UartRxPioIrq(uint8_t pin, uint16_t buflen = 256) {
        this->pin = pin;
        if(buflen < 1) buflen = 1;
        this->buflen = buflen;
    }

    ~UartRxPioIrq() {
        end();
    }

    bool begin(uint32_t baud) {
        //check pin
        if(pin == 0xFF) return false;

        //check baud
        if(baud > clock_get_hz(clk_sys) / 8) return panic("PANIC: UartRxPioIrq baud rate too high"); //max baud = f_cpu/8 >= 10M 
        if(baud < clock_get_hz(clk_sys) / 8 / 65536) return panic("PANIC: UartRxPioIrq baud rate too low"); //min baud = f_cpu/8/65536 <= 286 @ 150Mhz
            
        //only change baud if setup was completed    
        if(setup_done) {
            uart_rx_program_init(pio, sm, offset, pin, baud);
            return true;
        }

        //ringbuf
        rb_buf = new uint8_t[buflen + 1];
        if(!rb_buf) {
            return panic("PANIC: UartRxPioIrq out of memory");
        }
        rb.begin(rb_buf, buflen + 1);

        // This will find a free pio and state machine for our program and load it for us
        // We use pio_claim_free_sm_and_add_program_for_gpio_range (for_gpio_range variant)
        // so we will get a PIO instance suitable for addressing gpios >= 32 if needed and supported by the hardware
        //bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&uart_rx_program, &pio, &sm, &offset, pin, 1, true);
        bool success = pio_registry_claim("UartRx", &uart_tx_program, &pio, &sm, &offset, pin, 1, true);
        if(!success) {
            return panic("PANIC: UartRxPioIrq no free sm");
        }

        uart_rx_program_init(pio, sm, offset, pin, baud);

        //enable interrupt
        fstat_mask = (1u << (PIO_FSTAT_RXEMPTY_LSB + sm)); //pre-calculate fstat mask value
        rxfifo_shift = (io_rw_8*)&pio->rxf[sm] + 3; // pre-calculate address for 8-bit read from the uppermost byte of the FIFO, as data is left-justified
        const uint irqn = 0; //0 for PIOx_IRQ_0 or 1 for PIOx_IRQ_1 etc where x is the PIO number
        const int pio_irq = pio_get_irq_num(pio, irqn); //get IRQ for a PIO hardware instance
        irq_add_shared_handler(pio_irq, _UartRxPioIrq_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
        irq_set_enabled(pio_irq, true);
        irq_index = pio_irq - pio_get_irq_num(pio, 0); // Get index of the IRQ
        pio_set_irqn_source_enabled(pio, irq_index, pio_get_rx_fifo_not_empty_interrupt_source(sm), true); // Set pio to tell us when the FIFO is NOT empty

        //register instance
        int i;
        for(i = 0; i < UARTRXPIOIRQ_NUM; i++) {
            if(!_UartRxPioIrq_instances[i]) _UartRxPioIrq_instances[i] = this;
            break;
        }
        if(i >= UARTRXPIOIRQ_NUM) {
            return panic("PANIC: UartRxPioIrq too many instances");
        }

        setup_done = true;
        return true;    
    }

    void end() {
        setup_done = false; //NOTE: potential race condition when other thread is reading at this time

        //disable irq
        pio_set_irqn_source_enabled(pio, irq_index, pio_get_rx_fifo_not_empty_interrupt_source(sm), false);

        //don't disable IRQ, it might be in use by something else. Removing the instance will disable the handler.

        //remove UartRxPioIrq instance (and reorder list to keep non-nulls first)
        for(int i = 0; i < UARTRXPIOIRQ_NUM; i++) {
            if(_UartRxPioIrq_instances[i] == this) {
                //find last non-null entry or 0
                int j;
                for(j = UARTRXPIOIRQ_NUM-1; j > 0; j--) {
                    if(_UartRxPioIrq_instances[i]) break;
                }
                //replace instance with last non-null entry
                _UartRxPioIrq_instances[i] = _UartRxPioIrq_instances[j];
                //remove last non-null entry
                _UartRxPioIrq_instances[j] = nullptr;
                break;
            }
        }

        // This will free resources and unload our pio program
        if(pio) {
            pio_remove_program_and_unclaim_sm(&uart_rx_program, pio, sm, offset);
            pio = nullptr;
        }

        //delete ringbuffer buffer
        delete[] rb_buf;
    }

    //read up to len bytes into buf
    int read(uint8_t *buf, int len) {
        if(!setup_done) return 0;

        int i;
        for(i = 0; i < len; i++) {
            if( rb.pop(buf+i) == 0 ) break;
        }
        return i;
    }

    int available() {
        if(!setup_done) return 0;        
        return rb.len();
    }    
};

void __not_in_flash_func(_UartRxPioIrq_irq_handler)(void) {
    //_UartRxPioIrq_instances is ordered, exit on first null
    for(uint8_t i = 0; i < UARTRXPIOIRQ_NUM; i++) {
      if(_UartRxPioIrq_instances[i]) {
        _UartRxPioIrq_instances[i]->_irq_handler();
      }else{
        break;
      }
    }
}