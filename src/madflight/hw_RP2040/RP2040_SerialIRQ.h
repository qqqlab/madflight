/*==========================================================================================
RP2040_SerialIRQ.h - High Performance RP2040 Buffered RX/TX UART Driver

MIT License

Copyright (c) 2024 https://github.com/qqqlab

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

// RP2040 UART API:
// https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_uart/include/hardware/uart.h
// https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_uart/uart.c


// Ring buffer class for SerialIRQ
class SerialRingBuf {

private:
public:
  uint8_t *buf;
  uint32_t bufsize = 0; //buffer can hold bufsize-1 bytes
  uint32_t head = 0; //previous push position
  uint32_t tail = 0; //next pop position

public:
  void begin(uint8_t *buf, uint32_t bufsize) {
    this->buf = buf;
    this->bufsize = bufsize;
    clear();
  }

  //clear buffer
  void clear() {
    tail = head;
  }

  //number of bytes in buffer
  uint32_t len() {
    return (head >= tail ? head - tail : bufsize + head - tail); 
  }

  //increase a buffer position - used in IRQ so store in RAM
  uint32_t __not_in_flash_func(inc)( uint32_t v) {
    return (v < bufsize - 1 ? v + 1 : 0 ); 
  }

  //push a byte in to the buffer - used in IRQ so store in RAM
  size_t __not_in_flash_func(push)(uint8_t c) {
    uint32_t next = inc(head);
    if(next == tail) return 0; //buffer full
    buf[next] = c;
    head = next;
    return 1;
  }

  //pop a byte from the buffer - used in IRQ so store in RAM
  size_t __not_in_flash_func(pop)(uint8_t *c) {
    if(head == tail) return 0; //buffer empty
    *c = buf[tail];
    tail = inc(tail);
    return 1;
  }

/*
  //optimized push
  void push(uint8_t *data, size_t len) {
    if(head + len < bufsize && (head >= tail || head + len < tail )) {
      memcpy(buf+head+1, data, len);
      head+=len;
      return len;
    }else{
      //TODO - optimize using two memcpy() calls
      size_t push_len = 0;
      for(int i=0;i<len;i++) 
        push_len += push(data[i]);
      return push_len;
    }
  }
*/

};

//global buffers for two uarts
SerialRingBuf _uart_rxbuf[2];
SerialRingBuf _uart_txbuf[2];

// UART0/UART1 interrupt handler
static void __not_in_flash_func(_uart_IRQ_handler)(int uart_idx) {
  uart_hw_t *uart_hw =  uart_get_hw((uart_idx == 0 ? uart0 : uart1));
  SerialRingBuf *rbuf = &_uart_rxbuf[uart_idx];
  SerialRingBuf *tbuf = &_uart_txbuf[uart_idx];

  uart_hw->icr = UART_UARTICR_RTIC_BITS | UART_UARTICR_RXIC_BITS | UART_UARTICR_TXIC_BITS; //clear the interrupt flags

  //RECEIVE
  while(!(uart_hw->fr & UART_UARTFR_RXFE_BITS)) { //while rxfifo not empty
    rbuf->push( uart_hw->dr ); //receive, this clears RX interrupt flag
  }

  //TRANSMIT - fill txfifo from tbuf until txfifo full or tbuf empty
  while(!(uart_hw->fr & UART_UARTFR_TXFF_BITS)) { //while txfifo not full
    uint8_t c;
    if(tbuf->pop(&c)) {
      uart_hw->dr = c;
    } else {
      break; //tbuf empty
    }
  }
}

// UART0 interrupt handler
static void __not_in_flash_func(_uart0_IRQ_handler)() {
  _uart_IRQ_handler(0);
}

// UART1 interrupt handler
static void __not_in_flash_func(_uart1_IRQ_handler)() {
  _uart_IRQ_handler(1);
}


// Arduino replacement serial class
class SerialIRQ {
private:
  SerialRingBuf *tbuf;
  SerialRingBuf *rbuf;
  int tpin;
  int rpin;
  uint8_t uart_idx;
  uart_inst_t *uart_inst;
  uart_hw_t *uart_hw;
  uint32_t uart_irq_id;

public:
  uint32_t baud_actual;

  SerialIRQ(uart_inst_t *uart, int txpin, uint8_t *txbuffer, uint32_t txbufsize, int rxpin, uint8_t *rxbuffer, uint32_t rxbufsize) {
    if(uart == uart0) {
      uart_idx = 0;
      uart_inst = uart0;
      uart_irq_id = UART0_IRQ;
      uart_hw = uart0_hw;
    }else {
      uart_idx = 1;
      uart_inst = uart1;
      uart_irq_id = UART1_IRQ;
      uart_hw = uart1_hw; 
    }
    tpin = txpin;
    rpin = rxpin;
    tbuf = &(_uart_txbuf[uart_idx]);
    rbuf = &(_uart_rxbuf[uart_idx]);
    tbuf->begin(txbuffer, txbufsize);
    rbuf->begin(rxbuffer, rxbufsize);
  }

  void begin(uint32_t baud) {
    //deinit
    irq_set_enabled(uart_irq_id, false); //disable interrupt
    uart_set_irq_enables(uart_inst, false, false); // disable the UART interrupts
    uart_deinit(uart_inst);

    //set pins
    gpio_set_function(rpin, GPIO_FUNC_UART);
    gpio_set_function(tpin, GPIO_FUNC_UART);

    //properties
    baud_actual = uart_init(uart_inst, baud); //The call will return the actual baud rate selected
    uart_set_format(uart_inst, 8, 1, UART_PARITY_NONE); //data format
    uart_set_hw_flow(uart_inst, false, false); //no hardware flow
    uart_set_fifo_enabled(uart_inst, true); //enable FIFOs

    //interrupts
    //irq_set_priority(uart_irq_id, 0);
    if(uart_idx==0) {
      irq_set_exclusive_handler(uart_irq_id, _uart0_IRQ_handler); //interrupt handler
    }else{
      irq_set_exclusive_handler(uart_irq_id, _uart1_IRQ_handler); //interrupt handler
    }
    irq_set_enabled(uart_irq_id, true); //enable interrupt
    uart_set_irq_enables(uart_inst, true, false); // enable the UART to send interrupts - RX only
    //uart_set_irq_enables(UART_ID, true, true); //DO NOT USE, this also sets uart_hw->ifls  // enable the UART tx & rx & rt interrupts -
    uart_hw->imsc = UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS | UART_UARTIMSC_TXIM_BITS; //enable UART tx & rx interrupts

    //FIFO sizes: 0b000=4, 0b001=8, 0b010=16, 0b011=24, 0b100=28 bytes
    uart_hw->ifls = 0b000000; // rxfifo[5:3], txfifo[2:0] - set rx to 4 bytes (keep rx responsive), tx to 4 bytes
  }

  uint32_t available() { 
    return rbuf->len();
  }

  int read() {
    uint8_t c;
    if(rbuf->pop(&c)) return c;
    return -1;
  }

  size_t write(uint8_t c) {
    return write(&c, 1);
  }

  size_t write(uint8_t *buf, size_t len) {
    //push into tx ring buffer
    size_t push_len = 0;
    for(size_t i=0;i<len;i++) {
      if(tbuf->push(buf[i])) push_len++; else break;
    }

    //trigger interrupt
    irq_set_pending(uart_irq_id);

    return push_len;
  }

};