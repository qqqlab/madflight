/*==========================================================================================
SerialPioIRQ.h - High Performance RP2 Buffered RX/TX PIO UART Driver

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

#include "UartTxPioIrq.h"
#include "UartRxPioIrq.h"

// Serial PIO IRQ class
class SerialPioIRQ {
private:
  UartTxPioIrq tx;
  UartRxPioIrq rx; 
public:
  SerialPioIRQ(uint8_t txpin, uint8_t rxpin, uint16_t txbuflen = 256, uint16_t rxbuflen = 256) : tx(txpin, txbuflen), rx(rxpin, rxbuflen) {}

  void begin(int baud) {
    tx.begin(baud);
    rx.begin(baud);
  }
  
  int available() {
    return rx.available();
  }

  int availableForWrite() {
    return tx.availableForWrite();
  }

  int readBytes(uint8_t *buf, int len) {
    return rx.read(buf, len);
  }

  int write(uint8_t *buf, int len) {
    return tx.write(buf, len);
  }
};



