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

//based on: https://github.com/rossihwang/pico_dma_uart

#pragma once

class SerialDMA {
public:
  //buffer size has to be a power of two, will be rounded up to next greater value: i.e. 128->128, 129->256
  uint32_t begin(uint8_t uart_num, uint32_t baudrate, int txpin, int rxpin, uint16_t txbuflen, uint16_t rxbuflen, uint8_t bits = 8, char parity = 'N', uint8_t stop = 1, bool invert = false); //returns actual baud rate
  uint32_t setBaud(uint32_t baudrate); //call this after begin to change baud rate, returns actual baud rate
  uint16_t write(const uint8_t* data, uint16_t length);
  uint16_t read(uint8_t* data, uint16_t length);
  uint16_t available();
  uint16_t availableForWrite();

  int readBytes(char *buf, int len) {return read((uint8_t*)buf, len);}
  void begin(int baud) {setBaud(baud);}

  void setFormat() {

  }

private:
  uart_inst_t* uart_ = nullptr;

#if PICO_RP2040
  int8_t rx_ctrl_dma_ch = -1;
  uint8_t rx_ctrl_dummy_read;
  uint8_t rx_ctrl_dummy_write;
#endif
  int8_t rx_dma_ch = -1;
  uint8_t rx_buf_len_pow; // = 8; //2^8 = 256 bytes
  uint16_t rx_buf_len; // = 1 << (rx_buf_len_pow);
  uint8_t * rx_buf = nullptr; //needs to be aligned! ... static version: __attribute__((aligned(256))) uint8_t rx_buf[256];  
  uint16_t rx_user_idx = 0; // next index to read
  uint16_t rx_dma_idx = 0;  // next index dma will write

  int8_t tx_dma_ch = -1;
  uint8_t tx_buf_len_pow; // = 8; //2^8 = 256 bytes
  uint16_t tx_buf_len; // = 1 << (tx_buf_len_pow);
  uint8_t * tx_buf = nullptr; //needs to be aligned! ... static version: //__attribute__((aligned(256))) uint8_t tx_buf[256];
  uint16_t tx_user_idx = 0;  // next index to write
  uint16_t tx_dma_idx = 0;  // next index dma will read
  uint16_t tx_dma_size = 0; // size of current dma transfer

  void init_dma();
  uint8_t log_2(uint16_t val);
};
