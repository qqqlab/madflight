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

#include <Arduino.h>
#include <hardware/uart.h>
#include <FreeRTOS.h>
#include <semphr.h>

class SerialDMA {
public:
  SerialDMA() = default;
  ~SerialDMA();

  SerialDMA(const SerialDMA&) = delete;
  SerialDMA& operator=(const SerialDMA&) = delete;

  // Buffer sizes are rounded up to a power of two.
  // Returns actual baud rate, or 0 on failure.
  uint32_t begin(
      uint8_t uart_num,
      uint32_t baudrate,
      int txpin,
      int rxpin,
      uint16_t txbuflen = 256,
      uint16_t rxbuflen = 512,
      uint8_t bits = 8,
      char parity = 'N',
      uint8_t stop = 1,
      bool invert = false);

  uint32_t setBaud(uint32_t baudrate);

  // Task-context APIs only; do not call from an ISR.
  uint16_t write(const uint8_t* data, uint16_t length);
  uint16_t read(uint8_t* data, uint16_t length);

  uint16_t available();
  uint16_t availableForWrite();

  int readBytes(char* buf, int len) {
    if (len <= 0) return 0;
    return read(reinterpret_cast<uint8_t*>(buf),
                static_cast<uint16_t>(
                    len > 65535 ? 65535 : len));
  }

  void begin(int baud) {
    setBaud(static_cast<uint32_t>(baud));
  }

  // Call explicitly before reinitializing if desired.
  void end();

  // Number of RX bytes discarded after buffer overrun.
  uint32_t rxOverruns() const {
    return rx_overruns_;
  }

private:
  static constexpr uint32_t MAX_BUFFER_SIZE = 4096;

  uart_inst_t* uart_ = nullptr;
  uint8_t uart_num_ = 0;

  int8_t rx_dma_ch_ = -1;
  int8_t tx_dma_ch_ = -1;

  uint8_t* rx_buf_ = nullptr;
  uint8_t* tx_buf_ = nullptr;

  uint16_t rx_buf_len_ = 0;
  uint16_t tx_buf_len_ = 0;
  uint8_t rx_buf_pow_ = 0;
  uint8_t tx_buf_pow_ = 0;

  // RX positions and accounting. Only RX API callers modify
  // these fields, protected by rx_mutex_.
  uint16_t rx_dma_pos_ = 0;
  uint32_t rx_produced_ = 0;
  uint32_t rx_consumed_ = 0;
  uint32_t rx_overruns_ = 0;

  // TX ring. Only TX API callers modify these fields,
  // protected by tx_mutex_.
  uint16_t tx_user_idx_ = 0;
  uint16_t tx_dma_idx_ = 0;
  uint16_t tx_dma_size_ = 0;
  bool tx_dma_active_ = false;

  SemaphoreHandle_t rx_mutex_ = nullptr;
  SemaphoreHandle_t tx_mutex_ = nullptr;

  bool initialized_ = false;

  static bool roundBufferSize(
      uint16_t requested,
      uint16_t& actual,
      uint8_t& pow);

  void serviceTxLocked();
  void updateRxLocked();
  bool lock(SemaphoreHandle_t mutex);
};
