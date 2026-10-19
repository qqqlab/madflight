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

#ifdef ARDUINO_ARCH_RP2040

#include "SerialDMA.h"

#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/structs/dma.h>
#include <hardware/structs/uart.h>
#include <cstring>
#include <cstdlib>
#include <cstdint>

SerialDMA::~SerialDMA() {
  end();
}

bool SerialDMA::roundBufferSize(
    uint16_t requested,
    uint16_t& actual,
    uint8_t& pow) {

  if (requested == 0 || requested > MAX_BUFFER_SIZE) {
    return false;
  }

  uint32_t n = 1;
  pow = 0;

  while (n < requested) {
    n <<= 1;
    ++pow;
  }

  actual = static_cast<uint16_t>(n);
  return true;
}

bool SerialDMA::lock(SemaphoreHandle_t mutex) {
  return mutex &&
      xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE;
}

uint32_t SerialDMA::begin(
    uint8_t uart_num,
    uint32_t baudrate,
    int txpin,
    int rxpin,
    uint16_t txbuflen,
    uint16_t rxbuflen,
    uint8_t bits,
    char parity,
    uint8_t stop,
    bool invert) {

  // Reject invalid configuration before touching hardware.
  if (uart_num > 1 || baudrate == 0 ||
      txpin < 0 || rxpin < 0 ||
      (bits < 5 || bits > 8) ||
      (stop != 1 && stop != 2) ||
      (parity != 'N' && parity != 'E' && parity != 'O')) {
    return 0;
  }

  // This driver is deliberately RP2350-specific.
#if PICO_RP2040
  return 0;
#endif

  uint16_t new_tx_len = 0;
  uint16_t new_rx_len = 0;
  uint8_t new_tx_pow = 0;
  uint8_t new_rx_pow = 0;

  if (!roundBufferSize(txbuflen, new_tx_len, new_tx_pow) ||
      !roundBufferSize(rxbuflen, new_rx_len, new_rx_pow)) {
    return 0;
  }

  // Allocate before destroying a working instance.
  auto* new_rx = static_cast<uint8_t*>(
      aligned_alloc(new_rx_len, new_rx_len));
  auto* new_tx = static_cast<uint8_t*>(
      aligned_alloc(new_tx_len, new_tx_len));

  if (!new_rx || !new_tx) {
    free(new_rx);
    free(new_tx);
    return 0;
  }

  memset(new_rx, 0, new_rx_len);
  memset(new_tx, 0, new_tx_len);

  // Allocate mutexes before replacing the current instance.
  SemaphoreHandle_t new_rx_mutex = xSemaphoreCreateMutex();
  SemaphoreHandle_t new_tx_mutex = xSemaphoreCreateMutex();

  if (!new_rx_mutex || !new_tx_mutex) {
    if (new_rx_mutex) vSemaphoreDelete(new_rx_mutex);
    if (new_tx_mutex) vSemaphoreDelete(new_tx_mutex);
    free(new_rx);
    free(new_tx);
    return 0;
  }

  // Do not call begin() concurrently with read/write.
  end();

  rx_buf_ = new_rx;
  tx_buf_ = new_tx;
  rx_buf_len_ = new_rx_len;
  tx_buf_len_ = new_tx_len;
  rx_buf_pow_ = new_rx_pow;
  tx_buf_pow_ = new_tx_pow;
  rx_mutex_ = new_rx_mutex;
  tx_mutex_ = new_tx_mutex;

  uart_num_ = uart_num;
  uart_ = (uart_num == 0) ? uart0 : uart1;

  // Configure UART and pins.
  gpio_set_function(txpin, GPIO_FUNC_UART);
  gpio_set_function(rxpin, GPIO_FUNC_UART);

  gpio_set_outover(txpin, invert ? GPIO_OVERRIDE_INVERT
                                  : GPIO_OVERRIDE_NORMAL);
  gpio_set_inover(rxpin, invert ? GPIO_OVERRIDE_INVERT
                                 : GPIO_OVERRIDE_NORMAL);

  uint32_t actual_baud = uart_init(uart_, baudrate);

  uart_set_format(
      uart_,
      bits,
      stop,
      parity == 'E' ? UART_PARITY_EVEN :
      parity == 'O' ? UART_PARITY_ODD :
                      UART_PARITY_NONE);

  uart_set_hw_flow(uart_, false, false);
  uart_set_fifo_enabled(uart_, true);

  // Claim channels. Return them if initialization fails.
  rx_dma_ch_ = static_cast<int8_t>(
      dma_claim_unused_channel(false));

  tx_dma_ch_ = static_cast<int8_t>(
      dma_claim_unused_channel(false));

  if (rx_dma_ch_ < 0 || tx_dma_ch_ < 0) {
    end();
    return 0;
  }

  volatile void* uart_dr =
      static_cast<volatile void*>(&uart_get_hw(uart_)->dr);

  // RX: endless, DREQ-paced DMA into an aligned circular buffer.
  dma_channel_config rc =
      dma_channel_get_default_config(rx_dma_ch_);

  channel_config_set_transfer_data_size(&rc, DMA_SIZE_8);
  channel_config_set_read_increment(&rc, false);
  channel_config_set_write_increment(&rc, true);
  channel_config_set_ring(&rc, true, rx_buf_pow_);
  channel_config_set_dreq(
      &rc, uart_num == 0 ? DREQ_UART0_RX : DREQ_UART1_RX);

  dma_channel_configure(
      rx_dma_ch_,
      &rc,
      rx_buf_,
      uart_dr,
      dma_encode_endless_transfer_count(),
      true);

  // TX: normal DMA transfers from a circular buffer.
  dma_channel_config tc =
      dma_channel_get_default_config(tx_dma_ch_);

  channel_config_set_transfer_data_size(&tc, DMA_SIZE_8);
  channel_config_set_read_increment(&tc, true);
  channel_config_set_write_increment(&tc, false);
  channel_config_set_ring(&tc, false, tx_buf_pow_);
  channel_config_set_dreq(
      &tc, uart_num == 0 ? DREQ_UART0_TX : DREQ_UART1_TX);

  dma_channel_configure(
      tx_dma_ch_,
      &tc,
      uart_dr,
      tx_buf_,
      0,
      false);

  rx_dma_pos_ = 0;
  rx_produced_ = 0;
  rx_consumed_ = 0;
  rx_overruns_ = 0;

  tx_user_idx_ = 0;
  tx_dma_idx_ = 0;
  tx_dma_size_ = 0;
  tx_dma_active_ = false;

  initialized_ = true;
  return actual_baud;
}

uint32_t SerialDMA::setBaud(uint32_t baudrate) {
  if (!initialized_ || !uart_ || baudrate == 0) {
    return 0;
  }

  // Avoid changing UART configuration mid-transfer.
  if (!lock(tx_mutex_)) return 0;
  if (!lock(rx_mutex_)) {
    xSemaphoreGive(tx_mutex_);
    return 0;
  }

  serviceTxLocked();

  // Do not reset UART with uart_init() here.
  uint32_t actual = uart_set_baudrate(uart_, baudrate);

  xSemaphoreGive(rx_mutex_);
  xSemaphoreGive(tx_mutex_);
  return actual;
}

void SerialDMA::updateRxLocked() {
  if (!initialized_ || rx_dma_ch_ < 0) return;

  uintptr_t base = reinterpret_cast<uintptr_t>(rx_buf_);
  uintptr_t addr = static_cast<uintptr_t>(
      dma_channel_hw_addr(rx_dma_ch_)->write_addr);

  // DMA write ring wraps the current write address at the
  // buffer boundary. Read the live address, not TRANS_COUNT.
  uint16_t pos = static_cast<uint16_t>(
      (addr - base) & (rx_buf_len_ - 1));

  uint16_t delta = static_cast<uint16_t>(
      (pos - rx_dma_pos_) & (rx_buf_len_ - 1));

  rx_produced_ += delta;
  rx_dma_pos_ = pos;

  uint32_t queued = rx_produced_ - rx_consumed_;

  if (queued > rx_buf_len_) {
    uint32_t lost = queued - rx_buf_len_;
    rx_overruns_ += lost;
    rx_consumed_ += lost;
  }
}

uint16_t SerialDMA::available() {
  if (!initialized_ || !lock(rx_mutex_)) return 0;

  updateRxLocked();

  uint32_t n = rx_produced_ - rx_consumed_;
  if (n > rx_buf_len_) n = rx_buf_len_;

  xSemaphoreGive(rx_mutex_);
  return static_cast<uint16_t>(n);
}

uint16_t SerialDMA::read(uint8_t* data, uint16_t length) {
  if (!initialized_ || !data || length == 0 ||
      !lock(rx_mutex_)) {
    return 0;
  }

  updateRxLocked();

  uint32_t queued = rx_produced_ - rx_consumed_;
  if (queued > rx_buf_len_) queued = rx_buf_len_;

  if (length > queued) {
    length = static_cast<uint16_t>(queued);
  }

  uint16_t idx = static_cast<uint16_t>(
      rx_consumed_ & (rx_buf_len_ - 1));

  uint16_t first = length;
  if (first > rx_buf_len_ - idx) {
    first = rx_buf_len_ - idx;
  }

  memcpy(data, &rx_buf_[idx], first);

  if (length > first) {
    memcpy(data + first, rx_buf_, length - first);
  }

  rx_consumed_ += length;

  xSemaphoreGive(rx_mutex_);
  return length;
}

void SerialDMA::serviceTxLocked() {
  if (!initialized_ || tx_dma_ch_ < 0) return;

  if (tx_dma_active_) {
    if (dma_channel_is_busy(tx_dma_ch_)) {
      return;
    }

    tx_dma_idx_ = static_cast<uint16_t>(
        (tx_dma_idx_ + tx_dma_size_) & (tx_buf_len_ - 1));

    tx_dma_size_ = 0;
    tx_dma_active_ = false;
  }

  if (tx_user_idx_ == tx_dma_idx_) return;

  uint16_t pending = static_cast<uint16_t>(
      (tx_user_idx_ + tx_buf_len_ - tx_dma_idx_)
      & (tx_buf_len_ - 1));

  if (pending == 0) return;

  tx_dma_size_ = pending;
  tx_dma_active_ = true;

  dma_channel_set_read_addr(
      tx_dma_ch_,
      &tx_buf_[tx_dma_idx_],
      false);

  dma_channel_set_trans_count(
      tx_dma_ch_,
      pending,
      true);
}

uint16_t SerialDMA::availableForWrite() {
  if (!initialized_ || !lock(tx_mutex_)) return 0;

  serviceTxLocked();

  uint16_t n = static_cast<uint16_t>(
      (tx_dma_idx_ + tx_buf_len_ - tx_user_idx_ - 1)
      & (tx_buf_len_ - 1));

  xSemaphoreGive(tx_mutex_);
  return n;
}

uint16_t SerialDMA::write(
    const uint8_t* data,
    uint16_t length) {

  if (!initialized_ || !data || length == 0 ||
      !lock(tx_mutex_)) {
    return 0;
  }

  serviceTxLocked();

  uint16_t avail = static_cast<uint16_t>(
      (tx_dma_idx_ + tx_buf_len_ - tx_user_idx_ - 1)
      & (tx_buf_len_ - 1));

  if (length > avail) {
    xSemaphoreGive(tx_mutex_);
    return 0;
  }

  uint16_t first = length;
  if (first > tx_buf_len_ - tx_user_idx_) {
    first = tx_buf_len_ - tx_user_idx_;
  }

  memcpy(&tx_buf_[tx_user_idx_], data, first);

  uint16_t second = length - first;
  if (second) {
    memcpy(tx_buf_, data + first, second);
  }

  tx_user_idx_ = static_cast<uint16_t>(
      (tx_user_idx_ + length) & (tx_buf_len_ - 1));

  serviceTxLocked();

  xSemaphoreGive(tx_mutex_);
  return length;
}

void SerialDMA::end() {
  // Must not be called concurrently with read/write/begin.
  initialized_ = false;

  if (rx_dma_ch_ >= 0) {
    dma_channel_abort(rx_dma_ch_);
    dma_channel_unclaim(rx_dma_ch_);
    rx_dma_ch_ = -1;
  }

  if (tx_dma_ch_ >= 0) {
    dma_channel_abort(tx_dma_ch_);
    dma_channel_unclaim(tx_dma_ch_);
    tx_dma_ch_ = -1;
  }

  if (uart_) {
    uart_deinit(uart_);
    uart_ = nullptr;
  }

  free(rx_buf_);
  free(tx_buf_);
  rx_buf_ = nullptr;
  tx_buf_ = nullptr;

  rx_buf_len_ = tx_buf_len_ = 0;

  if (rx_mutex_) {
    vSemaphoreDelete(rx_mutex_);
    rx_mutex_ = nullptr;
  }

  if (tx_mutex_) {
    vSemaphoreDelete(tx_mutex_);
    tx_mutex_ = nullptr;
  }
}

#endif //#ifdef ARDUINO_ARCH_RP2040
