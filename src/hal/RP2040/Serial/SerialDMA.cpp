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

#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/uart.h>
#include <hardware/sync.h>
#include <string.h>
#include <stdlib.h> //aligned_alloc

#include "SerialDMA.h"

// DMA completion interrupt used for TX. Installed as a SHARED handler. If another driver owns this
// irq exclusively, the driver still works but queued bytes are only pushed out by write()/availableForWrite()/service().
#define SERIALDMA_IRQ_INDEX 1
#define SERIALDMA_IRQ       DMA_IRQ_1

static SerialDMA* s_instances[2] = {nullptr, nullptr};
static bool s_irq_registered = false;
static bool s_irq_ok = false;

void SerialDMA::dmaIrqHandler() {
  for(int i = 0; i < 2; i++) {
    SerialDMA* s = s_instances[i];
    if(s) s->irqHandler();
  }
}

void SerialDMA::irqHandler() {
  if(tx_dma_ch < 0 || !tx_irq_ok) return;
  if(dma_irqn_get_channel_status(SERIALDMA_IRQ_INDEX, tx_dma_ch)) {
    dma_irqn_acknowledge_channel(SERIALDMA_IRQ_INDEX, tx_dma_ch); //acknowledge first: a completion after this re-raises the flag
    uint32_t save = spin_lock_blocking(tx_lock);
    tx_kick();
    spin_unlock(tx_lock, save);
  }
}

uint32_t SerialDMA::begin(uint8_t uart_num, uint32_t baudrate, int txpin, int rxpin, uint16_t txbuflen, uint16_t rxbuflen, uint8_t bits, char parity, uint8_t stop, bool invert) {
  if(uart_num > 1) return 0;
  if(txpin < 0 && rxpin < 0) return 0;

  //in case begin() is called twice: stop running DMA before touching buffers
  stop_dma();

  uart_num_ = uart_num;
  uart_ = UART_INSTANCE(uart_num);

  //only configure pins that are used
  if(txpin >= 0) {
    gpio_set_function(txpin, GPIO_FUNC_UART);
    gpio_set_outover(txpin, (invert ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL));
  }
  if(rxpin >= 0) {
    gpio_set_function(rxpin, GPIO_FUNC_UART);
    gpio_set_inover(rxpin, (invert ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL));
  }

  //uart_init resets the peripheral and the format, so only call it here. setBaud() uses uart_set_baudrate().
  uint32_t actual_baud = uart_init(uart_, baudrate);

  //properties
  uart_set_format(uart_, bits, stop, (parity=='E' ? UART_PARITY_EVEN : ( parity=='O' ? UART_PARITY_ODD : UART_PARITY_NONE)) );
  uart_set_hw_flow(uart_, false, false); //no hardware flow (is default?)
  uart_set_fifo_enabled(uart_, true); //enable FIFOs (is default?)

  //buffers: min 4 bytes (ring size 0 would mean "no wrap"), max 32768 bytes (uint16_t)
  rx_buf_len_pow = log_2(rxbuflen);
  tx_buf_len_pow = log_2(txbuflen);
  if(rx_buf_len_pow < 2) rx_buf_len_pow = 2;
  if(tx_buf_len_pow < 2) tx_buf_len_pow = 2;
  if(rx_buf_len_pow > 15) rx_buf_len_pow = 15;
  if(tx_buf_len_pow > 15) tx_buf_len_pow = 15;
  rx_buf_len = (uint16_t)1 << rx_buf_len_pow;
  tx_buf_len = (uint16_t)1 << tx_buf_len_pow;

  free(rx_buf); rx_buf = nullptr;
  free(tx_buf); tx_buf = nullptr;
  rx_buf = (uint8_t*)aligned_alloc(rx_buf_len, rx_buf_len);
  tx_buf = (uint8_t*)aligned_alloc(tx_buf_len, tx_buf_len);

  //the DMA ring wrap needs buffers aligned to their size, otherwise the DMA wraps at the wrong address
  if(!rx_buf || !tx_buf || ((uintptr_t)rx_buf & (rx_buf_len - 1)) || ((uintptr_t)tx_buf & (tx_buf_len - 1))) {
    free(rx_buf); rx_buf = nullptr;
    free(tx_buf); tx_buf = nullptr;
    return 0;
  }
  memset(rx_buf, 0, rx_buf_len);

  if(!tx_lock) {
    int n = spin_lock_claim_unused(true);
    tx_lock = spin_lock_init(n);
  }

  init_dma();

  return actual_baud;
}

//round up to next power of two: 255->8, 256->8, 257->9
uint8_t SerialDMA::log_2(uint16_t val) {
  uint8_t i = 0;
  if(val == 0) return 0;
  val--;
  while(val>0) {
    i++;
    val>>=1;
  }
  return i;
}

uint32_t SerialDMA::setBaud(uint32_t baudrate) {
  if(!uart_) return 0;
  return uart_set_baudrate(uart_, baudrate); //does not reset the uart / format
}

void SerialDMA::stop_dma() {
  //abort running channels (channels stay claimed so they can be re-used). Only done at (re)initialisation.
  if(tx_dma_ch >= 0 && tx_irq_ok) {
    dma_irqn_set_channel_enabled(SERIALDMA_IRQ_INDEX, tx_dma_ch, false);
  }
  tx_irq_ok = false;
#if PICO_RP2040
  if(rx_ctrl_dma_ch >= 0) dma_channel_abort(rx_ctrl_dma_ch); //abort ctrl first, it chains to rx
#endif
  if(rx_dma_ch >= 0) dma_channel_abort(rx_dma_ch);
  if(tx_dma_ch >= 0) dma_channel_abort(tx_dma_ch);
  rx_user_idx = rx_dma_idx = 0;
  tx_user_idx = tx_dma_idx = tx_dma_size = 0;
  tx_active = false;
}

void SerialDMA::init_dma() {
  const uint rx_dreq = uart_get_dreq(uart_, false);
  const uint tx_dreq = uart_get_dreq(uart_, true);
  volatile void *uart_dr = (volatile void *)&uart_get_hw(uart_)->dr;

  rx_user_idx = 0;
  rx_dma_idx = 0;

#if PICO_RP2040
  // RP2040 does not have self trigger, use second dma channel to re-trigger rx channel
  // DMA uart read
  if(rx_dma_ch < 0) rx_dma_ch = dma_claim_unused_channel(true);
  if(rx_ctrl_dma_ch < 0) rx_ctrl_dma_ch = dma_claim_unused_channel(true);

  // DMA control to re-trigger uart read channel (performs dummy 1 byte transfer from rx_ctrl_dummy_read to rx_ctrl_dummy_write)
  dma_channel_config ctrl_config = dma_channel_get_default_config(rx_ctrl_dma_ch);
  channel_config_set_transfer_data_size(&ctrl_config, DMA_SIZE_8);
  channel_config_set_read_increment(&ctrl_config, false);
  channel_config_set_write_increment(&ctrl_config, false);
  channel_config_set_chain_to(&ctrl_config, rx_dma_ch);
  channel_config_set_enable(&ctrl_config, true);
  dma_channel_configure(rx_ctrl_dma_ch, &ctrl_config, &rx_ctrl_dummy_write, &rx_ctrl_dummy_read, 1, false);

  // DMA uart read
  dma_channel_config rx_config = dma_channel_get_default_config(rx_dma_ch);
  channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_8);
  channel_config_set_read_increment(&rx_config, false);
  channel_config_set_write_increment(&rx_config, true);
  channel_config_set_ring(&rx_config, true, rx_buf_len_pow);  //true = write buffer
  channel_config_set_dreq(&rx_config, rx_dreq);
  channel_config_set_chain_to(&rx_config, rx_ctrl_dma_ch);
  channel_config_set_enable(&rx_config, true);
  // note: rx_buf_len<<16 to fill buffer 65536 times before chaining to ctrl_ch (remove <<16 to test chaining)
  dma_channel_configure(rx_dma_ch, &rx_config, rx_buf, uart_dr, ((uint32_t)rx_buf_len) << 16, true);
#else
  // RP2350 has self trigger
  // DMA uart read
  if(rx_dma_ch < 0) rx_dma_ch = dma_claim_unused_channel(true);
  dma_channel_config rx_config = dma_channel_get_default_config(rx_dma_ch);
  channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_8);
  channel_config_set_read_increment(&rx_config, false);
  channel_config_set_write_increment(&rx_config, true);
  channel_config_set_ring(&rx_config, true, rx_buf_len_pow);  //true = write buffer
  channel_config_set_dreq(&rx_config, rx_dreq);
  channel_config_set_enable(&rx_config, true);
  dma_channel_configure(rx_dma_ch, &rx_config, rx_buf, uart_dr, dma_encode_transfer_count_with_self_trigger(rx_buf_len), true);
  dma_channel_set_irq0_enabled(rx_dma_ch, false);
#endif

  /// DMA uart write (started on demand by tx_kick)
  if(tx_dma_ch < 0) tx_dma_ch = dma_claim_unused_channel(true);
  dma_channel_config tx_config = dma_channel_get_default_config(tx_dma_ch);
  channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
  channel_config_set_read_increment(&tx_config, true);
  channel_config_set_write_increment(&tx_config, false);
  channel_config_set_ring(&tx_config, false, tx_buf_len_pow); //false = read buffer
  channel_config_set_dreq(&tx_config, tx_dreq);
  dma_channel_configure(tx_dma_ch, &tx_config, uart_dr, tx_buf, 0, false);

  tx_user_idx = 0;
  tx_dma_idx = 0;
  tx_dma_size = 0;
  tx_active = false;

  //TX completion interrupt (shared handler; falls back to polling if the irq is owned exclusively by someone else)
  s_instances[uart_num_] = this;
  if(!s_irq_registered) {
    s_irq_registered = true;
    if(irq_get_exclusive_handler(SERIALDMA_IRQ) == nullptr) {
      irq_add_shared_handler(SERIALDMA_IRQ, SerialDMA::dmaIrqHandler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
      irq_set_enabled(SERIALDMA_IRQ, true);
      s_irq_ok = true;
    }
  }
  if(s_irq_ok) {
    dma_irqn_acknowledge_channel(SERIALDMA_IRQ_INDEX, tx_dma_ch);
    dma_irqn_set_channel_enabled(SERIALDMA_IRQ_INDEX, tx_dma_ch, true);
    tx_irq_ok = true;
  }
}

// Free space in the tx ring. Call with tx_lock held.
// Bytes of the running transfer that were already sent are free again (transfer_count counts down, so reading it
// while the dma runs can only under-estimate the free space).
uint16_t SerialDMA::tx_free_locked() {
  uint32_t consumed = 0;
  if(tx_active) {
    uint32_t remaining = dma_channel_hw_addr(tx_dma_ch)->transfer_count;
    if(remaining <= tx_dma_size) consumed = tx_dma_size - remaining;
  }
  return (tx_dma_idx + consumed + tx_buf_len - tx_user_idx - 1) & (tx_buf_len - 1);
}

// Start a transfer for all queued bytes if the dma is idle. NEVER touches a running transfer.
// Call with tx_lock held (from task or from the dma interrupt).
void SerialDMA::tx_kick() {
  if(tx_active) {
    if(dma_channel_is_busy(tx_dma_ch)) return;

    // previous transfer completed
    tx_dma_idx = (tx_dma_idx + tx_dma_size) & (tx_buf_len - 1);
    tx_dma_size = 0;
    tx_active = false;
  }

  uint16_t pending = (tx_user_idx + tx_buf_len - tx_dma_idx) & (tx_buf_len - 1);
  if(pending == 0) return;

  __dmb(); //buffer writes must be visible before the dma starts reading

  tx_dma_size = pending;
  tx_active = true;
  dma_channel_set_read_addr(tx_dma_ch, &tx_buf[tx_dma_idx], false);
  dma_channel_set_trans_count(tx_dma_ch, pending, true); //trigger
}

void SerialDMA::service() {
  if(tx_dma_ch < 0 || !tx_buf) return;
  uint32_t save = spin_lock_blocking(tx_lock);
  tx_kick();
  spin_unlock(tx_lock, save);
}

uint16_t SerialDMA::availableForWrite() {
  if(tx_dma_ch < 0 || !tx_buf) return 0;
  uint32_t save = spin_lock_blocking(tx_lock);
  if(!tx_irq_ok) tx_kick(); //polling fallback
  uint16_t avail = tx_free_locked();
  spin_unlock(tx_lock, save);
  return avail;
}

uint16_t SerialDMA::write(const uint8_t* data, uint16_t length) {
  if(tx_dma_ch < 0 || !tx_buf || !data || length == 0) return 0;

  uint32_t save = spin_lock_blocking(tx_lock);
  uint16_t avail = tx_free_locked();
  spin_unlock(tx_lock, save);
  if(length > avail) return 0; // not enough space, nothing written

  //copy into the free area. The dma/interrupt never reads there: it only reads up to tx_user_idx, which is published below.
  uint16_t first = length;
  if(first > tx_buf_len - tx_user_idx) first = tx_buf_len - tx_user_idx;
  memcpy(&tx_buf[tx_user_idx], data, first);
  if(length > first) memcpy(tx_buf, data + first, length - first);

  //publish the bytes and start the dma if it is idle (otherwise the completion interrupt starts the next transfer)
  save = spin_lock_blocking(tx_lock);
  tx_user_idx = (tx_user_idx + length) & (tx_buf_len - 1);
  tx_kick();
  spin_unlock(tx_lock, save);

  return length;
}

uint16_t SerialDMA::available() {
  if(rx_dma_ch < 0 || !rx_buf) return 0;

  //update buffer tail (on RP2350 the upper bits of transfer_count hold the trigger mode, the mask removes them)
  rx_dma_idx = (rx_buf_len - dma_channel_hw_addr(rx_dma_ch)->transfer_count) & (rx_buf_len - 1);

  if(rx_user_idx <= rx_dma_idx) return rx_dma_idx - rx_user_idx;
  return rx_buf_len + rx_dma_idx - rx_user_idx;
}

uint16_t SerialDMA::read(uint8_t* data, uint16_t length) {
  if(!data || length == 0) return 0;

  uint16_t avail = available(); //also refreshes rx_dma_idx
  if(avail < length) length = avail; // read as much as we have
  if(length == 0) return 0;

  uint16_t left = rx_buf_len - rx_user_idx; //bytes until end of ring
  if(left > length) left = length;
  memcpy(data, &rx_buf[rx_user_idx], left);
  if(left < length) memcpy(&data[left], rx_buf, length - left);

  rx_user_idx = (rx_user_idx + length) & (rx_buf_len - 1);

  return length;
}

#endif //#ifdef ARDUINO_ARCH_RP2040
