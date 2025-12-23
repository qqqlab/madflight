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

#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/uart.h>
#include <string.h>
#include <stdlib.h> //aligned_alloc

#include "SerialDMA.h"


uint32_t SerialDMA::begin(uint8_t uart_num, uint32_t baudrate, int txpin, int rxpin, uint16_t txbuflen, uint16_t rxbuflen, uint8_t bits, char parity, uint8_t stop, bool invert) {
  uart_ = UART_INSTANCE(uart_num);
  gpio_set_function(txpin, GPIO_FUNC_UART);
  gpio_set_function(rxpin, GPIO_FUNC_UART);

  //invert
  gpio_set_outover(txpin, (invert ? 1 : 0));
  gpio_set_inover(rxpin, (invert ? 1 : 0));

  uint32_t actual_baud = setBaud(baudrate);

  //properties
  uart_set_format(uart_, bits, stop, (parity=='E' ? UART_PARITY_EVEN : ( parity=='O' ? UART_PARITY_ODD : UART_PARITY_NONE)) );
  uart_set_hw_flow(uart_, false, false); //no hardware flow (is default?)
  uart_set_fifo_enabled(uart_, true); //enable FIFOs (is default?)

  rx_buf_len_pow = log_2(rxbuflen);
  tx_buf_len_pow = log_2(txbuflen);
  rx_buf_len = 1 << (rx_buf_len_pow);
  tx_buf_len = 1 << (tx_buf_len_pow);
  free(rx_buf);
  free(tx_buf);  
  rx_buf = (uint8_t*)aligned_alloc(rx_buf_len, rx_buf_len);
  tx_buf = (uint8_t*)aligned_alloc(tx_buf_len, tx_buf_len);

  init_dma();

  return actual_baud;
}

//round up to next power of two: 255->8, 256->8, 257->9
uint8_t SerialDMA::log_2(uint16_t val) {
  uint8_t i = 0;
  val--;
  while(val>0) {
    i++;
    val>>=1;
  }
  return i;
}

uint32_t SerialDMA::setBaud(uint32_t baudrate) {
  return uart_init(uart_, baudrate);
}

void SerialDMA::init_dma() {
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
  channel_config_set_dreq(&rx_config, DREQ_UART0_RX);
  channel_config_set_chain_to(&rx_config, rx_ctrl_dma_ch);
  channel_config_set_enable(&rx_config, true);
  dma_channel_configure(rx_dma_ch, &rx_config, rx_buf, &uart0_hw->dr, rx_buf_len << 16, true); // note: rx_buf_len<<16 to fill buffer 65356 times before chaining to ctrl_ch (remove <<16 to test chaining)
#else
  // RP2350 has self trigger
  // DMA uart read
  if(rx_dma_ch < 0) rx_dma_ch = dma_claim_unused_channel(true);
  dma_channel_config rx_config = dma_channel_get_default_config(rx_dma_ch);
  channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_8);
  channel_config_set_read_increment(&rx_config, false);
  channel_config_set_write_increment(&rx_config, true);
  channel_config_set_ring(&rx_config, true, rx_buf_len_pow);  //true = write buffer
  channel_config_set_dreq(&rx_config, DREQ_UART0_RX);
  channel_config_set_enable(&rx_config, true);
  dma_channel_configure(rx_dma_ch, &rx_config, rx_buf, &uart0_hw->dr, dma_encode_transfer_count_with_self_trigger(rx_buf_len), true);
  dma_channel_set_irq0_enabled(rx_dma_ch, false);
#endif

  /// DMA uart write
  if(tx_dma_ch < 0) tx_dma_ch = dma_claim_unused_channel(true);  
  dma_channel_config tx_config = dma_channel_get_default_config(tx_dma_ch);
  channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
  channel_config_set_read_increment(&tx_config, true);
  channel_config_set_write_increment(&tx_config, false);
  channel_config_set_ring(&tx_config, false, tx_buf_len_pow); //false = read buffer
  channel_config_set_dreq(&tx_config, DREQ_UART0_TX);
  dma_channel_set_config(tx_dma_ch, &tx_config, false);
  dma_channel_set_write_addr(tx_dma_ch, &uart0_hw->dr, false);
}

uint16_t SerialDMA::availableForWrite() {
  return tx_buf_len - 1 - dma_channel_hw_addr(tx_dma_ch)->transfer_count;
}

uint16_t SerialDMA::write(const uint8_t* data, uint16_t length) {
  if (length == 0) {
    return 0;
  }

  uint16_t avail = availableForWrite();
  if (length > avail) {
    return 0; // not enough space to write
  }

  //copy to dma buffer
  if ((tx_buf_len - 1) < tx_user_idx + length) {
    memcpy(&tx_buf[tx_user_idx], data, tx_buf_len - tx_user_idx);
    memcpy(tx_buf, &data[tx_buf_len - tx_user_idx], length - (tx_buf_len - tx_user_idx));
  } else {
    memcpy(&tx_buf[tx_user_idx], data, length);
  }
  tx_user_idx = (tx_user_idx + length) & (tx_buf_len - 1);

  //check if busy
  dma_channel_hw_t *hw = dma_channel_hw_addr(tx_dma_ch);
  uint32_t ctrl = hw->al1_ctrl;
  bool do_abort = false;
  if(dma_channel_is_busy(tx_dma_ch)) {
    hw->al1_ctrl = 0; //EN=0 -> pause the current transfer sequence (i.e. BUSY will remain high if already high)
    do_abort = true;
  }

  //update tx_dma_size, tx_dma_index
  uint32_t bytes_remaining = dma_channel_hw_addr(tx_dma_ch)->transfer_count;
  uint32_t bytes_written = tx_dma_size - bytes_remaining;
  tx_dma_size = bytes_remaining + length;
  tx_dma_idx = (tx_dma_idx + bytes_written) & (tx_buf_len - 1);

  //abort if needed
  if(do_abort) dma_channel_abort(tx_dma_ch);

  // restart tx dma
  uint8_t* start = &tx_buf[tx_dma_idx];
  hw->read_addr = (uintptr_t) start;
  hw->transfer_count = tx_dma_size;
  hw->ctrl_trig = ctrl;

  return length;
}

uint16_t SerialDMA::available() {
  //update buffer tail
  uint16_t rx_dma_idx = (rx_buf_len - dma_channel_hw_addr(rx_dma_ch)->transfer_count) & (rx_buf_len - 1);
  
  if (rx_user_idx <= rx_dma_idx) {
    return rx_dma_idx - rx_user_idx;
  }else{
    return tx_buf_len + rx_dma_idx - rx_user_idx;
  }
}

uint16_t SerialDMA::read(uint8_t* data, uint16_t length) {
  if (length == 0) {
    return 0;
  }

  uint16_t avail = available();
  if (avail < length) {
    // read as much as we have
    length = avail;
  }

  if (rx_user_idx < rx_dma_idx) {
    memcpy(data, &rx_buf[rx_user_idx], length);
  } else {
    uint16_t left = rx_buf_len - rx_user_idx;
    if (length < left) {
      left = length; // limit to target buffer size
    }
    memcpy(data, &rx_buf[rx_user_idx], left);
    if (left < length) {
      memcpy(&data[left], rx_buf, length - left);
    }
  }
  rx_user_idx = (rx_user_idx + length) & (rx_buf_len - 1);

  return length;
}
