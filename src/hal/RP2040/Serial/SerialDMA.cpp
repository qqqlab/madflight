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
#include <string.h>
#include <stdlib.h> //aligned_alloc

#include "SerialDMA.h"

uint32_t SerialDMA::begin(uint8_t uart_num, uint32_t baudrate, int txpin, int rxpin, uint16_t txbuflen, uint16_t rxbuflen, uint8_t bits, char parity, uint8_t stop, bool invert) {
  if(uart_num > 1) return 0;
  if(txpin < 0 && rxpin < 0) return 0;

  //in case begin() is called twice: stop running DMA before touching buffers
  stop_dma();
  
  //save uart instance in class
  uart_inst = UART_INSTANCE(uart_num);
  
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
  uint32_t actual_baud = uart_init(uart_inst, baudrate);

  //uart properties
  uart_set_format(uart_inst, bits, stop, (parity=='E' ? UART_PARITY_EVEN : ( parity=='O' ? UART_PARITY_ODD : UART_PARITY_NONE)) );
  uart_set_hw_flow(uart_inst, false, false); //no hardware flow (is default?)
  uart_set_fifo_enabled(uart_inst, true); //enable FIFOs (is default?)

  //buffers: min 4 bytes (ring size 0 would mean "no wrap"), max 32768 bytes (uint16_t)
  rx_buf_len_pow = log_2(rxbuflen);
  tx_buf_len_pow = log_2(txbuflen);
  if(rx_buf_len_pow < 2) rx_buf_len_pow = 2;
  if(tx_buf_len_pow < 2) tx_buf_len_pow = 2;
  if(rx_buf_len_pow > 15) rx_buf_len_pow = 15;
  if(tx_buf_len_pow > 15) tx_buf_len_pow = 15;
  rx_buf_len = 1 << (rx_buf_len_pow);
  tx_buf_len = 1 << (tx_buf_len_pow);
  free(rx_buf);
  free(tx_buf);
  rx_buf = nullptr;
  tx_buf = nullptr;
  rx_buf = (uint8_t*)aligned_alloc(rx_buf_len, rx_buf_len);
  tx_buf = (uint8_t*)aligned_alloc(tx_buf_len, tx_buf_len);

  //confirm DMA ring starting address is aligned with buffer size boundary
  if(!rx_buf || !tx_buf || ((uintptr_t)rx_buf & (rx_buf_len - 1)) || ((uintptr_t)tx_buf & (tx_buf_len - 1))) {
    free(rx_buf);
    free(tx_buf);
    rx_buf = nullptr;
    tx_buf = nullptr;
    return 0;
  }
  memset(rx_buf, 0, rx_buf_len); //not really needed

  init_dma();

  return actual_baud;
}

//round up to next power of two: 255->8, 256->8, 257->9
uint8_t SerialDMA::log_2(uint16_t val) {
  if(val == 0) return 0;
  uint8_t i = 0;
  val--;
  while(val>0) {
    i++;
    val>>=1;
  }
  return i;
}

uint32_t SerialDMA::setBaud(uint32_t baudrate) {
  if(!uart_inst) return 0;
  return uart_set_baudrate(uart_inst, baudrate); //does not reset the uart / format
}

void SerialDMA::stop_dma() {
  //abort running channels (channels stay claimed so they can be re-used)
#if PICO_RP2040
  if(rx_ctrl_dma_ch >= 0) dma_channel_abort(rx_ctrl_dma_ch); //abort ctrl first, it chains to rx
#endif
  if(rx_dma_ch >= 0) dma_channel_abort(rx_dma_ch);
  if(tx_dma_ch >= 0) dma_channel_abort(tx_dma_ch);
  rx_user_idx = 0;
  rx_dma_idx = 0;
  tx_user_idx = 0;
}

void SerialDMA::init_dma() {
  const uint rx_dreq = uart_get_dreq(uart_inst, false);
  const uint tx_dreq = uart_get_dreq(uart_inst, true);
  volatile void *uart_dr = (volatile void *)&uart_get_hw(uart_inst)->dr;

  rx_user_idx = 0;
  rx_dma_idx = 0;

#if PICO_RP2040
  // RP2040 does not have self trigger, use second dma channel to re-trigger rx channel
  // DMA uart read
  rx_dma_ch = dma_claim_unused_channel(true);
  rx_ctrl_dma_ch = dma_claim_unused_channel(true);

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
  dma_channel_configure(rx_dma_ch, &rx_config, rx_buf, uart_dr, rx_buf_len << 16, true); // note: rx_buf_len<<16 to fill buffer 65356 times before chaining to ctrl_ch (remove <<16 to test chaining)
#else
  // RP2350 has self trigger
  // DMA uart read
  rx_dma_ch = dma_claim_unused_channel(true);
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
  rx_dma_hw = dma_channel_hw_addr(rx_dma_ch);

  // DMA uart write
  tx_dma_ch = dma_claim_unused_channel(true);
  dma_channel_config tx_config = dma_channel_get_default_config(tx_dma_ch);
  channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
  channel_config_set_read_increment(&tx_config, true);
  channel_config_set_write_increment(&tx_config, false);
  channel_config_set_ring(&tx_config, false, tx_buf_len_pow); //false = read buffer
  channel_config_set_dreq(&tx_config, tx_dreq);
  dma_channel_set_config(tx_dma_ch, &tx_config, false);
  dma_channel_set_write_addr(tx_dma_ch, uart_dr, false);

  tx_dma_hw = dma_channel_hw_addr(tx_dma_ch);
  tx_dma_hw->read_addr = (uintptr_t) tx_buf;
  tx_user_idx = 0;
}

uint16_t SerialDMA::availableForWrite() {
  if (!tx_dma_hw) return 0;

  //read_addr is last byte read by dma. It can only increase while we look, so the result is conservative.
  return (tx_dma_hw->read_addr - tx_user_idx - 1) & (tx_buf_len - 1);
}

uint16_t SerialDMA::write(const uint8_t* data, uint16_t length) {
  if (length == 0 || !tx_dma_hw) {
    return 0;
  }

  uint16_t avail = availableForWrite();
  if (length > avail) {
    return 0; // not enough space to write, nothing written
  }

  //copy to dma buffer. This only touches the free area, which the dma is not reading.
  uint16_t first = length;
  if(first > tx_buf_len - tx_user_idx) first = tx_buf_len - tx_user_idx;
  memcpy(&tx_buf[tx_user_idx], data, first);
  if(length > first) memcpy(tx_buf, data + first, length - first);
  tx_user_idx = (tx_user_idx + length) & (tx_buf_len - 1);

  //save ctrl register
  uint32_t ctrl = tx_dma_hw->al1_ctrl;

  //abort
  tx_dma_hw->al1_ctrl = 0; //EN=0 -> pause the current transfer sequence (i.e. BUSY will remain high if already high)
  dma_channel_abort(tx_dma_ch); //complete transfers in flight, set transfer_count = 0

  //set new transfer_count
  tx_dma_hw->transfer_count = (tx_user_idx - tx_dma_hw->read_addr) & (tx_buf_len - 1);

  //(re)start
  tx_dma_hw->ctrl_trig = ctrl;

  return length;
}



uint16_t SerialDMA::available() {
  if (!rx_dma_hw) return 0;

  //update buffer tail (on RP2350 the upper bits of transfer_count hold the trigger mode, the mask removes them)
  uint16_t rx_dma_idx = (rx_buf_len - rx_dma_hw->transfer_count) & (rx_buf_len - 1);
  
  if (rx_user_idx <= rx_dma_idx) {
    return rx_dma_idx - rx_user_idx;
  }else{
    return rx_buf_len + rx_dma_idx - rx_user_idx;
  }
}

uint16_t SerialDMA::read(uint8_t* data, uint16_t length) {
  if (length == 0 || !rx_dma_hw) return 0;

  uint16_t avail = available();
  if (avail < length) {
    // read as much as we have
    length = avail;
  }

  uint16_t left = rx_buf_len - rx_user_idx; //bytes until end of ring
  if(left > length) left = length;
  memcpy(data, &rx_buf[rx_user_idx], left);
  if(left < length) memcpy(&data[left], rx_buf, length - left);

  rx_user_idx = (rx_user_idx + length) & (rx_buf_len - 1);

  return length;
}

#endif //#ifdef ARDUINO_ARCH_RP2040
