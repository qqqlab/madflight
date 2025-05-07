/*==========================================================================================
SerialRingBuf.h - Ring Buffer for Serial I/O

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

// Ring buffer class
class SerialRingBuf {

private:
public:
  uint8_t *buf;
  uint16_t bufsize = 0; //buffer can hold bufsize-1 bytes
  volatile uint16_t head = 0; //previous push position - NOTE: volatile to prevent compiler to optimize things away
  volatile uint16_t tail = 0; //next pop position - NOTE: volatile to prevent compiler to optimize things away

public:
  void begin(uint8_t *buf, uint16_t bufsize) {
    this->buf = buf;
    this->bufsize = bufsize;
    clear();
  }

  //clear buffer
  void clear() {
    tail = head;
  }

  //number of bytes in buffer
  uint16_t len() {
    return (head >= tail ? head - tail : bufsize + head - tail); 
  }
  
  //number of bytes free in buffer
  uint16_t free_space() {
    return bufsize - 1 - len();
  }

  //push 1 byte in to the buffer - used in IRQ so store in RAM
  uint16_t __not_in_flash_func(push)(uint8_t c) {
    uint16_t next = inc(head);
    if(next == tail) return 0; //buffer full
    buf[head] = c;
    head = next;
    return 1;
  }

  //push up to data_len bytes
  uint16_t push(const uint8_t *data, size_t data_len)
  {
    //exit if no data
    if (data_len == 0 || data == nullptr) return 0;

    //get number of bytes available in ringbuf
    volatile uint16_t avail = free_space(); //volatile to prevent race conditions, available space could increase while this thread is processing
    if(data_len > avail) data_len = avail;

    if(data_len == 0) return 0;
    
    //get number of bytes before buf wraps
    uint16_t remaining = bufsize - head; //no need for volatile here, this thread is the only thread writing to head - NOTE: remaining can be larger than data_len, but this is normal
    
    if (data_len <= remaining) {
      memcpy(buf + head, data, data_len);
      if(data_len == remaining) {
        //Serial.printf("nowrap1 data_len=%d head=%d tail=%d remaining=%d\n", data_len, head, tail, remaining);
        head = 0;
      }else{
        //Serial.printf("nowrap2 data_len=%d head=%d tail=%d remaining=%d\n", data_len, head, tail, remaining);
        head += data_len;
      }
    }else{
      //Serial.printf("wrap    data_len=%d head=%d tail=%d remaining=%d\n", data_len, head, tail, remaining);
      memcpy(buf + head, data, remaining);
      memcpy(buf, data + remaining, data_len - remaining);
      head = data_len - remaining;
    } 
    return data_len;
  }

  //pop 1 byte from the buffer - used in IRQ so store in RAM
  uint16_t __not_in_flash_func(pop)(uint8_t *c) {
    if(head == tail) return 0; //buffer empty
    *c = buf[tail];
    tail = inc(tail);
    return 1;
  }

  //pop up to data_len bytes
  uint16_t pop(uint8_t *data, size_t data_len)
  {
    //exit if no data
    if (data_len == 0 || data == nullptr) return 0;

    //get number of bytes in ringbuf
    volatile uint16_t avail = len(); //volatile to prevent race conditions, number of bytes in ringbuf could increase while this thread is processing
    if(data_len > avail) data_len = avail;

    if(data_len == 0) return 0;
    
    //get number of bytes before buf wraps
    uint16_t remaining = bufsize - tail; //no need for volatile here, this thread is the only thread writing to tail - NOTE: remaining can be larger than data_len, but this is normal
    
    if (data_len <= remaining) {
      memcpy(data, buf + tail, data_len);
      if(data_len == remaining) {
        //Serial.printf("nowrap1 data_len=%d head=%d tail=%d remaining=%d\n", data_len, head, tail, remaining);
        tail = 0;
      }else{
        //Serial.printf("nowrap2 data_len=%d head=%d tail=%d remaining=%d\n", data_len, head, tail, remaining);
        tail += data_len;
      }
    }else{
      //Serial.printf("wrap    data_len=%d head=%d tail=%d remaining=%d\n", data_len, head, tail, remaining);
      memcpy(data, buf + tail, remaining);
      memcpy(data + remaining, buf, data_len - remaining);
      tail = data_len - remaining;
    } 
    return data_len;
  }

private:
  //increase a buffer position - used in IRQ so store in RAM
  uint16_t __not_in_flash_func(inc)( uint16_t v) {
    return (v < bufsize - 1 ? v + 1 : 0 ); 
  }
};
