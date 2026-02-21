/*==========================================================================================
MIT License

Copyright (c) 2026 https://madflight.com

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
===========================================================================================*/

//multi publisher, multi subscriber - fifo - lockfree

#pragma once

#define MF_PUBSUB_DEPTH_OFFSET 1 //never pull the deepest n messages - improves multicore performance

#include <atomic>
#include <cassert>
#include <cstring> //memcpy

template <typename T>
class MsgTopicMPMS : public MsgTopicBase {
private:
  struct cell_t {
    std::atomic<uint32_t>   seq;
    T                       msg;
  };

  cell_t* const             buf;
  uint32_t const            buf_mask;
  std::atomic<uint32_t>     wpos;

  static inline uint32_t nextPowerOfTwo(uint32_t buffer_size) {
    uint32_t result = buffer_size - 1;
    for(uint32_t i = 1; i <= sizeof(uint32_t) * 4; i <<= 1) {
      result |= result >> i;
    }
    return result + 1;
  }

public:
  MsgTopicMPMS(uint32_t buffer_size)
    : buf(new cell_t [nextPowerOfTwo(buffer_size)])
    , buf_mask(nextPowerOfTwo(buffer_size) - 1)
  {
    buffer_size = buf_mask + 1;
    assert((buffer_size >= 2) && ((buffer_size & (buffer_size - 1)) == 0));
    //set seq to -4, -3, -2, 1 for size==4
    for (uint32_t i = 0; i != buffer_size; i += 1) {
      buf[i].seq.store(i - buffer_size, std::memory_order_relaxed); 
    }
    wpos.store(0, std::memory_order_relaxed);
  }

  ~MsgTopicMPMS() {
    delete [] buf;
  }

  uint32_t capacity() const {
    return buf_mask + 1;
  }

  uint32_t get_generation() override {
    return wpos.load(std::memory_order_relaxed);
  }

  //publish a message, returns the published message generation
  uint32_t publish(void* msg) override {
    for (;;) {
      uint32_t pos = wpos.load(std::memory_order_relaxed);
      cell_t* cell = &buf[pos & buf_mask];
      uint32_t seq = cell->seq.load(std::memory_order_acquire);
      if (seq + buf_mask + 1 == pos) { //seq is size behind pos (buf_mask + 1 = size)
        if (wpos.compare_exchange_weak(pos, pos + 1, std::memory_order_relaxed)) {
          cell->seq.store(pos + 1, std::memory_order_release); //invalidate seq
          memcpy(&cell->msg, msg, sizeof(T));
          cell->seq.store(pos, std::memory_order_release);
          return pos + 1;
        };
      }
    }
  }

  //pull next message from fifo buffer (subscriber_gen+1), if not found then pull closest gen in fifo buffer
  //returns false if no message in buffer, otherwise returns true, msg, and updated subscriber_gen
  bool pull_next(void* msg, uint32_t *subscriber_gen) override {
    for (;;) {
      //find pos for generation
      uint32_t pos = wpos.load(std::memory_order_relaxed);
      if(pos == 0) return false; //will miss a pull every 4,000,000,000 publishes...

      //find pos for generation
      uint32_t depth = pos - (*subscriber_gen + 1);
      if( (int32_t)depth < 0) depth = 0; 
      if( depth > buf_mask - MF_PUBSUB_DEPTH_OFFSET) depth = buf_mask - MF_PUBSUB_DEPTH_OFFSET;
      pos -= depth + 1; //pos is post-incremented on publish

      //pull msg from cell at pos
      cell_t* cell = &buf[pos & buf_mask]; 
      uint32_t seq1 = cell->seq.load(std::memory_order_acquire);
      if(seq1 == pos) {
        memcpy(msg, &cell->msg, sizeof(T));
        uint32_t seq2 = cell->seq.load(std::memory_order_acquire);
        if(seq2 == pos) { //exit if msg is consistent
          *subscriber_gen = pos + 1; //set subscriber_gen to the generation we retrieved
          return true; 
        }
      }
    }
  }

/*
  bool pull(T& msg, uint32_t generation) {
    for (;;) {
      //find pos for generation
      uint32_t pos = wpos.load(std::memory_order_relaxed);
      uint32_t depth = pos - generation;
      if( (int32_t)depth < 0 || depth > buf_mask ) return false; //generation not available in buffer
      pos -= depth + 1; //pos is post-incremented on publish

      //pull msg from cell at pos
      cell_t* cell = &buf[pos & buf_mask]; 
      uint32_t seq1 = cell->seq.load(std::memory_order_acquire);
      if(seq1 == pos) {
        memcpy(&msg, &cell->msg, sizeof(T));
        uint32_t seq2 = cell->seq.load(std::memory_order_acquire);
        if(seq2 == pos) return true; //exit if msg is consistent
      }
    }
  }
*/

};
