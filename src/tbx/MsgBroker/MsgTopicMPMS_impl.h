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
    std::atomic<uint32_t>   gen; //cell generation
    T                       msg;
  };

  cell_t* const             buf;
  uint32_t const            buf_mask;
  std::atomic<uint32_t>     topic_gen; //topic generation

  static inline uint32_t nextPowerOfTwo(uint32_t buffer_size) {
    uint32_t result = buffer_size - 1;
    for(uint32_t i = 1; i <= sizeof(uint32_t) * 4; i <<= 1) {
      result |= result >> i;
    }
    return result + 1;
  }

public:
  MsgTopicMPMS(char * name, uint32_t buffer_size)
    : buf(new cell_t [nextPowerOfTwo(buffer_size)])
    , buf_mask(nextPowerOfTwo(buffer_size) - 1)
    , MsgTopicBase(name)
  {
    buffer_size = buf_mask + 1;
    assert((buffer_size >= 2) && ((buffer_size & (buffer_size - 1)) == 0));
    //write valid gen: i.e. buf[0].gen=-4, [1]=-3, [2]=-2, [3]=-1 for size==4
    for (uint32_t i = 0; i != buffer_size; i += 1) {
      buf[i].gen.store(i - buffer_size, std::memory_order_relaxed); 
    }
    topic_gen.store(0, std::memory_order_relaxed);
  }

  ~MsgTopicMPMS() {
    delete [] buf;
  }

  uint32_t capacity() const {
    return buf_mask + 1;
  }

  uint32_t get_generation() override {
    return topic_gen.load(std::memory_order_relaxed);
  }

  //publish a message, returns the published message generation
  uint32_t publish(void* msg) override {
    for (;;) {
      uint32_t tgen = topic_gen.load(std::memory_order_relaxed);
      cell_t* cell = &buf[tgen & buf_mask];
      uint32_t cgen = cell->gen.load(std::memory_order_acquire);
      if (cgen + buf_mask + 1 == tgen) { //cgen should be buf_size behind tgen (buf_size = buf_mask + 1), if not then another thread published while we read tgen and cgen
        if (topic_gen.compare_exchange_weak(tgen, tgen + 1, std::memory_order_relaxed)) { //increase topic_gen, this invalidates buf[tgen].gen
          memcpy(&cell->msg, msg, sizeof(T)); //write msg data to cell
          cell->gen.store(tgen, std::memory_order_release); //write valid buf[tgen].gen
          return tgen + 1; //return the topic_gen
        };
      }
    }
  }

  //pull message with topic_gen == subscriber_gen+1, if not found then pull closest topic_gen available in buffer
  //returns true, msg, and updated subscriber_gen if publish() was called at least once, else returns false and unmodified msg, subscriber_gen
  bool pull_next(void* msg, uint32_t *subscriber_gen) override {
    for (;;) {
      uint32_t tgen = topic_gen.load(std::memory_order_relaxed);
      if(tgen == 0) return false; //will miss a pull every 4,000,000,000 publishes, but don't need additional vars/checks...

      //find tgen for subscriber_gen+1
      uint32_t depth = tgen - (*subscriber_gen + 1);
      if( (int32_t)depth < 0) depth = 0; //subscriber_gen+1 > topic_gen -> look for depth = 0 (newest topic_gen in buffer)
      if( depth > buf_mask - MF_PUBSUB_DEPTH_OFFSET) depth = buf_mask - MF_PUBSUB_DEPTH_OFFSET; //subscriber_gen+1 <= topic_gen-buf_size -> look for depth = buf_size-1 (oldest topic_gen in buffer)
      tgen -= depth + 1; //tgen is post-incremented on publish, so decrement tgen by 1 here

      //pull msg from buf[tgen]
      cell_t* cell = &buf[tgen & buf_mask]; 
      uint32_t cgen1 = cell->gen.load(std::memory_order_acquire);
      if(cgen1 == tgen) { //check cell.gen
        memcpy(msg, &cell->msg, sizeof(T)); //retrieve msg data from cell
        uint32_t cgen2 = cell->gen.load(std::memory_order_acquire);
        if(cgen2 == tgen) { //if cell.gen did not change, then msg is consistent
          *subscriber_gen = tgen + 1; //set subscriber_gen to the topic_gen we retrieved
          return true; 
        }
      }
    }
  }

};
