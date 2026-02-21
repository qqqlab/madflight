/* Modifications for madflight:

add memcopy
add peek()
rename variables

*/


/*  Multi-producer/multi-consumer bounded queue.MF_QueueLockFree
 *  Copyright (c) 2010-2011, Dmitry Vyukov. All rights reserved.
 *  Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright notice, this list of
 *        conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright notice, this list
 *        of conditions and the following disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *  THIS SOFTWARE IS PROVIDED BY DMITRY VYUKOV "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 *  DMITRY VYUKOV OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *  The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted
 *  as representing official policies, either expressed or implied, of Dmitry Vyukov.
 */ 

#pragma once

#include <atomic>
#include <cassert>
#include <cstring> //memcpy

template<typename T>
class MF_QueueLockFree
{
private:
  struct cell_t {
    std::atomic<size_t>   seq;
    T                     data;
  };

  cell_t* const           buf;
  size_t const            buf_mask;
  std::atomic<size_t>     wpos;
  std::atomic<size_t>     rpos;
  MF_QueueLockFree(MF_QueueLockFree const&);
  void operator = (MF_QueueLockFree const&);

  static inline size_t nextPowerOfTwo(size_t buffer_size) {
    size_t result = buffer_size - 1;
    for(size_t i = 1; i <= sizeof(size_t) * 4; i <<= 1) {
      result |= result >> i;
    }
    return result + 1;
  }

public:
  MF_QueueLockFree(size_t buffer_size)
    : buf(new cell_t [nextPowerOfTwo(buffer_size)])
    , buf_mask(nextPowerOfTwo(buffer_size) - 1)
  {
    buffer_size = buf_mask + 1;
    assert((buffer_size >= 2) && ((buffer_size & (buffer_size - 1)) == 0));
    for (size_t i = 0; i != buffer_size; i += 1) {
      buf[i].seq.store(i, std::memory_order_relaxed);
    }
    wpos.store(0, std::memory_order_relaxed);
    rpos.store(0, std::memory_order_relaxed);
  }

  ~MF_QueueLockFree() {
    delete [] buf;
  }

  size_t size() const {
    size_t head = rpos.load(std::memory_order_acquire);
    return wpos.load(std::memory_order_relaxed) - head;
  }

  size_t capacity() const {
    return buf_mask + 1;
  }

  bool push(T const& data) {
    for (;;) {
      size_t pos = wpos.load(std::memory_order_relaxed);
      cell_t* cell = &buf[pos & buf_mask];
      size_t seq = cell->seq.load(std::memory_order_acquire);
      intptr_t diff = (intptr_t)seq - (intptr_t)pos;
      if (diff == 0) {
        if (wpos.compare_exchange_weak(pos, pos + 1, std::memory_order_relaxed)) {
          memcpy(&cell->data, &data, sizeof(T));
          cell->seq.store(pos + 1, std::memory_order_release);
          return true;
        };
      } else if (diff < 0) {
        return false;
      }
    }

  }

  bool pop(T& data) {
    for (;;) {
      size_t pos = rpos.load(std::memory_order_relaxed);
      cell_t* cell = &buf[pos & buf_mask];
      size_t seq = cell->seq.load(std::memory_order_acquire);
      intptr_t diff = (intptr_t)seq - (intptr_t)(pos + 1);
      if (diff == 0) {
        if (rpos.compare_exchange_weak(pos, pos + 1, std::memory_order_relaxed)) {
          memcpy(&data, &cell->data, sizeof(T));
          cell->seq.store(pos + buf_mask + 1, std::memory_order_release);
          return true;
        };
      } else if (diff < 0) {
        return false;
      }
    }
  }

  bool peek(T& data, size_t depth = 0) {
    if(depth + 1 > size()) return false;
    for (;;) {
      size_t pos = rpos.load(std::memory_order_relaxed) + depth;
      cell_t* cell = &buf[pos & buf_mask];
      size_t seq = cell->seq.load(std::memory_order_acquire);
      intptr_t diff = (intptr_t)seq - (intptr_t)(pos + 1);
      if (diff == 0) {
        memcpy(&data, &cell->data, sizeof(T));
        return true;
      } else if (diff < 0) {
        return false;
      }
    }
  }

};
