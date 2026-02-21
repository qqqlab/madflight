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

//single publisher, multi subscriber - fifo - lockfree

#pragma once

template <class T>
class MsgTopic : public MsgTopicBase {
  private:
    uint32_t generation = 0;
    uint16_t msglen = 0; //message length
    uint16_t buflen = 0; //message length rounded up to next 4 bytes for alignment
    uint16_t bufdepth_1 = 0; //size of buffer in messages minus 1 (E.g. exclusive one message used for loading new data)
    uint8_t *buf = nullptr; //buffer for bufdepth messages
    uint8_t *buflast = nullptr; //pre-calculated pointer to last message
    uint8_t *bufin = nullptr; //next publish message buffer 
  
  public:
    MsgTopic(const char* name, uint16_t fifo_depth = 1) : MsgTopicBase(name) {
      bufdepth_1 = fifo_depth;
      msglen = sizeof(T);
      buflen = ((msglen + 3) / 4) * 4; //round up to the next 4 bytes
      buf = (uint8_t*)aligned_alloc(4, buflen * (bufdepth_1 + 1)); //add one extra message for loading new data
      memset(buf, 0, buflen * (bufdepth_1 + 1)); //clear buffer (not really needed)
      bufin = buf + buflen;
      buflast = buf + bufdepth_1 * buflen;
    }

    uint32_t get_generation() override {
        return generation;
    }

    //publish a message, returns the published message generation
    uint32_t publish (void *msg) override {
      memcpy(bufin, msg, buflen); //use buflen for speed (reading potentially some garbage at the end of msg)
      generation++; //update as soon as data is in buf
      if(bufin < buflast) bufin += buflen; else bufin = buf; //calc next bufin
      return generation;
    }

    //pull next message from fifo buffer (subscriber_gen+1), if not found then pull closest gen in fifo buffer
    //returns false if no message in buffer, otherwise returns true, msg, and updated subscriber_gen
    bool pull_next(void* msg, uint32_t *subscriber_gen) override {
      if(!generation) return false; //will miss a pull every 4,000,000,000 publishes...
      uint8_t tries = 5;
      do {
        uint32_t topic_gen = generation; //copy the current topic generation (generation is volatile)
        uint32_t depth = topic_gen - (*subscriber_gen + 1); //we're interested in the message at this depth
        //limit depth to [0 .. bufdepth_1 - 1]
        if(depth & 0x80000000) { //unsigned negative
          depth = 0;
        } else if(depth >= bufdepth_1) {
          depth = bufdepth_1 - 1; 
        }
        uint8_t* bufout = buf + ((topic_gen - depth) % (bufdepth_1 + 1)) * buflen; //get pointer to message at this depth
        memcpy(msg, bufout, msglen); //copy the message
        if((generation - topic_gen) < (bufdepth_1 - depth)) { //if the message at depth did not get overwitten by publish(), then we have a consistent copy
          *subscriber_gen = topic_gen - depth; //update the subscriber generation
          return true;
        }
        //the message got updated while we were memcpy'ing it, try again...
        //note: maybe increase subscriber_gen to look for next message and hopefully have a greater change of getting it
      }while(--tries);
      return false;
    }
};
