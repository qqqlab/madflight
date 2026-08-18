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
    volatile uint32_t _topic_gen = 0;
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
        return _topic_gen;
    }

    uint32_t publish(T *msg) {
      return _publish((void*) msg);
    }

    //pull latest message from topic
    bool pull_latest(T* msg) {
      uint32_t gen_to_get = get_generation();
      return _pull(msg, MsgTopicBase::PullOp::LAST_GREATER_EQUAL, &gen_to_get);
    }

  protected:  
    //publish a message, returns the published message generation
    uint32_t _publish(void *msg) override {
      memcpy(bufin, msg, buflen); //use buflen for speed (reading potentially some garbage at the end of msg)
      _topic_gen = _topic_gen + 1; //update as soon as data is in buf
      if(bufin < buflast) bufin += buflen; else bufin = buf; //calc next bufin
      return _topic_gen;
    }

   bool _pull(void* msg, PullOp op, uint32_t *gen_to_pull) override {
      if(!_topic_gen) return false; //will miss a pull every 4,000,000,000 publishes, but don't need additional vars/checks...
      uint8_t tries = 5;
      do {
        uint32_t tgen = _topic_gen; //copy the current topic generation (_topic_gen can change with publish in other thread)
        int32_t depth = (int32_t)(tgen - *gen_to_pull); //we're interested in the message at this depth
        //generations [tgen - bufdepth_1 - 1 ... tgen] are in the fifo
        //depth [bufdepth_1 - 1 ... 0] are in the fifo
        switch(op) {
          case PullOp::LAST_GREATER_EQUAL:
            if(depth < 0) return false; //the gen we want is not yet in fifo
            depth = 0; //pick the most recent msg in fifo
            break;
          case PullOp::FIRST_GREATER_EQUAL:
            if(depth < 0) return false; //the gen we want is not yet in fifo
            if(depth > bufdepth_1 - 1) depth = bufdepth_1 - 1; //pick oldest msg in fifo
            break;
        }

        //get msg at depth
        uint8_t* bufout = buf + ((tgen - depth) % (bufdepth_1 + 1)) * buflen; //get pointer to message at this depth
        memcpy(msg, bufout, msglen); //copy the message
        if((_topic_gen - tgen) < (bufdepth_1 - depth)) { //if the message at depth did not get overwitten by publish(), then we have a consistent copy
          *gen_to_pull = tgen - depth; //update the subscriber generation
          return true;
        }
        //the message got updated while we were memcpy'ing it, try again...
        //note: maybe increase subscriber_gen to look for next message and hopefully have a greater change of getting it
      }while(--tries);
      return false;
    }

};
