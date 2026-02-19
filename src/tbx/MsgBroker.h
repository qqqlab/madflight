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

#pragma once

#ifndef MF_MSGTOPIC_LIST_SIZE
  #define MF_MSGTOPIC_LIST_SIZE 40 //max number of topics. Used only for top() statistics
#endif
#ifndef MF_MSGSUB_LIST_SIZE
  #define MF_MSGSUB_LIST_SIZE 8 //max number of subscribers per topic. Used only for top() statistics
#endif

#include "../hal/hal.h" //STM32 FreeRTOS

class MsgBroker;
class MsgSubscriptionBase;
template <class T> class MsgSubscription;
class MsgTopicBase;
template <class T> class MsgTopic;

//=============================================================================
class MsgBroker {
  public:
    static int topic_count();
    static void top();
    static void reset_stats();

  protected:
    friend class MsgTopicBase;
    template <class T> friend class MsgTopic;

    static void add_topic(MsgTopicBase *topic);
  private:
    static MsgTopicBase* topic_list[MF_MSGTOPIC_LIST_SIZE];
    static uint32_t stat_ts;
};

//=============================================================================
class MsgTopicBase {
  public:
    uint32_t get_generation() {return generation;}
    inline void publish_generation_only() {generation++;}

  protected:
    friend class MsgBroker;
    friend class MsgSubscriptionBase;
    template <class T> friend class MsgTopic;
    template <class T> friend class MsgTopicQueue;

    char name[9] = {};
    volatile uint32_t generation = 0; //counts messages published to this topic
    uint32_t stat_generation = 0; //starting generation for statistics
    MsgSubscriptionBase* sub_list[MF_MSGSUB_LIST_SIZE] = {};

    virtual ~MsgTopicBase() {}
    MsgTopicBase(const char* name);
  
    //pull next message with gen
    virtual bool pull(void* msg, uint32_t *subscriber_gen) = 0;

    //subscription
    void add_subscription(MsgSubscriptionBase *sub);
    void remove_subscription(MsgSubscriptionBase *sub);
    int subscriber_count();
};

//=============================================================================
//single publisher, multi subscriber - lockless - uses FIFO buffer for storage
template <class T>
class MsgTopic : public MsgTopicBase {
  private:
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

      MsgBroker::add_topic(this);
    }

    void publish(T *msg) { 
      memcpy(bufin, msg, buflen); //use buflen for speed (reading potentially some garbage at the end of msg)
      generation++; //update as soon as data is in buf
      if(bufin < buflast) bufin += buflen; else bufin = buf; //calc next bufin
    }

    //pull message subscriber_gen+1 from fifo buffer, if not found then pull closest gen in fifo buffer
    bool pull(void* msg, uint32_t *subscriber_gen) override {
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

//=============================================================================
//multi publisher, multi subscriber - uses FreeRTOS Queue for storage
template <class T>
class MsgTopicQueue : public MsgTopicBase {
  public:
    MsgTopicQueue(const char* name, int len) : MsgTopicBase(name) {
      queue = xQueueCreate(1, len);
      MsgBroker::add_topic(this);
    }

    void publish(void *msg) {
      xQueueOverwrite(queue, msg);
      generation++;
    }

    void publishFromISR(void *msg) {
      BaseType_t higherPriorityTaskWoken;
      xQueueOverwriteFromISR(queue, msg, &higherPriorityTaskWoken);
      (void)higherPriorityTaskWoken;
      generation++;
    }

    bool pull(void* msg, uint32_t *subscriber_gen) override {
      uint8_t tries = 5;
      uint32_t topic_gen;
      do {
        topic_gen = generation; //copy the current topic generation (generation is volatile)
        if(xQueuePeek(queue, msg, 0) != pdPASS) return false; //exit if nothing in the queue
        if(topic_gen == generation) { //if generation did not change, then we got the message for that generation
          *subscriber_gen = topic_gen; //update the subscriber generation
          return true;
        }
      }while(--tries);
      //give up, return guess of retrieved generation
      *subscriber_gen = topic_gen; //update the subscriber generation
      return true;
    }

  private:
    QueueHandle_t queue = nullptr;
};

//=============================================================================
class MsgSubscriptionBase {
  public:
    bool updated(); //returns true if new msg available

  protected:
    friend class MsgBroker;
    template <class T> friend class MsgSubscription;

    char name[9] = {};
    uint32_t generation = 0; //last pulled topic generation 
    uint32_t stat_pull_cnt = 0; //pull counter

    virtual ~MsgSubscriptionBase();
    MsgSubscriptionBase(const char* name, MsgTopicBase *topic); //start a new subscription

    bool pull(void *msg); //pull message: returns true if msg was pulled, returns false if no msg available
    bool pull_updated(void *msg); //pull updated message: returns true when updated msg available, else returns false and does not update msg
    bool pull_last(void *msg); //pull last message: returns true if msg was pulled, returns false if no msg available

  private:
    MsgSubscriptionBase() {}
    MsgTopicBase *topic;
};

//=============================================================================
template <class T>
class MsgSubscription : public MsgSubscriptionBase {
  public:
    MsgSubscription(const char* name, MsgTopicBase *topic) : MsgSubscriptionBase(name, topic) {}
    bool pull(T *msg) { return MsgSubscriptionBase::pull(msg); }
    bool pull_updated(T *msg) { return MsgSubscriptionBase::pull_updated(msg); }
    bool pull_last(T *msg) { return MsgSubscriptionBase::pull_last(msg); }
};
