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

#include "../../hal/hal.h" //STM32 FreeRTOS

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
    static MsgTopicBase* _topic_list[MF_MSGTOPIC_LIST_SIZE];
    static uint32_t _stat_ts;
};

//=============================================================================
class MsgTopicBase {
  public:
    char name[9] = {};

    //pull last message from topic
    template <class T> bool pull_latest(T* msg) {
      uint32_t gen_to_get = get_generation();
      return _pull(msg, MsgTopicBase::PullOp::LAST_GREATER_EQUAL, &gen_to_get);
    }

  protected:
    friend class MsgBroker;
    friend class MsgSubscriptionBase;
    template <class T> friend class MsgSubscription;
    template <class T> friend class MsgTopic;
    template <class T> friend class MsgTopicMPMS;

    uint32_t _stat_start_gen = 0; //starting generation for statistics
    MsgSubscriptionBase* _sub_list[MF_MSGSUB_LIST_SIZE] = {}; //subscriptions for this topic

    virtual ~MsgTopicBase() {}
    MsgTopicBase(const char* name);

    enum class PullOp {LAST_GREATER_EQUAL, FIRST_GREATER_EQUAL};

    //interface
    virtual uint32_t get_generation() = 0; //counts messages published to this topic
    virtual uint32_t _publish(void* msg) = 0; //publish a message to the fifo
    virtual bool _pull(void* msg, PullOp op, uint32_t *gen_to_pull) = 0; //pull a message from the fifo

    //subscription
    void add_subscription(MsgSubscriptionBase *sub);
    void remove_subscription(MsgSubscriptionBase *sub);
    int subscriber_count();
};

#include "MsgTopic_impl.h"
#include "MsgTopicMPMS_impl.h"

//=============================================================================
class MsgSubscriptionBase {
  public:
    char name[9] = {};
    uint32_t get_generation() {
      return _sub_gen;
    }

  protected:
    friend class MsgBroker;
    template <class T> friend class MsgSubscription;

    uint32_t _sub_gen = 0; //last pulled topic generation 
    uint32_t _stat_pull_cnt = 0; //pull counter

    virtual ~MsgSubscriptionBase() {
      topic->remove_subscription(this);
    }

  private:
    MsgSubscriptionBase() {}
    MsgTopicBase *topic;
};

//=============================================================================
template <class T>
class MsgSubscription : public MsgSubscriptionBase {
  public:

    // Start a new subscription
    MsgSubscription(const char* name, MsgTopicBase *topic) {
      this->topic = topic;
      strncpy(this->name, name, sizeof(this->name) - 1);
      this->name[sizeof(this->name) - 1] = 0;
      topic->add_subscription(this);
    }

    // Pull next (oldest) message from fifo, which is newer than the previous message pulled
    // Returns true if found, false if no new message available
    bool pull_next(T *msg) {
        uint32_t gen_to_get = _sub_gen + 1; //get next msg
        if(!topic->_pull(msg, MsgTopicBase::PullOp::FIRST_GREATER_EQUAL, &gen_to_get)) return false;
        _sub_gen = gen_to_get;
        _stat_pull_cnt++;
        return true;
    }

    // Pull latest (newest) message from fifo, which is newer than the previous message pulled
    // Returns true if found, false if no new message available
    bool pull_latest(T *msg) {
        uint32_t gen_to_get = topic->get_generation();
        if(gen_to_get == _sub_gen) return false; //last msg in fifo was already pulled
        if(!topic->_pull(msg, MsgTopicBase::PullOp::LAST_GREATER_EQUAL, &gen_to_get)) return false;
        _sub_gen = gen_to_get;
        _stat_pull_cnt++;
        return true;
    }
};
