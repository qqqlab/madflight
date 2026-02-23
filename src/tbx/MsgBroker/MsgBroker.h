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
    static MsgTopicBase* topic_list[MF_MSGTOPIC_LIST_SIZE];
    static uint32_t stat_ts;
};

//=============================================================================
class MsgTopicBase {
  protected:
    friend class MsgBroker;
    friend class MsgSubscriptionBase;
    template <class T> friend class MsgTopic;
    template <class T> friend class MsgTopicQueue;
    template <typename T> friend class MsgTopicMPMS;

    char name[9] = {};
    uint32_t stat_generation = 0; //starting generation for statistics
    MsgSubscriptionBase* sub_list[MF_MSGSUB_LIST_SIZE] = {};

    virtual ~MsgTopicBase() {}
    MsgTopicBase(const char* name);

    //interface
    virtual uint32_t get_generation() = 0; //counts messages published to this topic
    virtual uint32_t publish(void* msg) = 0; //publish a message
    virtual bool pull_next(void* msg, uint32_t *subscriber_gen) = 0; //pull next message relative to subscriber_gen

    //subscription
    void add_subscription(MsgSubscriptionBase *sub);
    void remove_subscription(MsgSubscriptionBase *sub);
    int subscriber_count();
};

#include "MsgTopic_impl.h"
#include "MsgTopicMPMS_impl.h"
#include "MsgTopicQueue_impl.h"

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

    bool pull_next(void *msg); //pull next message: returns true if msg was pulled, returns false if no msg available
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
    bool pull(T *msg) { return MsgSubscriptionBase::pull_next(msg); }
    bool pull_updated(T *msg) { return MsgSubscriptionBase::pull_updated(msg); }
    bool pull_last(T *msg) { return MsgSubscriptionBase::pull_last(msg); }
};
