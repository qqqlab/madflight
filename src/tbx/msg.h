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

#include <Arduino.h> //String
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
  protected:
    friend class MsgTopicBase;

    static void add_topic(MsgTopicBase *topic);
  private:
    static MsgTopicBase* topic_list[MF_MSGTOPIC_LIST_SIZE];
};

//=============================================================================
class MsgTopicBase {
  public:
    uint32_t get_generation() {return generation;}

  protected:
    friend class MsgBroker;
    friend class MsgSubscriptionBase;

    String name;
    uint32_t generation = 0; //counts messages published to this topic
    MsgSubscriptionBase* sub_list[MF_MSGSUB_LIST_SIZE] = {};

    MsgTopicBase(String name, int len);
    void publish(void *msg);
    bool pull(void *msg);
    void add_subscription(MsgSubscriptionBase *sub);
    void remove_subscription(MsgSubscriptionBase *sub);
    int subscriber_count();
    virtual ~MsgTopicBase() {}
  private:
    MsgTopicBase() {}
    QueueHandle_t queue;
};

//=============================================================================
class MsgSubscriptionBase {
  public:
    bool updated(); //returns true if new msg available
  protected:
    friend class MsgBroker;
    template <class T> friend class MsgSubscription;

    String name;
    uint32_t generation = 0; //last pulled topic generation 
    uint32_t pull_cnt = 0; //pull counter

    MsgSubscriptionBase(String name, MsgTopicBase *topic); //start a new subscription
    virtual ~MsgSubscriptionBase();
    bool pull(void *msg); //pull message: returns true if msg was pulled, returns false if no msg available
    bool pull_updated(void *msg); //pull updated message: returns true when updated msg available, else returns false and does not update msg
  private:
    MsgTopicBase *topic;
    MsgSubscriptionBase() {}
};

//=============================================================================
template <class T>
class MsgTopic : public MsgTopicBase {
  public:
    MsgTopic(String name) : MsgTopicBase(name, sizeof(T)) {}
    void publish(T *msg) { MsgTopicBase::publish(msg); }
  protected:
    bool pull(T *msg) { return MsgTopicBase::pull(msg); }
};

//=============================================================================
template <class T>
class MsgSubscription : public MsgSubscriptionBase {
  public:
    MsgSubscription(String name, MsgTopic<T> *topic) : MsgSubscriptionBase(name, topic) {}
    bool pull(T *msg) { return MsgSubscriptionBase::pull(msg); }
    bool pull_updated(T *msg) { return MsgSubscriptionBase::pull_updated(msg); }
};
