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

#include <Arduino.h> //String
#include "../hal/hal.h" //STM32 FreeRTOS

//#include <FreeRTOS.h>
//#include <queue.h>

#define TOPIC_LIST_SIZE 40 //max number of topics in MessageBroker. Only for statistics
#define SUB_LIST_SIZE 8 //max number of subscribers per topic. Only for statistics

class MsgBroker;
class MsgSubscriptionBase;
template <class T> class MsgSubscription;
class MsgTopicBase;
template <class T> class MsgTopic;

//=============================================================================
class MsgBroker {
    friend class MsgTopicBase;
  public:
    static int topic_count();
    static void top();
  protected:
    static void add_topic(MsgTopicBase *topic);
  private:
    static MsgTopicBase* topic_list[TOPIC_LIST_SIZE];
};

//=============================================================================
class MsgSubscriptionBase {
    friend class MsgBroker;
    template <class T> friend class MsgSubscription;
  protected:
    String name;
    uint32_t generation = 0; //last pulled topic generation 
    uint32_t pull_cnt = 0; //pull counter
  private:
    MsgSubscriptionBase() {}
};

//=============================================================================
class MsgTopicBase {
    friend class MsgBroker;
  protected:
    String name;
    uint32_t generation = 0; //counts messages published to this topic
    MsgSubscriptionBase* sub_list[SUB_LIST_SIZE] = {};
    MsgTopicBase(String name);
    void add_subscription(MsgSubscriptionBase *sub);
    void remove_subscription(MsgSubscriptionBase *sub);
    int subscriber_count();
  private:
    MsgTopicBase() {}
};

//=============================================================================
template <class T>
class MsgTopic : public MsgTopicBase {
    friend MsgSubscription<T>;

  public:
    MsgTopic(String name) : MsgTopicBase(name) {
        queue = xQueueCreate(1, sizeof(T));
    }

    //publish msg
    void publish(T *msg) {
        xQueueOverwrite(queue, msg);
        generation++;
    }

  protected:
    //pull last published message, returns false if no message was published
    bool pull(T *msg) {
        return (xQueuePeek(queue, msg, 0) == pdPASS);
    }

  private:
    QueueHandle_t queue;
};

//=============================================================================
template <class T>
class MsgSubscription : public MsgSubscriptionBase {
  public:
    //start a new subscription
    MsgSubscription(String name, MsgTopic<T> *topic) : topic{topic} {
        this->name = name;
        topic->add_subscription(this);
    }

    ~MsgSubscription() {
        topic->remove_subscription(this);
    }

    //returns true if new msg available
    bool updated() {
        return (generation != topic->generation);
    }

    //pull message, returns true and msg when new msg available; else returns false and does not update msg
    bool pull(T *msg) {
        if(!updated()) return false;
        if(!topic->pull(msg)) return false; 
        generation = topic->generation;
        pull_cnt++;
        return true; //NOTE: retrieved msg might be older than this->generation
    }

  private:
    MsgTopic<T> *topic;
};





