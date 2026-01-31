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

#include "msg.h"

//=============================================================================
// MsgBroker
//=============================================================================
MsgTopicBase* MsgBroker::topic_list[MF_MSGTOPIC_LIST_SIZE] = {};

void MsgBroker::add_topic(MsgTopicBase *topic) {
  for(int i = 0; i < MF_MSGTOPIC_LIST_SIZE; i++) {
    if(!topic_list[i]) {
      topic_list[i] = topic;
      return;
    }
  }
}

int MsgBroker::topic_count() {
  int cnt = 0;
  for(int i = 0; i < MF_MSGTOPIC_LIST_SIZE; i++) {
    if(topic_list[i]) {
      cnt++;
    }
  }
  return cnt;
}

void MsgBroker::top() {
  Serial.println("\n=== TOPICS AND SUBSCRIBERS ===\n");
  for(int i = 0; i < MF_MSGTOPIC_LIST_SIZE; i++) {
    MsgTopicBase *t = topic_list[i];
    if(t) {
      Serial.printf("topic:%-10s gen=%d subscribers=%d\n", t->name.c_str(), (int)t->generation, t->subscriber_count());
      for(int j = 0; j < MF_MSGSUB_LIST_SIZE; j++) {
        MsgSubscriptionBase *s = t->sub_list[j];
        if(s) Serial.printf("  sub:%-10s gen=%d published=%d missed=%d\n", s->name.c_str(), (int)s->generation, (int)s->pull_cnt, (int)(t->generation - s->pull_cnt));
      } 
    }
  }
}

//=============================================================================
// MsgTopicBase
//=============================================================================
MsgTopicBase::MsgTopicBase(String name, int len) {
  Serial.printf("cr topic %s %d\n",name.c_str(),len);Serial.flush();
    this->name = name;
    queue = xQueueCreate(1, len);
    MsgBroker::add_topic(this);
}
void MsgTopicBase::publish(void *msg) {
    xQueueOverwrite(queue, msg);
    generation++;
}
bool MsgTopicBase::pull(void *msg) {
    return (xQueuePeek(queue, msg, 0) == pdPASS);
}

void MsgTopicBase::add_subscription(MsgSubscriptionBase *sub) {
  for(int i = 0; i < MF_MSGSUB_LIST_SIZE; i++) {
    if(!sub_list[i]) {
      sub_list[i] = sub;
      return;
    }
  }
}

void MsgTopicBase::remove_subscription(MsgSubscriptionBase *sub) {
  for(int i = 0; i < MF_MSGSUB_LIST_SIZE; i++) {
    if(sub_list[i] = sub) {
      sub_list[i] = nullptr;
      return;
    }
  }
}

int MsgTopicBase::subscriber_count() {
  int cnt = 0;
  for(int i = 0; i < MF_MSGSUB_LIST_SIZE; i++) {
    if(sub_list[i]) {
      cnt++;
    }
  }
  return cnt;
}

//=============================================================================
// MsgSubscriptionBase
//=============================================================================
bool MsgSubscriptionBase::updated() {
    return (generation != topic->generation);
}

MsgSubscriptionBase::MsgSubscriptionBase(String name, MsgTopicBase *topic) : topic{topic} {
    this->name = name;
    topic->add_subscription(this);
}

MsgSubscriptionBase::~MsgSubscriptionBase() {
    topic->remove_subscription(this);
}

//pull message: returns true if msg was pulled, returns false if no msg available
bool MsgSubscriptionBase::pull(void *msg) {
    if(!topic->pull(msg)) return false; 
    generation = topic->generation;
    pull_cnt++;
    return true; //NOTE: retrieved msg might be older than this->generation
}

//pull updated message: returns true when updated msg available, else returns false and does not update msg 
bool MsgSubscriptionBase::pull_updated(void *msg) {
    if(!updated()) return false;
    return pull(msg);
}
