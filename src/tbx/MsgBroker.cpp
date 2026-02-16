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

#include "MsgBroker.h"

//=============================================================================
// MsgBroker
//=============================================================================
MsgTopicBase* MsgBroker::topic_list[MF_MSGTOPIC_LIST_SIZE] = {};
uint32_t MsgBroker::stat_ts = micros();

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
  float dt = 1e-6 * (micros() - stat_ts);
  Serial.printf("\n=== Message Broker - Measurement Period: %.2f seconds ===\n\n", dt);
  for(int i = 0; i < MF_MSGTOPIC_LIST_SIZE; i++) {
    MsgTopicBase *t = topic_list[i];
    if(t) {
      uint32_t t_dgen = t->generation - t->stat_generation;
      Serial.printf("topic:%-12s freq:%4.0fHz  gen:%8d   pubs:%8d\n", t->name, t_dgen / dt, (int)t->generation, (int)t_dgen);
      for(int j = 0; j < MF_MSGSUB_LIST_SIZE; j++) {
        MsgSubscriptionBase *s = t->sub_list[j];
        if(s) Serial.printf("  sub:%-12s freq:%4.0fHz  gen:%8d  pulls:%8d  misses:%8d\n", s->name, s->stat_pull_cnt / dt, (int)s->generation, (int)s->stat_pull_cnt, (int)(t_dgen - s->stat_pull_cnt));
      } 
    }
  }
  reset_stats();
}

void MsgBroker::reset_stats() {
  stat_ts = micros();
  for(int i = 0; i < MF_MSGTOPIC_LIST_SIZE; i++) {
    MsgTopicBase *t = topic_list[i];
    if(t) {
      t->stat_generation = t->generation;
      for(int j = 0; j < MF_MSGSUB_LIST_SIZE; j++) {
        MsgSubscriptionBase *s = t->sub_list[j];
        if(s) s->stat_pull_cnt = 0;
      }       
    }
  }
}

//=============================================================================
// MsgTopicBase
//=============================================================================
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
    if(sub_list[i] == sub) {
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

MsgSubscriptionBase::MsgSubscriptionBase(const char *name, MsgTopicBase *topic) : topic{topic} {
    strncpy(this->name, name, sizeof(this->name) - 1);
    this->name[sizeof(this->name) - 1] = 0;
    topic->add_subscription(this);
}

MsgSubscriptionBase::~MsgSubscriptionBase() {
    topic->remove_subscription(this);
}

//pull message: returns true if msg was pulled, returns false if no msg available
bool MsgSubscriptionBase::pull(void *msg) {
    if(!topic->pull(msg)) return false;
    generation = topic->generation;
    stat_pull_cnt++;
    return true; //NOTE: retrieved msg might be older than this->generation
}

//pull updated message: returns true when updated msg available, else returns false and does not update msg 
bool MsgSubscriptionBase::pull_updated(void *msg) {
    if(!updated()) return false;
    return pull(msg);
}
