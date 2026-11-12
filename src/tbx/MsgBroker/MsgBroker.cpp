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


//#define DEBUG_PRINTF(...) MF_printf( __VA_ARGS__ )

#ifndef DEBUG_PRINTF
  #define DEBUG_PRINTF(...)
#endif

//=============================================================================
// MsgBroker
//=============================================================================
MsgTopicBase* MsgBroker::_topic_list[MF_MSGTOPIC_LIST_SIZE] = {};

#if MF_MSGBROKER_STAT
uint32_t MsgBroker::_stat_ts = micros();
#endif

void MsgBroker::add_topic(MsgTopicBase *topic) {
  DEBUG_PRINTF("MsgBroker::add_topic(%d)", topic->name);
  for(int i = 0; i < MF_MSGTOPIC_LIST_SIZE; i++) {
    if(!_topic_list[i]) {
      _topic_list[i] = topic;
      return;
    }
  }
}

int MsgBroker::topic_count() {
  int cnt = 0;
  for(int i = 0; i < MF_MSGTOPIC_LIST_SIZE; i++) {
    if(_topic_list[i]) {
      cnt++;
    }
  }
  return cnt;
}

void MsgBroker::top() {
#if MF_MSGBROKER_STAT  
  float dt = 1e-6 * (micros() - _stat_ts);
  MF_printf("\n=== Message Broker - Measurement Period: %.2f seconds ===\n\n", dt);
  MF_printf(" Type:Name           Freq      Gen    Count  Runtime\n");
  for(int i = 0; i < MF_MSGTOPIC_LIST_SIZE; i++) {
    MsgTopicBase *t = _topic_list[i];
    if(t) {
      uint32_t push_cnt = t->get_generation() - t->_stat_start_gen;
      MF_printf(
        "topic:%-12s %4.0fHz %8d %8d %5.1fus  retries:%d %d  msglen:%d\n"
        , t->name
        , push_cnt / dt
        , (int)t->get_generation()
        , (int)push_cnt
        , (push_cnt==0 ? 0.0 : (float)t->_stat_push_dt / push_cnt)
        , (int)t->_stat_push_retry_cnt
        , (int)t->_stat_pull_retry_cnt
        , (int)t->_stat_msglen
      );
      for(int j = 0; j < MF_MSGSUB_LIST_SIZE; j++) {
        MsgSubscriptionBase *s = t->_sub_list[j];
        if(s) MF_printf(
          "  sub:%-12s %4.0fHz %8d %8d %5.1fus\n"
          , s->name
          , s->_stat_pull_cnt / dt
          , (int)s->_sub_gen
          , (int)s->_stat_pull_cnt
          , (s->_stat_pull_cnt==0 ? 0.0 : (float)s->_stat_pull_dt / s->_stat_pull_cnt)
        );
      } 
    }
  }
  reset_stats();
#else
  MF_printf("\n=== Message Broker statistics disabled, use MF_MSGBROKER_STAT=1 to enable ===\n");
#endif
}

void MsgBroker::reset_stats() {
#if MF_MSGBROKER_STAT
  _stat_ts = micros();
  for(int i = 0; i < MF_MSGTOPIC_LIST_SIZE; i++) {
    MsgTopicBase *t = _topic_list[i];
    if(t) {
      t->_stat_start_gen = t->get_generation();
      t->_stat_push_dt = 0;
      t->_stat_push_retry_cnt = 0;
      t->_stat_pull_retry_cnt = 0;
      for(int j = 0; j < MF_MSGSUB_LIST_SIZE; j++) {
        MsgSubscriptionBase *s = t->_sub_list[j];
        if(s) {
          s->_stat_pull_cnt = 0;
          s->_stat_pull_dt = 0;
        }
      }       
    }
  }
#endif
}

//=============================================================================
// MsgTopicBase
//=============================================================================
MsgTopicBase::MsgTopicBase(const char* name) {
  DEBUG_PRINTF("MsgTopicBase::new topic=%s\n", name);
  strncpy(this->name, name, sizeof(this->name) - 1);
  this->name[sizeof(this->name) - 1] = 0;
  MsgBroker::add_topic(this);
}

void MsgTopicBase::add_subscription(MsgSubscriptionBase *sub) {
  DEBUG_PRINTF("MsgTopicBase::add_subscription sub=%s topic=%s\n", sub->name, name);
  for(int i = 0; i < MF_MSGSUB_LIST_SIZE; i++) {
    if(!_sub_list[i]) {
      _sub_list[i] = sub;
      return;
    }
  }
}

void MsgTopicBase::remove_subscription(MsgSubscriptionBase *sub) {
  DEBUG_PRINTF("MsgTopicBase::remove_subscription sub=%s topic=%s\n", sub->name, name);
  for(int i = 0; i < MF_MSGSUB_LIST_SIZE; i++) {
    if(_sub_list[i] == sub) {
      _sub_list[i] = nullptr;
      return;
    }
  }
}

int MsgTopicBase::subscriber_count() {
  int cnt = 0;
  for(int i = 0; i < MF_MSGSUB_LIST_SIZE; i++) {
    if(_sub_list[i]) {
      cnt++;
    }
  }
  return cnt;
}
