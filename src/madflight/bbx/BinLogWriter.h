#pragma once

#define LOG_TYPE_LEN  64    // max number of message types
#define QUEUE_LENGTH  100   // max number of messages in the queue
#define MAX_MSG_LEN   89    // max message len is FMT with 89 (0x59) bytes

#define HEAD_BYTE1  0xA3
#define HEAD_BYTE2  0x95


namespace BinLogWriter {
  enum Command_t { NONE, START, STOP };
  enum State_t { READY, STARTING, STARTED };

  extern uint32_t missCnt;

  void stop();
  void start();
  void setup();
  void log_msg(const char* msg);
  void log_parm(const char* name, float value, float default_value);
};