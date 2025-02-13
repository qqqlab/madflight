#pragma once

#include "../bb_interfase.h"

// dummy BinLog class for (not) logging a message to a binary log
class BinLog {
public:
  BinLog_interface(const char* name) {}
  ~BinLog_interface() {}

  // Message fields
  void TimeUS() {}
  void TimeUS(uint32_t ts) {}
  void u8(const char* label, uint8_t v, float mult = 1, const char* unit = NULL) {}
  void u8flightmode(const char* label, uint8_t v, float mult = 1, const char* unit = NULL) {}
  void u16(const char* label, uint16_t v, float mult = 1, const char* unit = NULL) {}
  void u16x100(const char* label, uint16_t v, float mult = 1, const char* unit = NULL) {}
  void u32(const char* label, uint32_t v, float mult = 1, const char* unit = NULL) {}
  void u32x100(const char* label, uint32_t v, float mult = 1, const char* unit = NULL) {}
  void u64(const char* label, uint64_t v, float mult = 1, const char* unit = NULL) {}
  void i8(const char* label, int8_t v, float mult = 1, const char* unit = NULL) {}
  void i16(const char* label, int16_t v, float mult = 1, const char* unit = NULL) {}
  void i16x100(const char* label, int16_t v, float mult = 1, const char* unit = NULL) {}
  void i32(const char* label, int32_t v, float mult = 1, const char* unit = NULL) {}
  void i32x100(const char* label, int32_t v, float mult = 1, const char* unit = NULL) {}
  void i32latlon(const char* label, int32_t v, float mult = 1, const char* unit = NULL) {}
  void i64(const char* label, int64_t v, float mult = 1, const char* unit = NULL) {}
  void flt(const char* label, float v, float mult = 1, const char* unit = NULL) {}
  void dbl(const char* label, double v, float mult = 1, const char* unit = NULL) {}
  void char4(const char* label, const char* v, float mult = 1, const char* unit = NULL) {}
  void char16(const char* label, const char* v, float mult = 1, const char* unit = NULL) {}
  void char64(const char* label, const char* v, float mult = 1, const char* unit = NULL) {}
  void blob64(const char* label, const int16_t* v, float mult = 1, const char* unit = NULL) {}
}

BlackBox bb_instance;
BlackBox &bb = bb_instance;
