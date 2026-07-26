/*==========================================================================================
MIT License

Copyright (c) 2023-2025 https://madflight.com

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

#include "param.yaml.h"

#define CFG_HDR0 'm'
#define CFG_HDR1 'a'
#define CFG_HDR2 'd'
#define CFG_HDR3 '2'

class CfgClass : public CfgParam {
private:
  //keep CfgHeader 40 bytes long!!!
  struct __attribute__((packed)) CfgHeader {
    uint8_t header0 = CFG_HDR0;
    uint8_t header1 = CFG_HDR1;
    uint8_t header2 = CFG_HDR2;
    uint8_t header3 = CFG_HDR3;
    uint16_t len = 0; //number of bytes for hdr+param+crc
    uint16_t _reserved0 = 0;
    uint32_t madflight_param_crc = 0;
    uint8_t _reserved1[28] = {0};
  } hdr;

public:
  CfgClass();
  void begin();

  //indexed parameter manipulation
  uint16_t paramCount(); //get number of parameters
  int getIndex(String namestr); //get parameter index for a parameter name
  bool getNameAndValue(uint16_t index, String* name, float* value); //get parameter name and value for index

  //named parameter manipulation
  bool setParam(String namestr, String val); //CLI set a parameter value, returns true on success
  bool setParamMavlink(String namestr, float val); //set a parameter value, returns true on success
  float getValue(String namestr, float default_value); //get parameter as float

  //loading and saving
  void clear(); //load defaults from param_list
  void loadFromEeprom(); //read parameters from eeprom/flash
  void loadFromString(const char *batch); //load text unconditional
  void load_madflight(const char *board, const char *config); //load board+config if crc is different
  void writeToEeprom(); //write config to flash

  //CLI commands
  void cli_dump(const char* filter = nullptr); //CLI dump: print all config values
  void cli_diff(const char* filter = nullptr); //CLI diff: print all modified config values

  //print
  bool getOptionString(uint16_t param_idx, int32_t param_val, char out_option[20]);
  void printPins();
  enum class printModuleMode {GIZMO, CFG_ERROR, GIZMO_NO_CR};
  void printModule(const char* module_name, printModuleMode mode = printModuleMode::GIZMO);
  void printNameAndValue(uint16_t i, const char* comment = nullptr);
  void printValue(uint16_t i);

private:
  bool load_cmdline(String cmdline);
  int get_enum_index(const char* k, const char* values);
  void print_options(const char *str); //print option list without "mf_"
  float getValue(int i); //get parameter as float
  bool setValue(int i, float val); //set parameter value
};

extern CfgClass cfg;