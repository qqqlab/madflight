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
    uint32_t board_and_config_crc = 0;
    uint8_t _reserved1[28] = {0};
  };

public:
  CfgClass();
  //loading and saving
  void setup(const char *board, const char *config);
  void clear(); //clear config to default param (does not load madflight_board, madflight_config)
  void save(); //save to eeprom

  //indexed parameter manipulation
  uint16_t paramCount(); //get number of parameters
  int getIndex(String namestr); //get parameter index for a parameter name
  bool getNameAndValue(uint16_t index, String* name, float* value); //get parameter name and value for index

  //named parameter manipulation
  bool cli_set_param(String namestr, String val); //CLI set a parameter value, returns true on success
  bool mavlink_set_param(String namestr, float val); //set a parameter value, returns true on success
  float get_param(String namestr, float default_value); //get parameter as float

  //CLI commands
  void cli_dump(const char* filter = nullptr, bool diff = false); //CLI dump: print all config values
  void cli_diff(const char* filter = nullptr); //CLI diff: print all modified config values
  void cli_defaults(const char* filter = nullptr); //CLI defaults: load defaults from param_list (without publishing)
  void cli_save(); //CLI save: save param and reboot

  //print
  bool getOptionString(uint16_t param_idx, int32_t param_val, char out_option[20]);
  void printPins();
  enum class printModuleMode {GIZMO, CFG_ERROR, GIZMO_NO_CR};
  void printModule(const char* module_name, printModuleMode mode = printModuleMode::GIZMO);
  void printNameAndValue(uint16_t i, const char* comment = nullptr);
  void printValue(uint16_t i);

private:
  bool _load_cmdline(String cmdline);
  int _get_enum_index(const char* k, const char* values);
  void _print_options(const char *str); //print option list without "mf_"
  float _get_param(int i); //get parameter as float
  bool _set_param(int i, float val, bool publish = true); //set parameter value, and publish changes

  uint32_t _calc_board_and_config_crc();
  void _load(); //load defaults, eeprom, board+config
  void _load_from_string(const char *batch); //load text unconditional

  const char *_board = nullptr; //madflight_board config string
  const char *_config = nullptr; //madflight_config config string
  bool _is_board_and_config_loaded = false;
};

extern CfgClass cfg;