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

#include "cfg.h" //MF_PARAM_LIST and enums for options
#include "../hal/hal.h"
#include "../tbx/tbx_crc.h"
#include "../bbx/bbx.h"

//create global module instance
CfgClass cfg;

CfgClass::CfgClass() {}

void CfgClass::setup(const char *board, const char *config) {
  _board = board;
  _config = config;
  hal_eeprom_begin();
  cfg._load();
}

//get number of parameters
uint16_t CfgClass::paramCount() {
  return Cfg::param_cnt;
}

//get parameter name and value for index
bool CfgClass::getNameAndValue(uint16_t index, String* name, float* value) {
  if(index >= paramCount()) return false;
  *name = Cfg::param_list[index].name;
  *value = _get_param(index);
  return true;
}

//get parameter value as float
float CfgClass::get_param(String namestr, float default_value) {
  int i = getIndex(namestr);
  if(i<0) return default_value;
  return _get_param(i);
}

//get enum option name for param_idx and param_val
bool CfgClass::getOptionString(uint16_t param_idx, int32_t param_val, char out_option[20]) {
  out_option[0] = 0;
  if(param_idx>=paramCount()) return false;
  if(param_val<0) return false;
  const char *option = Cfg::param_list[param_idx].options;
  int opt_i = 0;
  int pos = 0;
  //skip i-1 commas
  while(option[pos] && opt_i < (int)param_val) {
    if(option[pos] == ',') opt_i++;
    pos++;
  }
  if(!option[pos]) return false;
  int out_i = 0;
  while(option[pos] && option[pos] != ',' && out_i<20-1) {
    out_option[out_i] = option[pos];
    pos++;
    out_i++;
  }
  out_option[out_i] = 0;
  //remove "mf_" prefix
  if(strncmp(out_option,"mf_",3)==0) strcpy(out_option, out_option + 3);
  return true;
}

//print all parameters for module_name on single line
void CfgClass::printModule(const char* module_name, printModuleMode mode) {
  if(mode == printModuleMode::CFG_ERROR) {
    Serial.println();
  }
  
  String modname = String(module_name);
  modname.toUpperCase();
  Serial.printf("%s: ", modname.c_str());
  modname.toLowerCase();
  String modname_ = modname + '_';

  //print gizmo
  String type_name = modname_ + "gizmo";
  int gizmo_i = getIndex(type_name);
  if(gizmo_i >= 0) {
    printValue(gizmo_i);
    Serial.print(" ");
  }

  if(mode == printModuleMode::GIZMO) {
    Serial.println();
    return;
  }

  if(mode == printModuleMode::GIZMO_NO_CR) {
    Serial.print("- ");
    return;
  }

  if(mode == printModuleMode::CFG_ERROR) {
    Serial.print("ERROR check pin/bus config: ");
  }

  //print config
  for(int i = 0; i < paramCount(); i++) {
    if(strncmp(Cfg::param_list[i].name, modname_.c_str(), modname_.length()) == 0 && i != gizmo_i) { //starts with module_name + '_', omit gizmo
      Serial.print(Cfg::param_list[i].name);
      Serial.print(':');
      printValue(i);
      Serial.print(' ');
    }
  }
  //print module pins
  String pinname = "pin_" + modname; 
  for(int i = 0; i < paramCount(); i++) {
    if(strncmp(Cfg::param_list[i].name, pinname.c_str(), pinname.length()) == 0) { //starts with 'pin_' + module_name
      Serial.print(Cfg::param_list[i].name);
      Serial.print(':');
      printValue(i);
      Serial.print(' ');
    }
  }
  Serial.println();

  if(mode == printModuleMode::CFG_ERROR) {
    Serial.println();
  }
}

//print "<name> <value> # options: <options>" for given param_index
void CfgClass::printNameAndValue(uint16_t i, const char* comment) {
  if(i>paramCount()) return;
  Serial.printf("%-16s", Cfg::param_list[i].name);
  Serial.print(' ');
  printValue(i);
  if(comment) Serial.printf(" # %s", comment);
  const char *options = Cfg::param_list[i].options;
  if(options && options[0] != 0) {
    Serial.printf(" # options: ");
    _print_options(options);
  }
  Serial.println();
}

//print param value
void CfgClass::printValue(uint16_t i) {
  if(i >= paramCount()) return;
  float val = _get_param(i);
  switch(Cfg::param_list[i].type) {
    case 'e': { //enum
      char option[20];
      if(getOptionString(i, val, option)) {
        Serial.print(option);
      }else{
        Serial.printf("%d", (int)val); //option lookup failed, print numeric value
      }
      break;
    }
    case 'f': //float
      Serial.printf("%f", val);
      break;
    case 'i': //integer
      Serial.printf("%d", (int)val);
      break;
    case 'p': //pinnumber/pinname
      hal_print_pin_name(val);
      break;
    default:
      Serial.printf("ERROR invalid type '%c'", Cfg::param_list[i].type);
  }
}

static int _cfg_param_list_name_compare(const void *a, const void *b) {
  uint16_t i = *(uint16_t*)a;
  uint16_t j = *(uint16_t*)b;
  return strcmp( Cfg::param_list[i].name, Cfg::param_list[j].name );
}

//CLI dump: print all config values, sorted by name
void CfgClass::cli_dump(const char* filter, bool diff) {
  char filt[17] = {};
  if(filter) {
    strncpy(filt, filter, 16);
  }
  strlwr(filt);
  uint16_t arr[paramCount()];
  for(int i=0; i<paramCount(); i++) arr[i] = i;
  qsort(arr, paramCount(), 2, _cfg_param_list_name_compare);
  for(int j=0; j<paramCount(); j++) {
    uint16_t i = arr[j];
    if(strstr(Cfg::param_list[i].name, filt) && ( !diff || _get_param(i) != Cfg::param_list[i].defval )) {
      printNameAndValue(i);
    }
  }
}

//CLI diff: print all modified config values, sorted by name
void CfgClass::cli_diff(const char* filter) {
  cli_dump(filter, true);
}

//sort by pin number (using inefficient sort)
void CfgClass::printPins() {
  Serial.println("\n=== PINOUT ===\n");
  for(int pinno = 0; pinno < 128; pinno++) {
    int cnt = 0;
    for(int i = 0; i < paramCount(); i++) {
      if(strncmp(Cfg::param_list[i].name, "pin_", 4) == 0 && _get_param(i) == pinno) {
        if(cnt==0) {
          printNameAndValue(i);
        }else{
          printNameAndValue(i, "WARNING: Duplicate pin assignment");
        }
        cnt++;
      }
    }
  }
}

//CLI set a parameter value, returns true on success
bool CfgClass::cli_set_param(String name_filter, String val) {
  name_filter.trim();
  if(name_filter.length() < 3) return false;
  bool rv = true;
  char filt[17] = {};
  strncpy(filt, name_filter.c_str(), 16);
  strlwr(filt);
  uint16_t arr[paramCount()];
  for(int i=0; i<paramCount(); i++) arr[i] = i;
  qsort(arr, paramCount(), 2, _cfg_param_list_name_compare);
  for(int j=0; j<paramCount(); j++) {
    uint16_t i = arr[j];
    if(strstr(Cfg::param_list[i].name, filt)) {
      String namestr = Cfg::param_list[i].name;
      if(!_cli_set_param(namestr, val)) rv = false;
    }
  }
  cli_dump(name_filter.c_str());
  return rv;
}

bool CfgClass::_cli_set_param(String namestr, String val) {
  //Serial.printf("cfg.setParam %s %s\n", namestr.c_str(), val.c_str());
  namestr.trim();
  val.trim();
  if(namestr == "") return false;
  if(val == "") {
    Serial.printf("CFG: WARNING - No value for param '%s'\n", namestr.c_str());
    return false;
  }
  int i = getIndex(namestr);
  if(i < 0) {
    Serial.printf("CFG: WARNING - Param '%s' not found\n", namestr.c_str());
    return false;
  }
  switch(Cfg::param_list[i].type) {
    case 'e': { //enum
      int enum_idx = _get_enum_index(val.c_str(), Cfg::param_list[i].options);
      if(enum_idx >= 0) {
        _set_param(i, enum_idx);
        return true;
      }else{
        Serial.printf("CFG: WARNING - Param '%s' has no '%s' option. Available options: ", namestr.c_str(), val.c_str());
        _print_options(Cfg::param_list[i].options);
        Serial.println();
        return false;
      }
      break;
    }
    case 'f': //float
      _set_param(i, val.toFloat());
      break;
    case 'i': //integer
      _set_param(i, val.toInt());
      break;
    case 'p': //pinnumber/pinname
      _set_param(i, hal_get_pin_number(val));
      break;
  }
  return true;
}

//Set a parameter value, returns true on success
bool CfgClass::_set_param(int i, float val, bool publish) {
  if(i < 0 || i >= paramCount()) return false;

  CfgParam* param = (CfgParam*) this;
  float* param_float = (float*) param;
  int32_t* param_int32_t = (int32_t*) param;

  bool changed = false;
  switch(Cfg::param_list[i].type) {
    case 'f': //float
      changed = (param_float[i] != val);
      param_float[i] = val;
      break;
    case 'e': //enum
    case 'i': //integer
    case 'p': //pinnumber/pinname
      changed = (param_int32_t[i] != val);
      param_int32_t[i] = val;
      break;
    default:
      return false;
  }
  if(publish && changed) {
    pid.load_param(); //force PID to reload parameters
    bbx.log_parm(Cfg::param_list[i].name, val, Cfg::param_list[i].defval); //log parameter to BBX
  }
  return true;
}

//get parameter value as float
float CfgClass::_get_param(int i) {
  if(i < 0 || i >= paramCount()) return 0;

  CfgParam* param = (CfgParam*) this;
  float* param_float = (float*) param;
  int32_t* param_int32_t = (int32_t*) param;

  switch(Cfg::param_list[i].type) {
    case 'f': //float
      return param_float[i];
      break;
    case 'e': //enum
    case 'i': //integer
    case 'p': //pinnumber/pinname
      return (float)param_int32_t[i];
      break;
    default:
      return 0;
  }
}

//for mavlink
bool CfgClass::mavlink_set_param(String namestr, float val) {
  namestr.trim();
  if(namestr == "") return false;
  int i = getIndex(namestr);
  if(i < 0) return false;
  return _set_param(i, val);
}

//get parameter index for a parameter name
int CfgClass::getIndex(String namestr) {
  namestr.trim();
  namestr.toLowerCase();
  const char *name = namestr.c_str();
  for(uint16_t i=0;i<paramCount();i++) {
    if(strcmp(Cfg::param_list[i].name, name) == 0) {
      return i;
    }
  }
  return -1;
}

//load defaults
void CfgClass::cli_defaults(const char* filter) {
  if(!filter) {
    //clear all, including _is_board_and_config_loaded flag
    clear();
    _is_board_and_config_loaded = false; //not needed but kept for clarity
  }else{
    //clear filtered parameters, keep _is_board_and_config_loaded unchanged
    char filt[17] = {};
    if(filter) {
      strncpy(filt, filter, 16);
    }
    strlwr(filt);
    for(int i=0; i<paramCount(); i++) {
      if(strstr(Cfg::param_list[i].name, filt)) {
        _set_param(i, Cfg::param_list[i].defval, false /*no publish*/); 
      }
    }
  }
}

void CfgClass::cli_save() {
  Serial.println("Saving and rebooting, please wait... ");
  cfg.save();
  delay(1000);
  hal_reboot();
}

//write config to flash
void CfgClass::save() {
  uint32_t pos = 0;
  uint32_t crc = 0xFFFFFFFF;

  //setup header
  CfgHeader hdr = {}; //clear to 0
  hdr.header0 = CFG_HDR0;
  hdr.header1 = CFG_HDR1;
  hdr.header2 = CFG_HDR2;
  hdr.header3 = CFG_HDR3;
  hdr.len = sizeof(CfgHeader) + sizeof(CfgParam) + 4; //number of bytes for hdr+param+crc
  //hdr._reserved0 = 0;
  if(_is_board_and_config_loaded) hdr.board_and_config_crc = _calc_board_and_config_crc();
  //hdr._reserved1 = 0,0,0,...

  Serial.printf("CFG: Saving to EEPROM: board_and_config_crc=%X\n", hdr.board_and_config_crc);

  //write header
  for(uint32_t i=0; i<sizeof(CfgHeader); i++) {
    uint8_t byte = ((uint8_t*)&hdr)[i];
    hal_eeprom_write(pos, byte);
    crc = tbx_crc32(&byte, 1, crc);
    pos++;
  }

  //write param
  CfgParam *param = this;
  for(uint32_t i=0; i<sizeof(CfgParam); i++) {
    uint8_t byte = ((uint8_t*)param)[i];
    hal_eeprom_write(pos, byte);
    crc = tbx_crc32(&byte, 1, crc);
    pos++;
  }

  //write crc
  for(uint32_t i=0; i<sizeof(crc); i++) {
    hal_eeprom_write(pos, ((uint8_t*)&crc)[i]);
    pos++;
  }
  hal_eeprom_commit();
  Serial.println("CFG: EEPROM written");
}

void CfgClass::_load_from_string(const char *batch) {
  int pos = 0;
  int lineno = 0;
  String cmdline = "";
  while(1) {
    char c = batch[pos];
    //if(c) Serial.print(c);
    if ( c=='\r' || c=='\n' || c==0 ) { //end of line, or end of string
      lineno++;
      if(!_load_cmdline(cmdline)) {
        Serial.printf("=== ERROR while processing line number %d: %s\n", lineno, cmdline.c_str());
      }
      cmdline = "";
      if(c==0) return;
    }else{
      cmdline += c;
    }
    pos++;
  }
}

uint32_t CfgClass::_calc_board_and_config_crc() {
  uint32_t crc = 0xFFFFFFFF;
  if(_board && _board[0]) crc = tbx_crc32((const uint8_t*)_board, strlen(_board), crc);
  if(_config && _config[0]) crc = tbx_crc32((const uint8_t*)_config, strlen(_config), crc);
  if(crc == 0) crc = 1; //reserve 0 as "not in eeprom" flag
  return crc;
}

void CfgClass::clear() {
  for(int i=0; i<paramCount(); i++) {
    _set_param(i, Cfg::param_list[i].defval, false /*no publish*/);
  }
  _is_board_and_config_loaded = false;
}

//#define MF_DEBUG_CFG1
//#define MF_DEBUG_CFG2

void CfgClass::_load() {
  //====================================
  // Step 1: load defaults
  //====================================
  clear();

  #ifdef MF_DEBUG_CFG1
    #define MF_DEBUG_DUMP() cfg.cli_dump("rcl_gizmo")
    Serial.printf("\nDEBUG: After load_defaults()\n");
    MF_DEBUG_DUMP();
  #endif

  //====================================
  // Step 2: load eeprom (eeprom could be shorter than defaults)
  //====================================
  //load header into buffer
  CfgHeader eeprom_hdr = {};
  uint8_t *buf = (uint8_t*)&eeprom_hdr;
  for(uint32_t i = 0; i < sizeof(CfgHeader); i++) {
    buf[i] = hal_eeprom_read(i);
    //Serial.printf("%02X ",buf[i]);
  }

  //check header
  if(eeprom_hdr.header0 != CFG_HDR0 
  || eeprom_hdr.header1 != CFG_HDR1 
  || eeprom_hdr.header2 != CFG_HDR2 
  || eeprom_hdr.header3 != CFG_HDR3 
  || eeprom_hdr.len < sizeof(CfgHeader) + 8 
  || eeprom_hdr.len > 4096) {
    Serial.println("CFG: Loading EEPROM skipped (Header invalid)");
  }else{
    uint32_t eeprom_datalen = eeprom_hdr.len - 4; //length of header+param (4=crc)
    uint32_t eeprom_paramlen = eeprom_datalen - sizeof(CfgHeader); //length of param

    //check crc
    uint32_t crc = 0xFFFFFFFF;
    for(uint32_t i = 0; i < eeprom_datalen; i++) { 
      uint8_t byte = hal_eeprom_read(i);
      crc = tbx_crc32(&byte, 1, crc);
    }
    uint32_t crc_eeprom;
    uint8_t *crc_eeprom_buf = (uint8_t*)&crc_eeprom;
    for(uint32_t i = 0; i < 4; i++) { //4=crc
      crc_eeprom_buf[i] = hal_eeprom_read(eeprom_datalen + i);
    }
    if(crc != crc_eeprom) {
      Serial.print("CFG: Loading EEPROM skipped (CRC invalid)");
      eeprom_hdr.board_and_config_crc = 0;
    }else{
      //load param from eeprom
      CfgParam *param = this;
      uint8_t *param_buf = (uint8_t*)param;
      uint32_t num_bytes = sizeof(CfgParam);
      if(num_bytes > eeprom_paramlen) num_bytes = eeprom_paramlen; //minimum of CfgParam and eeprom bytes
      for(uint32_t i = 0; i<num_bytes; i++) {
        param_buf[i] = hal_eeprom_read(sizeof(CfgHeader) + i);
      }
      Serial.println("CFG: Loading EEPROM OK");
    }
  }

  #ifdef MF_DEBUG_CFG1
    Serial.printf("\nDEBUG: After _load_from_eeprom()\n");
    MF_DEBUG_DUMP();
  #endif

  //====================================
  // Step 3: load board+config (if not already applied to eeprom)
  //====================================
  //check board+config crc against board+config crc stored in eeprom
  uint32_t prog_crc = _calc_board_and_config_crc();
  Serial.printf("CFG: board_and_config_crc eeprom=%X program=%X\n", eeprom_hdr.board_and_config_crc, prog_crc);
  if(eeprom_hdr.board_and_config_crc == prog_crc) {
    //the board+config parameters were already applied (and potentially modified since, so do not re-apply)
    Serial.println("CFG: madflight_board + madflight_config skipped (EEPROM is newer)");
  }else{
    //load board + config
    if(_board && _board[0]) {
      Serial.println("CFG: Loading madflight_board");
      _load_from_string(_board);
    }
    if(_config && _config[0]) {
      Serial.println("CFG: Loading madflight_config");
      _load_from_string(_config);
    }
    Serial.println("CFG: madflight_board + madflight_config OK");

    #ifdef MF_DEBUG_CFG2
      // debug crc
      Serial.printf("\n");
      Serial.printf("eeprom_board_and_config_crc = %X\n", eeprom_board_and_config_crc);
      Serial.printf("madflight_board+config_crc  = %X\n", crc);
      Serial.printf("%s\n", (hdr.board_and_config_crc == crc ? "MATCHED" : "NOT MATCHED"));
      if(_board && _board[0]) {
        uint32_t crc = 0xFFFFFFFF;
        crc = tbx_crc32((const uint8_t*)_board, strlen(_board), crc);
        Serial.printf("_board: crc=%X len=%d start=\"", crc, strlen(_board));
        for(int i=0;i<40;i++) Serial.print(_board[i]);
        Serial.print("\"\n");
      }
      if(_config && _config[0]) {
        uint32_t crc = 0xFFFFFFFF;
        crc = tbx_crc32((const uint8_t*)_config, strlen(_config), crc);
        Serial.printf("config:crc=%X len=%d start=\"",crc, strlen(_config));
        for(int i=0;i<40;i++) Serial.print(_config[i]);
        Serial.print("\"\n");
      }
    #endif
    #ifdef MF_DEBUG_CFG1
      Serial.printf("\nDEBUG: final\n");
      MF_DEBUG_DUMP();
    #endif
  }

  _is_board_and_config_loaded = true;
}

//returns true on success
bool CfgClass::_load_cmdline(String cmdline) {
  //remove starting and ending whitespace
  cmdline.trim();
  
  //remove # comment
  int comment_pos = cmdline.indexOf('#');
  if(comment_pos >= 0) cmdline = cmdline.substring(0, comment_pos);

  //remove c-style // comment (anything after '/' for that matter)
  comment_pos = cmdline.indexOf('/');
  if(comment_pos >= 0) cmdline = cmdline.substring(0, comment_pos);

  //split name/value
  int space_pos = cmdline.indexOf(' ');
  String name = cmdline.substring(0, space_pos);
  String value = cmdline.substring(space_pos+1);

  //exit if no name given
  name.trim();
  if(name.length() == 0) return true;

  //process parameter (prints error message)
  return cli_set_param(name, value);
}

//get enum index from key string, return -1 if not found
int CfgClass::_get_enum_index(const char* key, const char* options) {
  String skey = key;
  skey.toUpperCase();
  skey = String("mf_") + skey;
  const char *k = skey.c_str();
  int klen = strlen(k);
  int len = strlen(options);
  int pos = 0;
  int i = 0;
  while(pos<len) {
    if(strncmp(options+pos,k,klen)==0 && (pos+klen>=len || options[pos+klen] == ',')) {
      return i;
    }
    i++;
    while(pos<len && options[pos]!=',') pos++;
    pos++; //skip comma
  }
  return -1;
}

//print options without "mf_" prefix
void CfgClass::_print_options(const char *str)
{
  const char *p = str;
  const char *p2;
  while(*p) {
    p2 = strstr(p, "mf_");
    if(!p2) {
      Serial.print(p);
      return;
    }
    for(const char *c = p; c < p2; c++) Serial.print(*c);
    p = p2 + 3;
  }
}