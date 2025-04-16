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

//create global module instance
CfgClass cfg;

CfgClass::CfgClass() {}

void CfgClass::begin() {
  hal_eeprom_begin();
}

//get number of parameters
uint16_t CfgClass::paramCount() {
  return Cfg::param_cnt;
}

//get parameter name and value for index
bool CfgClass::getNameAndValue(uint16_t index, String* name, float* value) {
  if(index>=paramCount()) return false;
  *name = Cfg::param_list[index].name;
  if(Cfg::param_list[index].type =='f') {
    *value = param_float[index]; 
  }else{
    int32_t v = param_int32_t[index];
    *value = v;
  }
  return true;
}

//get parameter value as float
float CfgClass::getValue(String namestr, float default_value) {
  int i = getIndex(namestr);
  if(i<0) return default_value;
  if(Cfg::param_list[i].type == 'f') {
    return param_float[i];
  }else{
    return  param_int32_t[i];
  }
}

//print enum option name for parameter pointer
void CfgClass::printParamOption(const int32_t* param) {
  uint32_t i = ( (uint32_t)param - (uint32_t)(param_int32_t) ) / 4;
  if(i < paramCount()) {
    char option[20];
    if(getOptionString(i, *param, option)) {
      Serial.print(option);
      return;
    }
  }
  Serial.print("***INVALID***");
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
void CfgClass::printModule(const char* module_name) {
  String n = String(module_name);
  n.toUpperCase();
  Serial.printf("%s: ", n.c_str());
  n.toLowerCase();
  n += '_';
  
  String type_name = n + "gizmo";
  int type_i = getIndex(type_name);
  if(type_i >= 0) {
    printValue(type_i);
    Serial.print(" - ");
  }
  for(int i=0;i<paramCount();i++) {
    if(strncmp(Cfg::param_list[i].name, n.c_str(), n.length()) == 0 && i != type_i) { //starts with module_name + '_'
      Serial.print(Cfg::param_list[i].name + n.length()); //remove module_name + '_'
      Serial.print(':');
      printValue(i);
      Serial.print(' ');
    }
  }
  Serial.println();
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
    print_options(options);
  }
  Serial.println();
}

//print param value
void CfgClass::printValue(uint16_t i) {
  if(i >= paramCount()) return;
  switch(Cfg::param_list[i].type) {
    case 'e': { //enum
      char option[20];
      if(getOptionString(i, param_int32_t[i], option)) {
        Serial.print(option);
      }else{
        Serial.printf("%d", (int)param_int32_t[i]); //option lookup failed, print numeric value
      }
      break;
    }
    case 'f': //float
      Serial.printf("%f", param_float[i]);
      break;
    case 'i': //integer
      Serial.printf("%d", (int)param_int32_t[i]);
      break;
    case 'p': //pinnumber/pinname
      hal_print_pin_name(param_int32_t[i]);
      break;
    default:
      Serial.printf("ERROR invalid type '%c'", Cfg::param_list[i].type);
  }
}

//CLI print all config values
void CfgClass::list(const char* filter) {
  for(int i=0;i<paramCount();i++) {
    if(strstr(Cfg::param_list[i].name, filter)) {
      printNameAndValue(i);
    }
  }
}

/* non-sorted version
void CfgClass::printPins() {
  for(int i=0;i<paramCount();i++) {
    if(strncmp(Cfg::param_list[i].name, "pin_", 4) == 0 && param_int32_t[i] >= 0) {
      Serial.print("PIN: ");
      printNameAndValue(i);
    }
  }
}
*/

//(primitive) sort by pin number
void CfgClass::printPins() {
  for(int pinno = 0; pinno<128; pinno++) {
    int cnt = 0;
    for(int i=0;i<paramCount();i++) {
      if(strncmp(Cfg::param_list[i].name, "pin_", 4) == 0 && param_int32_t[i] == pinno) {
        Serial.print("PIN: ");
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
bool CfgClass::setParam(String namestr, String val) {
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
      int enum_idx = get_enum_index(val.c_str(), Cfg::param_list[i].options);
      if(enum_idx >= 0) {
        param_int32_t[i] = enum_idx;
        return true;
      }else{
        Serial.printf("CFG: WARNING - Param '%s' has no '%s' option. Available options: ", namestr.c_str(), val.c_str());
        print_options(Cfg::param_list[i].options);
        Serial.println();
        return false;
      }
      break;
    }
    case 'f': //float
      param_float[i] = val.toFloat();
      break;
    case 'i': //integer
      param_int32_t[i] = val.toInt();
      break;
    case 'p': //pinnumber/pinname
      param_int32_t[i] = hal_get_pin_number(val);
      break;
  }
  return true;
}

//for mavlink
bool CfgClass::setParamMavlink(String namestr, float val) {
  namestr.trim();
  if(namestr == "") return false;
  int i = getIndex(namestr);
  if(i < 0) return false;
  if(Cfg::param_list[i].type == 'f') {
    param_float[i] = val;
  }else{
    param_int32_t[i] = val;
  }
  return true;
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
void CfgClass::clear() {
  CfgParam cfg2;
  CfgParam *param = this;
  memcpy(param, &cfg2, sizeof(CfgParam));
}

//read parameters from eeprom/flash
void CfgClass::loadFromEeprom() {
  //Serial.printf("mf=%d all=%d i[17]=%d\n", Cfg::mf_param_cnt, Cfg::param_cnt, param_int32_t[16]);
  Serial.print("CFG: Loading EEPROM - ");

  cfg.clear();

  //load header into buffer
  CfgHeader hdr_new;
  uint8_t *buf = (uint8_t*)&hdr_new;
  for(uint32_t i = 0; i < sizeof(CfgHeader); i++) {
    buf[i] = hal_eeprom_read(i);
    //Serial.printf("%02X ",buf[i]);
  }

  //check header
  if(hdr_new.header0 != CFG_HDR0 
  || hdr_new.header1 != CFG_HDR1 
  || hdr_new.header2 != CFG_HDR2 
  || hdr_new.header3 != CFG_HDR3 
  || hdr_new.len<sizeof(CfgHeader)+8 
  || hdr_new.len>4096) {
    Serial.println("Header invalid, using defaults");
    return;
  }
  uint32_t datalen = hdr_new.len - 4; //length of header+param (4=crc)
  uint32_t paramlen = datalen - sizeof(CfgHeader); //length of param

  //check crc
  uint32_t crc = 0xFFFFFFFF;
  for(uint32_t i = 0; i < datalen; i++) { 
    uint8_t byte = hal_eeprom_read(i);
    crc = tbx_crc32(&byte, 1, crc);
  }
  uint32_t crc_eeprom;
  uint8_t *crc_eeprom_buf = (uint8_t*)&crc_eeprom;
  for(uint32_t i = 0; i < 4; i++) { //4=crc
    crc_eeprom_buf[i] = hal_eeprom_read(datalen + i);
  }
  if(crc != crc_eeprom) {
    Serial.println("CRC invalid, using defaults");
    return;
  }

  //load header from eeprom
  memcpy(&hdr, &hdr_new, sizeof(CfgHeader));

  //load param from eeprom
  CfgParam *param = this;
  uint8_t *param_buf = (uint8_t*)param;
  uint32_t num_bytes = sizeof(CfgParam);
  if(num_bytes > paramlen) num_bytes = paramlen; //minimum of CfgParam and eeprom bytes
  for(uint32_t i = 0; i<num_bytes; i++) {
    param_buf[i] = hal_eeprom_read(sizeof(CfgHeader) + i);
  }

  Serial.println("OK");
}

//write config to flash
void CfgClass::writeToEeprom() {
  uint32_t pos = 0;
  uint32_t crc = 0xFFFFFFFF;

  //setup header
  hdr.header0 = CFG_HDR0;
  hdr.header1 = CFG_HDR1;
  hdr.header2 = CFG_HDR2;
  hdr.header3 = CFG_HDR3;
  hdr.len = sizeof(CfgHeader) + sizeof(CfgParam) + 4; //number of bytes for hdr+param+crc
  hdr._reserved0 = 0;
  //hdr.madflight_param_crc; //this was set in load_madflight_param()
  //hdr._reserved1;

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

void CfgClass::loadFromString(const char *batch) {
  int pos = 0;
  int lineno = 0;
  String cmdline = "";
  while(1) {
    char c = batch[pos];
    //if(c) Serial.print(c);
    if ( c=='\r' || c=='\n' || c==0 ) { //end of line, or end of string
      lineno++;
      if(!load_cmdline(cmdline)) {
        Serial.printf(" while processing line number %d: %s\n", lineno, cmdline.c_str());
      }
      cmdline = "";
      if(c==0) return;
    }else{
      cmdline += c;
    }
    pos++;
  }
}

void CfgClass::load_madflight(const char *board, const char *config) {
  Serial.print("CFG: Loading config - ");

  //calc crc
  uint32_t crc = 0xFFFFFFFF;
  if(board && board[0]) crc = tbx_crc32((const uint8_t*)board, strlen(board), crc);
  if(config && config[0]) crc = tbx_crc32((const uint8_t*)config, strlen(config), crc);

  //check board+config crc against board+config crc stored in eeprom
  if(hdr.madflight_param_crc == crc) {
    //the board+config parameters were already applied (and potentially modified since, so do not re-apply)
    Serial.println("Skipping madflight_board + madflight_config (EEPROM is newer)");
    return;
  }

  //load board + config
  if(board && board[0]) {
    loadFromString(board);
    Serial.print("madflight_board OK, ");
  }else{
    Serial.print("madflight_board is empty, ");
  }
  if(config && config[0]) {
    loadFromString(config);
    Serial.println("madflight_config OK");
  }else{
    Serial.println("madflight_config is empty");
  }
  hdr.madflight_param_crc = crc; //save crc (and write it to eeprom with next CLI cwrite)
}

//returns true on success
bool CfgClass::load_cmdline(String cmdline) {
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
  return setParam(name, value);
}


//get enum index from key string, return -1 if not found
int CfgClass::get_enum_index(const char* key, const char* options) {
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
void CfgClass::print_options(const char *str)
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