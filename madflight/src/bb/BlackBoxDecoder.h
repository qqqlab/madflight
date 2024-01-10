/*==============================================================================================
Black Box Decoder
==============================================================================================*/

#pragma once

#include "BlackBox_Defines.h"

class BlackBoxDecoder {
public:

//Note: this function expects 0xff from getChar() when fetching past the end of buffer
void csv_decode(uint8_t (*getChar)(void), void (*putChar)(uint8_t)) {
  _getChar = getChar;
  _putChar = putChar;

  int ff_cnt = 0;
  bool lastCharWasEndRecord = true;
  bool csvHeaderWritten = false;
  while(true) {
    uint8_t c = getChar();

    //detect end of stream
    if(c == 0xff) {
      ff_cnt++;
      if(ff_cnt > BB_MAX_REC_CHARS) return;
    }else{
      ff_cnt = 0;
    }

    if(lastCharWasEndRecord) {
      //look for record type byte
      if(c == BB_HDR_REC_TYPE) {
        //got a header
        lastCharWasEndRecord = parseHeader();
      }else if(c<BB_REC_TYPE_SIZE) {
        //got a record
        if(!csvHeaderWritten) {
          putColumns(); //columns contain fieldnames at this point
          csvHeaderWritten = true;
        }
        lastCharWasEndRecord = parseRecord(c);
      }else{
        lastCharWasEndRecord = false;
      }
    }else{
      if(c == BB_ENDRECORD) {
        lastCharWasEndRecord = true;
      }
    }
  }
}

private:



uint8_t (*_getChar)(void);
void (*_putChar)(uint8_t);
uint8_t currentType = BB_HDR_REC_TYPE;
uint8_t currentFieldIdx = 0;
uint8_t columnCount = 0;
String column[BB_MAX_COLUMN_COUNT];

struct Field_s {
  uint8_t column = 0xff;
  bb_datatype_e datatype = BB_DATATYPE_COUNT;    
} fields[BB_REC_TYPE_SIZE][BB_MAX_FIELD_COUNT];

bool parseHeader() {
  uint8_t hdrType = _getChar();
  if(hdrType == BB_HDRTYPE_FIELD) { //BB_HDRTYPE_FIELD: recordtype, fieldindex, datatype, name
    //recordtype
    uint8_t recordtype = _getChar();
    if(recordtype >= BB_REC_TYPE_SIZE) return false;
    //fieldindex
    uint8_t fieldindex = _getChar();
    if(fieldindex >= BB_MAX_FIELD_COUNT) return false;
    //datatype
    uint8_t datatype = _getChar();
    if(datatype >= BB_DATATYPE_COUNT) return false;
    //name
    String name = "";
    uint8_t c;
    for(int i=0;i<BB_MAX_REC_CHARS;i++) {
      c = _getChar();
      if(c == BB_ENDRECORD) break;
      name = name + (char)c;
    }
    if(c != BB_ENDRECORD) return false;
    //process field 
    int col;
    for(col=0;col<columnCount;col++) {
      if(column[col] == name) break;
    }
    if(col == columnCount) {
      column[columnCount] = name;
      columnCount++;
    }
    fields[recordtype][fieldindex].column = col;
    fields[recordtype][fieldindex].datatype = (bb_datatype_e)datatype;
    //Serial.printf("\nFIELD:name=%s col=%d",name.c_str(),col);
    return true;
  }
  return false;
}

bool parseRecord(uint8_t type) {
  int fldidx = 0;
  while(fields[type][fldidx].column < BB_MAX_COLUMN_COUNT) {
    int col = fields[type][fldidx].column;
    bb_datatype_e datatype = fields[type][fldidx].datatype;
    switch(datatype) {
      case BB_DATATYPE_FLOAT: {
        float value = getFloat();
        column[col] = String(value);
        break;
      }
      case BB_DATATYPE_U32:{
        uint32_t value = getU32();
        column[col] = String(value);
        break;
      }
      case BB_DATATYPE_I32:{
        int32_t value = getI32();
        column[col] = String(value);
        break;
      }
      case BB_DATATYPE_U16:{
        uint16_t value = getU16();
        column[col] = String(value);
        break;
      }
      case BB_DATATYPE_I16:{
        int16_t value = getI16();
        column[col] = String(value);
        break;
      }
      case BB_DATATYPE_UVB:{
        uint32_t value = getUnsignedVB();
        column[col] = String(value);
        break;
      }
      case BB_DATATYPE_SVB:{
        int32_t value = getSignedVB();
        column[col] = String(value);
        break;
      }
      default:
        break;
    }
    fldidx++;
  }
  putColumns();
  return (_getChar() == BB_ENDRECORD);
}

//write columns to csv
void putColumns() {
  for(int i=0;i<columnCount;i++) {
    if(i!=0) _putChar('\t');
    for(uint32_t j=0;j<column[i].length();j++) _putChar(column[i][j]);
    column[i] = "";
  }
  _putChar('\n');  
}

//-------------------------------------------------
//get data from blackbox functions
//-------------------------------------------------
float getFloat() {
  uint8_t buf[4];
  buf[0] = _getChar();
  buf[1] = _getChar();
  buf[2] = _getChar();
  buf[3] = _getChar();
  return *((float*)buf);
}

float getU32() {
  uint8_t buf[4];
  buf[0] = _getChar();
  buf[1] = _getChar();
  buf[2] = _getChar();
  buf[3] = _getChar();
  return *((uint32_t*)buf);
}

float getI32() {
  uint8_t buf[4];
  buf[0] = _getChar();
  buf[1] = _getChar();
  buf[2] = _getChar();
  buf[3] = _getChar();
  return *((int32_t*)buf);
}

float getU16() {
  uint8_t buf[2];
  buf[0] = _getChar();
  buf[1] = _getChar();
  return *((uint16_t*)buf);
}

float getI16() {
  uint8_t buf[2];
  buf[0] = _getChar();
  buf[1] = _getChar();
  return *((int16_t*)buf);
}

//Get an unsigned integer from the blackbox using variable byte encoding.
uint32_t getUnsignedVB() {
  uint32_t c;
  uint8_t shift = 0;
  uint32_t value = 0;
  do {
    c = _getChar();
    value |= (c & 0x7f) << shift;
    shift += 7;
  } while( (c & 0x80) && (shift < 32) ); //process 5 bytes max
  return value;
}

//Get an signed integer from the blackbox using ZigZig and variable byte encoding.
int32_t getSignedVB() {
  return zigzagDecode( getUnsignedVB() );
}

/**
 * ZigZag encoding maps all values of a signed integer into those of an unsigned integer in such
 * a way that numbers of small absolute value correspond to small integers in the result.
 *
 * (Compared to just casting a signed to an unsigned which creates huge resulting numbers for
 * small negative integers).
 */
int32_t zigzagDecode(uint32_t value)
{
    return ( value & 1 ? -(value>>1)-1 : (value>>1) );
}

};
