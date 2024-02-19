/*==============================================================================================
Black Box Decoder
==============================================================================================*/

#pragma once

#include "BlackBox_Defines.h"

class BlackBoxDecoder {
public:

  bool addRecNoColumn = true; //add record number column
  char separator = ','; //column separator

  //Note: this function expects 0xff from getChar() when fetching past the end of buffer
  void csv_decode(BlackBoxFS *fs) {
    this->fs = fs;

    //read file start marker
    for(int i=0;i<16;i++){
      uint8_t c = fs->readChar();
      if(c != BB_STARTLOG[i]) {
        Serial.println("Invalid BlackBox file");
        return;
      }
    }
    
    recNo = 0;
    sl_cnt = 0;
    ff_cnt = 0;

    bool lastCharWasEndRecord = true;
    bool csvHeaderWritten = false;
    while(true) {
      if(eof) return; //exit on eof

      uint8_t c = readChar();

      if(lastCharWasEndRecord) {
        //look for record type byte
        if(c == BB_HDR_REC_TYPE) {
          //got a header
          lastCharWasEndRecord = parseHeader();
        }else if(c<BB_REC_TYPE_SIZE) {
          //got a record
          if(!csvHeaderWritten) {
            putColumns(0); //columns contain fieldnames at this point
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

  BlackBoxFS *fs;

  //uint8_t currentType = BB_HDR_REC_TYPE;
  //uint8_t currentFieldIdx = 0;
  uint8_t columnCount = 0;
  String column[BB_MAX_COLUMN_COUNT]; //CSV columns
  String recName[BB_MAX_FIELD_COUNT]; //record name for recordtype
  int recNo;

  //end of file detection
  bool eof = false;
  uint8_t sl_cnt = 0;
  uint8_t ff_cnt = 0;

  uint8_t readChar() {
    uint8_t c = fs->readChar();
    //detect end of stream
    if(c == 0xff) {
      ff_cnt++;
      if(ff_cnt > BB_MAX_REC_CHARS) {
        eof = true;
        ff_cnt = 0;
      }
    }else{
      ff_cnt = 0;
    }

    //detect start of next file
    if(c == BB_STARTLOG[sl_cnt]) {
      sl_cnt++;
      if(sl_cnt==16) {
        eof = true;
        sl_cnt = 0;
      }
    }else{
      sl_cnt=0;
    }
    
    return c;
  }

  void _putChar(uint8_t c) {
    Serial.printf("%c",c);
  }

  struct Field_s {
    uint8_t column = 0xff;
    bb_datatype_e datatype = BB_DATATYPE_COUNT;
  } fields[BB_REC_TYPE_SIZE][BB_MAX_FIELD_COUNT];

  bool parseHeader() {
    uint8_t hdrType = readChar();
    if (hdrType == BB_HDRTYPE_REC) { //BB_HDRTYPE_REC: recordtype, name
      //recordtype
      uint8_t recordtype = readChar();
      if(recordtype >= BB_REC_TYPE_SIZE) return false;
      //name
      String name = "";
      uint8_t c;
      for(int i=0;i<BB_MAX_REC_CHARS;i++) {
        c = readChar();
        if(c == BB_ENDRECORD) break;
        name = name + (char)c;
      }
      if(c != BB_ENDRECORD) return false;
      //store record info
      recName[recordtype] = name;
      return true;
    }else if(hdrType == BB_HDRTYPE_FIELD) { //BB_HDRTYPE_FIELD: recordtype, fieldindex, datatype, name
      //recordtype
      uint8_t recordtype = readChar();
      if(recordtype >= BB_REC_TYPE_SIZE) return false;
      //fieldindex
      uint8_t fieldindex = readChar();
      if(fieldindex >= BB_MAX_FIELD_COUNT) return false;
      //datatype
      uint8_t datatype = readChar();
      if(datatype >= BB_DATATYPE_COUNT) return false;
      //name
      String name = "";
      uint8_t c;
      for(int i=0;i<BB_MAX_REC_CHARS;i++) {
        c = readChar();
        if(c == BB_ENDRECORD) break;
        name = name + (char)c;
      }
      if(c != BB_ENDRECORD) return false;
      //store field info
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
    if(readChar() != BB_ENDRECORD) return false;
    if(eof) return false;
    putColumns(type);
    return true;
  }

  //write columns to csv
  void putColumns(uint8_t type) {
    //record number column
    if(addRecNoColumn) {
      if(recNo == 0) {
        putString("RecNo");
      }else{
        putString(String(recNo));
      }
      _putChar(separator);
    }

    //record type column
    if(recNo == 0) {
      putString("RecType");
    }else{
      putString(recName[type]);
    }
    _putChar(separator);

    //data columns
    for(int i=0;i<columnCount;i++) {
      if(i!=0) _putChar(separator);
      putString(column[i]);
      column[i] = "";
    }

    _putChar('\n');

    recNo++;
  }

  void putString(String s) {
    for(uint32_t i=0;i<s.length();i++) _putChar(s[i]);
  }

  //-------------------------------------------------
  //get data from blackbox functions
  //-------------------------------------------------
  float getFloat() {
    uint8_t buf[4];
    buf[0] = readChar();
    buf[1] = readChar();
    buf[2] = readChar();
    buf[3] = readChar();
    return *((float*)buf);
  }

  float getU32() {
    uint8_t buf[4];
    buf[0] = readChar();
    buf[1] = readChar();
    buf[2] = readChar();
    buf[3] = readChar();
    return *((uint32_t*)buf);
  }

  float getI32() {
    uint8_t buf[4];
    buf[0] = readChar();
    buf[1] = readChar();
    buf[2] = readChar();
    buf[3] = readChar();
    return *((int32_t*)buf);
  }

  float getU16() {
    uint8_t buf[2];
    buf[0] = readChar();
    buf[1] = readChar();
    return *((uint16_t*)buf);
  }

  float getI16() {
    uint8_t buf[2];
    buf[0] = readChar();
    buf[1] = readChar();
    return *((int16_t*)buf);
  }

  //Get an unsigned integer from the blackbox using variable byte encoding.
  uint32_t getUnsignedVB() {
    uint32_t c;
    uint8_t shift = 0;
    uint32_t value = 0;
    do {
      c = readChar();
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
