/*==============================================================================================
Black Box Data Logger
==============================================================================================*/

#pragma once

#include "BlackBox_Defines.h"

class BlackBoxWriter {

private:

BlackBoxFS *fs;

uint8_t logStarted = false; //enable logging
bool busy = false;

//vars for headers
uint8_t startWritten = false;
uint8_t writingHeaders = false;
uint8_t currentRecType;
uint8_t currentFieldIndex;
bool recTypeUsed[BB_REC_TYPE_SIZE];

public:

void setup( BlackBoxFS *fs ) {
  this->fs = fs;

  logStarted = false;
  startWritten = false;
  writingHeaders = false;
  busy = false;
  currentRecType = BB_REC_TYPE_SIZE;
  currentFieldIndex = 0;
  for(int i=0;i<BB_REC_TYPE_SIZE;i++) recTypeUsed[i] = false;
}

void writeHeaders() {
  logStarted = false;
  for(int i=0;i<BB_REC_TYPE_SIZE;i++) recTypeUsed[i] = false;
  startWritten = false;
  writingHeaders = true;
  busy = false;
}

void startLogging() {
  logStarted = true;
  busy = false;
}

void stopLogging() {
  logStarted = false;
}

bool isBusy() {
  //Note: this should be atomic test-and-set, but this will hopefully prevent concurrent writes better than doing nothing
  if(busy) return true;
  busy = true;
  return false;
}

//-------------------------------------------------
//write record functions
//-------------------------------------------------
void writeBeginRecord(uint8_t type, const char* name) {
  if(logStarted) {
    fs->writeChar(type);
  }else{
    writeHeaderRec(type, name);
  }
}

void writeFloat(String name, float value) {
  if(logStarted) {
    logFloat(value);
  }else{
    writeHeaderField(name, BB_DATATYPE_FLOAT);
  }
}

void writeU32(String name, uint32_t value) {
  if(logStarted) {
    logU32(value);
  }else{
    writeHeaderField(name, BB_DATATYPE_U32);
  }
}

void writeI32(String name, int32_t value) {
  if(logStarted) {
    logU32((uint32_t)value);
  }else{
    writeHeaderField(name, BB_DATATYPE_I32);
  }
}

void writeU16(String name, uint16_t value) {
  if(logStarted) {
    logU16(value);
  }else{
    writeHeaderField(name, BB_DATATYPE_U16);
  }
}

void writeI16(String name, int16_t value) {
  if(logStarted) {
    logU16((uint32_t)value);
  }else{
    writeHeaderField(name, BB_DATATYPE_I16);
  }
}

void writeU(String name, uint32_t value) {
  if(logStarted) {
    logU(value);
  }else{
    writeHeaderField(name, BB_DATATYPE_UVB);
  }
}

void writeI(String name, int32_t value) {
  if(logStarted) {
    logI((uint32_t)value); 
  }else{
    writeHeaderField(name, BB_DATATYPE_SVB);
  }
}

void writeEndrecord() {
  busy = false;
  if(logStarted) {
    fs->writeChar(BB_ENDRECORD);
  }else if(currentRecType<BB_REC_TYPE_SIZE && !recTypeUsed[currentRecType]) {
    recTypeUsed[currentRecType] = true;
  } 
}

void writeHeaderRec(uint8_t type, const char* name) {
  if(writingHeaders && type<BB_REC_TYPE_SIZE && !recTypeUsed[type]) {
    if(!startWritten) {
      logString(BB_STARTLOG);
      startWritten = true;
    } 
    fs->writeChar(BB_HDR_REC_TYPE);
    fs->writeChar(BB_HDRTYPE_REC);
    fs->writeChar(type);
    logString(name);
    fs->writeChar(BB_ENDRECORD);
    currentRecType = type;
    currentFieldIndex = 0;
  }else {
    currentRecType = BB_REC_TYPE_SIZE;
  }
}

void writeHeaderField(String name, bb_datatype_e datatype) {
  if(writingHeaders && currentRecType<BB_REC_TYPE_SIZE && !recTypeUsed[currentRecType]) {
    fs->writeChar(BB_HDR_REC_TYPE);
    fs->writeChar(BB_HDRTYPE_FIELD);
    fs->writeChar(currentRecType);
    fs->writeChar(currentFieldIndex);
    fs->writeChar(datatype);
    logString(name);
    fs->writeChar(BB_ENDRECORD);
    currentFieldIndex++;
  }
}


//-------------------------------------------------
//log to output device functions
//-------------------------------------------------
void logString(String s) {
  for(uint32_t i=0;i<s.length();i++) fs->writeChar(s[i]);
}

void logU32(uint32_t value) {
  uint8_t *buf = (uint8_t*)&value;
  fs->writeChar(buf[0]);
  fs->writeChar(buf[1]);
  fs->writeChar(buf[2]);
  fs->writeChar(buf[3]);
}

void logU16(uint16_t value) {
  uint8_t *buf = (uint8_t*)&value;
  fs->writeChar(buf[0]);
  fs->writeChar(buf[1]);
}

void logFloat(float value) {
  uint8_t *buf = (uint8_t*)&value;
  fs->writeChar(buf[0]);
  fs->writeChar(buf[1]);
  fs->writeChar(buf[2]);
  fs->writeChar(buf[3]);
}

//Write an unsigned integer to the blackbox using variable byte encoding.
void logU(uint32_t value)
{
    //While this isn't the final byte (we can only write 7 bits at a time)
    while (value > 127) {
        fs->writeChar((uint8_t) (value | 0x80)); // Set the high bit to mean "more bytes follow"
        value >>= 7;
    }
    fs->writeChar(value);
}


//Write a signed integer to the blackbox using ZigZig and variable byte encoding.
void logI(int32_t value)
{
    //ZigZag encode to make the value always positive
    logU(zigzagEncode(value));
}

/**
 * ZigZag encoding maps all values of a signed integer into those of an unsigned integer in such
 * a way that numbers of small absolute value correspond to small integers in the result.
 *
 * (Compared to just casting a signed to an unsigned which creates huge resulting numbers for
 * small negative integers).
 */
uint32_t zigzagEncode(int32_t value)
{
    return (uint32_t)((value << 1) ^ (value >> 31));
}


};