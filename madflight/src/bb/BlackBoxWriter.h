/*==============================================================================================
Black Box Data Logger
==============================================================================================*/

#pragma once

#include "BlackBox_Defines.h"

class BlackBoxWriter {

private:

bool logStarted = false;
bool startWritten = false;
void (*logChar)(uint8_t);
uint8_t currentRecType;
uint8_t currentFieldIndex;
bool recTypeUsed[BB_REC_TYPE_SIZE];

public:

void begin( void (*logChar)(uint8_t) ) {
  logStarted = false;
  startWritten = false;
  this->logChar = logChar;
  currentRecType = BB_REC_TYPE_SIZE;
  currentFieldIndex = 0;
  for(int i=0;i<BB_REC_TYPE_SIZE;i++) recTypeUsed[i] = false;
}

void start() {  
  logStarted = true;
}

void stop() {
  logStarted = false;
}

//-------------------------------------------------
//write record functions
//-------------------------------------------------
void writeBeginRecord(uint8_t type) {
  if(logStarted) {
    logChar(type);
  }else{
    if(type<BB_REC_TYPE_SIZE && !recTypeUsed[type]) {
      currentFieldIndex = 0;
      currentRecType = type;
    }else{
      currentRecType = BB_REC_TYPE_SIZE;
    }
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

void writeUnsignedVB(String name, uint32_t value) {
  if(logStarted) {
    logUnsignedVB(value); 
  }else{
    writeHeaderField(name, BB_DATATYPE_UVB);
  }
}

void writeSignedVB(String name, int32_t value) {
  if(logStarted) {
    logSignedVB((uint32_t)value); 
  }else{
    writeHeaderField(name, BB_DATATYPE_SVB);
  }
}

void writeHeaderField(String name, bb_datatype_e datatype) {
  if(currentRecType<BB_REC_TYPE_SIZE && !recTypeUsed[currentRecType]) {
    if(!startWritten) {
      logString(BB_STARTLOG);
      startWritten = true;
    } 
    logChar(BB_HDR_REC_TYPE);
    logChar(BB_HDRTYPE_FIELD);
    logChar(currentRecType);
    logChar(currentFieldIndex);
    logChar(datatype);      
    logString(name);
    logChar(BB_ENDRECORD);
    currentFieldIndex++;   
  }  
}

void writeEndrecord() {
  if(logStarted) {
    logChar(BB_ENDRECORD); 
  }else if(currentRecType<BB_REC_TYPE_SIZE && !recTypeUsed[currentRecType]) {
    recTypeUsed[currentRecType] = true;   
  } 
}

//-------------------------------------------------
//log to output device functions
//-------------------------------------------------
void logString(String s) {
  for(uint32_t i=0;i<s.length();i++) logChar(s[i]);
}

void logU32(uint32_t value) {
  uint8_t *buf = (uint8_t*)&value;
  logChar(buf[0]);
  logChar(buf[1]);
  logChar(buf[2]);
  logChar(buf[3]);
}

void logU16(uint16_t value) {
  uint8_t *buf = (uint8_t*)&value;
  logChar(buf[0]);
  logChar(buf[1]);
}

void logFloat(float value) {
  uint8_t *buf = (uint8_t*)&value;
  logChar(buf[0]);
  logChar(buf[1]);
  logChar(buf[2]);
  logChar(buf[3]);
}

//Write an unsigned integer to the blackbox using variable byte encoding.
void logUnsignedVB(uint32_t value)
{
    //While this isn't the final byte (we can only write 7 bits at a time)
    while (value > 127) {
        logChar((uint8_t) (value | 0x80)); // Set the high bit to mean "more bytes follow"
        value >>= 7;
    }
    logChar(value);
}


//Write a signed integer to the blackbox using ZigZig and variable byte encoding.
void logSignedVB(int32_t value)
{
    //ZigZag encode to make the value always positive
    logUnsignedVB(zigzagEncode(value));
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