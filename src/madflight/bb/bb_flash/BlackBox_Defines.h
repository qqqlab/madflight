/*==============================================================================================
Header Record:    BB_HDR_REC_TYPE BB_HDRTYPE_xxx data[] BB_ENDRECORD
Record Name:      BB_HDR_REC_TYPE BB_HDRTYPE_REC recordType name[] BB_ENDRECORD
Field Definition: BB_HDR_REC_TYPE BB_HDRTYPE_FIELD recordType fieldIndex dataType name[] BB_ENDRECORD
Data Record:      recordType data[] BB_ENDRECORD

example file:

qqqlab/BlackBox1                    -- file header
32 R P PID \n                       -- record: type='A' name="ACCEL"
32 F P 0 U ts \n                    -- field: type='A' index=0 datatype=uint32_t name="ts"
32 F P 1 F ax \n                    -- field: type='A' index=1 datatype=float name="ax"
32 F P 2 F ay \n                    -- field: type='A' index=2 datatype=float name="ay"
32 F P 3 F az \n                    -- field: type='A' index=3 datatype=float name="az"
32 R G GPS \n                       -- record: type='G' name="GPS"
32 F G 0 U ts \n                    -- field: type='G' index=0 datatype=uint32_t name="ts"
32 F G 1 I lat \n                   -- field: type='G' index=0 datatype=uint32_t name="lat"
32 F G 2 I lon \n                   -- field: type='G' index=0 datatype=uint32_t name="lon"
A <ts> <ax> <ay> <az> \n            -- ACCEL data: type='A' values=ts,ax,ay,az
A <ts> <ax> <ay> <az> \n            -- ACCEL data: type='A' values=ts,ax,ay,az
G <ts> <lat> <lon> \n               -- GPS data: type='G' values=ts,lat,lon
A <ts> <ax> <ay> <az>\n             -- ACCEL data: type='A' values=ts,ax,ay,az
A <ts> <ax> <ay> <az> \n            -- ACCEL data: type='A' values=ts,ax,ay,az
A <ts> <ax> <ay> <az> \n            -- ACCEL data: type='A' values=ts,ax,ay,az
G <ts> <lat> <lon> \n               -- GPS data: type='G' values=ts,lat,lon
==============================================================================================*/

#pragma once

#define BB_STARTLOG "qqqlabBlackBox1\n" //16 character log start marker
#define BB_REC_TYPE_SIZE 32 //max number of record types (plus one for BB_HDR_REC_TYPE)
#define BB_MAX_REC_CHARS 64 //maximum number characters in a record
#define BB_MAX_FIELD_COUNT 16 //maximum number of fields in a record
#define BB_MAX_COLUMN_COUNT 32 //maximum number of unique column names
#define BB_HDR_REC_TYPE BB_REC_TYPE_SIZE //record type for headers
#define BB_HDRTYPE_FIELD 'F'
#define BB_HDRTYPE_REC 'R'
#define BB_ENDRECORD '\n' //end record character

enum bb_datatype_e {
  BB_DATATYPE_FLOAT, //4 bytes
  BB_DATATYPE_U32, //4 bytes
  BB_DATATYPE_I32, //4 bytes
  BB_DATATYPE_U16, //2 bytes
  BB_DATATYPE_I16, //2 bytes
  BB_DATATYPE_UVB, //1-5 bytes
  BB_DATATYPE_SVB, //1-5 bytes
  BB_DATATYPE_COUNT
};

class BlackBoxFS {
public:
  virtual void setup() = 0; //setup the file system
  
  virtual void writeOpen() = 0; //create new file for writing
  virtual void writeChar(uint8_t c) = 0; //write char to file
  virtual void writeClose() = 0; //close file

  virtual void readOpen(int fileno); //open fileno for reading. Fileno 0:last file, -1:2nd last file, etc
  virtual uint8_t readChar() = 0; //read char from file

  virtual void erase() = 0; //erase all files
  virtual void dir() = 0; //list files
};