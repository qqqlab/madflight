/*==============================================================================================
Header Record:    <BB_HDR_REC_TYPE><BB_HDRTYPE_xxx><data><BB_ENDRECORD>
Field Definition: <BB_HDR_REC_TYPE><BB_HDRTYPE_FIELD><recordType><fieldindex><datatype><name><BB_ENDRECORD>
Data Record:      <recordType><data><BB_ENDRECORD>

example file:

qqqlab/BlackBox1                    -- file header
.FP0Uts\n                           -- field PID 0 uint32_t "ts"
.FP1Fax\n                           -- field PID 1 float "ax"
.FP2Fay\n                           -- field PID 2 float "ay"
.FP3Faz\n                           -- field PID 3 float "az"
.FG0Uts\n                           -- field GPS 0 uint32_t "ts"
.FG1Ilat\n                          -- field GPS 1 int32_t "lat"
.FG2Ilon\n                          -- field GPS 2 int32_t "lon"
P<ts><ax><ay><az>\n                 -- PID record: ts, pidx, pidy, pidz 
P<ts><ax><ay><az>\n                 -- PID record: ts, pidx, pidy, pidz  
G<ts><lat><lon>\n                   -- GPS record: ts, lat, lon
P<ts><ax><ay><az>\n                 -- PID record: ts, pidx, pidy, pidz 
P<ts><ax><ay><az>\n                 -- PID record: ts, pidx, pidy, pidz  
G<ts><lat><lon>\n                   -- GPS record: ts, lat, lon
==============================================================================================*/

#pragma once

#define BB_STARTLOG "qqqlabBlackBox1\n" //16 character log start marker
#define BB_REC_TYPE_SIZE 32 //max number of record types (plus one for headers)
#define BB_MAX_REC_CHARS 64 //maximum number characters in a record
#define BB_MAX_FIELD_COUNT 16 //maximum number of fields in a record
#define BB_MAX_COLUMN_COUNT 32 //maximum number of unique column names
#define BB_HDR_REC_TYPE BB_REC_TYPE_SIZE //record type for headers
#define BB_HDRTYPE_FIELD 'F'
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
