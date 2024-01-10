#include "BlackBox.h"
#include "BlackBoxDecoder.h"

BlackBox bb;

enum {
  BB_REC_PID,
  BB_REC_GPS
};

int bb_logCharSerial_cnt = 0;
void bb_logCharSerial(uint8_t c) {
  Serial.printf("%02X'%c' ",c,(c>=32&&c<128 ? c : '.'));
  bb_logCharSerial_cnt++;
  if(bb_logCharSerial_cnt==16) {
    Serial.println();
    bb_logCharSerial_cnt=0;
  }
}

const int bb_buf_size = 60000;
uint8_t bb_buf[bb_buf_size];
int bb_buf_len = 0;
int bb_buf_idx = 0;

void bb_logCharMemory(uint8_t c) {
  if(bb_buf_len < bb_buf_size) {
    bb_buf[bb_buf_len++] = c;
  }
}

uint8_t bb_decode_getCharMemory() {
  if(bb_buf_idx < bb_buf_len) {
    return bb_buf[bb_buf_idx++];
  }else{
    return 0xff;
  }
}

void bb_decode_putChar(uint8_t c) {
  Serial.printf("%c",c);
}

void bb_imu() {
  bb.writeBeginRecord(BB_REC_PID);
  bb.writeUnsignedVB("ts",11);
  bb.writeSignedVB("ax",12);
  bb.writeSignedVB("ay",13);
  bb.writeSignedVB("az",14);    
  bb.writeEndrecord();
}

void bb_gps() {
  bb.writeBeginRecord(BB_REC_GPS);
  bb.writeSignedVB("ts",-21);  
  bb.writeSignedVB("lat",-2222);
  bb.writeFloat("lon",1.23e-2);   
  bb.writeEndrecord();
}

void bb_setup() {
  bb.begin(bb_logCharMemory);
  bb_imu();
  bb_gps();
  bb.start();
}

void setup() {
  Serial.begin(115200);


}

void loop() {
  Serial.println();
  Serial.println("----------------------------");

  bb_logCharSerial_cnt = 0;
  bb_buf_len = 0;

  bb_setup();

  for(int i=0;i<5;i++) {
    bb_imu();
    bb_imu();    
    bb_gps(); 
  }

  int ii = 0;
  for(int i=0;i<bb_buf_len;i++) {
    uint8_t c = bb_buf[i];
    Serial.printf("%02X ",c);
    if(c == BB_ENDRECORD) {
      Serial.print("  ");
      for(int j=ii;j<i;j++) {
        c = bb_buf[j];
        Serial.printf("%c",(c>=32&&c<128 ? c : '.'));
      }
      Serial.println();
      ii = i+1;
    }
  }

  Serial.println();
  Serial.printf("bb_buf_len=%d\n",bb_buf_len);

  bb_buf_idx = 0; 
  BlackBoxDecoder bbd;
  bbd.csv_decode(bb_decode_getCharMemory, bb_decode_putChar);

  delay(1000);  
}