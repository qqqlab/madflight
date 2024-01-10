#include "W25Qxx.h"

#define HW_PIN_FLASH_CS    PB3
SPIClass  MySPI(PC12,PC11,PC10);
W25Qxx w25qxx;

void setup() {
    uint32_t t;
    byte buf[256];
    byte wdata[256];
    uint16_t n;

    for (int i=0; i<256; i++) {
      wdata[i] = i & 0x7f;
    }

    w25qxx.setSPIPort(&MySPI);
    w25qxx.begin(HW_PIN_FLASH_CS,18000000);

    Serial.begin(115200);
    
    //3 second startup delay
    for(int i=10;i>0;i--) { 
      Serial.printf("starting %d ...\n",i);
      delay(300);
    }

    w25qxx.readManufacturer(buf);
    Serial.print("JEDEC ID : ");
    for (byte i=0; i< 3; i++) {
      Serial.print(buf[i],HEX);
      Serial.print(" ");
    }
    Serial.println("");

    Serial.printf("Size: %d bytes\n", w25qxx.readSize());
    
    memset(buf,0,8);
    w25qxx.readUniqieID(buf);
    Serial.print("Unique ID : ");
    for (byte i=0; i< 8; i++) {
      Serial.print(buf[i],HEX);
      Serial.print(" ");
    }
    Serial.println("");
    
    //read 256 byte page
    memset(buf,0,256);
    t = micros();
    n = w25qxx.read(0, buf, 256);
    t = micros()-t;
    Serial.printf("dt=%d us - ", t);
    Serial.print("Read Data: n=");
    Serial.println(n,DEC);
    dump(buf,256);

    //fast read 256 byte page
    memset(buf,0,256);
    t = micros();    
    n = w25qxx.fastread(256, buf, 256);
    t = micros()-t;
    Serial.printf("dt=%d us - ", t);
    Serial.print("Fast Read Data: n=");
    Serial.println(n, DEC);
    dump(buf,256);

    //erase 4K sector
    t = micros();    
    n = w25qxx.eraseSector(0,true);
    t = micros()-t;
    Serial.printf("dt=%d us - ", t);
    Serial.print("Erase Sector(0): n=");
    Serial.println(n, DEC);
    memset(buf,0,256);
    n =  w25qxx.read(0, buf, 256);
    dump(buf,256);

    //write 16 bytes page
    t = micros();    
    n = w25qxx.pageWrite(10, wdata, 16, true);
    t = micros()-t;
    Serial.printf("dt=%d us - ", t);
    Serial.print("page_write(10,d,16): n=");
    Serial.println(n, DEC);
    memset(buf,0,256);
    n =  w25qxx.read(0,buf, 256);
    dump(buf,256);

    //write 256 byte page
    t = micros();    
    n = w25qxx.pageWrite(256, wdata, 256, true);
    t = micros()-t;
    Serial.printf("dt=%d us - ", t);
    Serial.print("page_write(256,d,256): n=");
    Serial.println(n, DEC);
    memset(buf,0,256);
    n =  w25qxx.read(256, buf, 256);
    dump(buf,256);

    buf[0] = w25qxx.readStatusReg1();
    Serial.print("Status Register-1: ");
    Serial.print(buf[0], BIN);
    Serial.println("");

    buf[0] = w25qxx.readStatusReg2();
    Serial.print("Status Register-2: ");
    Serial.print(buf[0], BIN);
    Serial.println("");
}

void loop() {

}

void dump(byte *dt, uint32_t n) {
  char buf[64];
  uint16_t clm = 0;
  byte data;
  byte sum;
  byte vsum[16];
  byte total =0;
  uint32_t saddr =0;
  uint32_t eaddr =n-1;

  Serial.println("----------------------------------------------------------");
  for (uint16_t i=0;i<16;i++) vsum[i]=0;
  for (uint32_t addr = saddr; addr <= eaddr; addr++) {
    data = dt[addr];
    if (clm == 0) {
      sum =0;
      sprintf(buf,"%05lx: ",addr);
      Serial.print(buf);
    }

    sum+=data;
    vsum[addr % 16]+=data;
    
    sprintf(buf,"%02x ",data);
    Serial.print(buf);
    clm++;
    if (clm == 16) {
      sprintf(buf,"|%02x ",sum);
      Serial.print(buf);      
      Serial.println("");
      clm = 0;
    }    
  }
  Serial.println("----------------------------------------------------------");
  Serial.print("       ");
  for (uint16_t i=0; i<16;i++) {
    total+=vsum[i];
    sprintf(buf,"%02x ",vsum[i]);
    Serial.print(buf);
  }
  sprintf(buf,"|%02x ",total);
  Serial.print(buf);
  Serial.println("");
  Serial.println("");
}
