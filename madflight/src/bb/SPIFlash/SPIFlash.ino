// The MIT License (MIT)
// Copyright (c) 2019 Ha Thach for Adafruit Industries

#include <SPI.h>

#include "src/Adafruit_SPIFlash.h"

// Un-comment to run example with custom SPI and SS e.g with FRAM breakout
// #define CUSTOM_CS   A5
// #define CUSTOM_SPI  SPI

#if defined(CUSTOM_CS) && defined(CUSTOM_SPI)
  Adafruit_FlashTransport_SPI flashTransport(CUSTOM_CS, CUSTOM_SPI);
#elif defined(ARDUINO_ARCH_ESP32)
  // ESP32 use same flash device that store code for file system.
  // SPIFlash will parse partition.cvs to detect FATFS/SPIFS partition to use
  Adafruit_FlashTransport_ESP32 flashTransport;
#elif defined(ARDUINO_ARCH_RP2040)
  // RP2040 use same flash device that store code for file system. Therefore we
  // only need to specify start address and size (no need SPI or SS)
  // By default (start=0, size=0), values that match file system setting in
  // 'Tools->Flash Size' menu selection will be used.
  Adafruit_FlashTransport_RP2040 flashTransport;
#else
  #error No (Q)SPI flash are defined for your board !
#endif

Adafruit_SPIFlash flash(&flashTransport);

#define BUFSIZE 256
#define TEST_WHOLE_CHIP 0

// 4 byte aligned buffer has best result with nRF QSPI
uint8_t bufwrite[BUFSIZE] __attribute__((aligned(4)));
uint8_t bufread[BUFSIZE] __attribute__((aligned(4)));

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(100); // wait for native usb
  }
  flash.begin();

  Serial.println("Adafruit Serial Flash Speed Test example");
  Serial.print("JEDEC ID: ");
  Serial.println(flash.getJEDECID(), HEX);
  Serial.print("Flash size: ");
  Serial.println(flash.size());
  Serial.println();

  //write_and_compare(0xAA);
  //write_and_compare(0x55);
  //test_overwrite();
  //erase_chip();
  //write_pattern(0,25600,0x55);
  findStart();
  findStartPage();
  test_readall();

  Serial.println("Speed test is completed.");
  Serial.flush();
}

void loop() {
  // nothing to do
}

bool write_and_compare(uint8_t pattern) {
  uint32_t dt;

#if TEST_WHOLE_CHIP
  uint32_t const flash_sz = flash.size();
  Serial.printf("Erase chip %d bytes\n",flash_sz);  
  dt = micros();
  flash.eraseChip();
  flash.waitUntilReady();
  dt = micros() - dt;
  Serial.printf("Erase chip %d bytes dt=%d\n",flash_sz,dt);  
#else
  uint32_t const flash_sz = 2*4096;
  dt = micros();
  flash.eraseSector(0);
  flash.eraseSector(1);  
  flash.waitUntilReady();
  dt = micros() - dt;
  Serial.printf("Erase %d bytes dt=%d\n",flash_sz,dt);  
#endif

  // write all
  memset(bufwrite, (int)pattern, BUFSIZE);

  Serial.printf("Writing 0x%02X\n", (int)pattern);
  for (uint32_t addr = 0; addr < flash_sz; addr += BUFSIZE) {
    dt = micros();    
    flash.writeBuffer(addr, bufwrite, BUFSIZE);
    dt = micros() - dt;
    Serial.printf("Write adr=%d len=%d dt=%dus\n", addr, BUFSIZE, dt);
  }

  // read and compare
  Serial.println("Read flash and compare");
  for (uint32_t addr = 0; addr < flash_sz; addr += BUFSIZE) {
    memset(bufread, 0, BUFSIZE);

    dt = micros();
    flash.readBuffer(addr, bufread, BUFSIZE);
    dt = micros() - dt;
    Serial.printf("Read adr=%d len=%d dt=%dus\n", addr, BUFSIZE, dt);

    if (memcmp(bufwrite, bufread, BUFSIZE)) {
      Serial.print("Error: flash contents mismatched at address 0x");
      Serial.println(addr, HEX);
      for (uint32_t i = 0; i < sizeof(bufread); i++) {
        if (i != 0)
          Serial.print(' ');
        if ((i % 16 == 0)) {
          Serial.println();
          if (i < 0x100)
            Serial.print('0');
          if (i < 0x010)
            Serial.print('0');
          Serial.print(i, HEX);
          Serial.print(": ");
        }

        if (bufread[i] < 0x10)
          Serial.print('0');
        Serial.print(bufread[i], HEX);
      }

      Serial.println();
      return false;
    }
  }
  return true;
}

void print_read(String msg) {
  memset(bufread, 0, BUFSIZE);
  flash.readBuffer(0, bufread, BUFSIZE);
  Serial.print(msg);
  for(int i=0;i<8;i++) Serial.printf(" %02X",bufread[i]);
  Serial.println();
}

void test_overwrite() {
  flash.eraseSector(0);
  flash.waitUntilReady();
  print_read("ERASE:         ");

  memset(bufwrite, 0xff, sizeof(bufwrite));
  bufwrite[0] = 0xF1;
  bufwrite[1] = 0xF2;  
  flash.writeBuffer(0, bufwrite, BUFSIZE);
  print_read("WRITE F1 F2:   ");

  memset(bufwrite, 0xff, sizeof(bufwrite));
  bufwrite[0] = 0xF1;
  bufwrite[1] = 0x2F;
  bufwrite[2] = 0xF3; 
  flash.writeBuffer(0, bufwrite, BUFSIZE);
  print_read("WRITE F1 2F F3:");
}

void test_readall() {
  uint32_t const flash_sz = flash.size();
  int cnt = 0;
  int cnt_empty = 0;
  uint32_t dt = micros();  
  for (uint32_t addr = 0; addr < flash_sz; addr += BUFSIZE) {
    flash.readBuffer(addr, bufread, BUFSIZE);
    if(bufread[0]==0xff) cnt_empty++;
    cnt++;
  }
  dt = micros() - dt;
  Serial.printf("Read len=%d dt=%dus %dKB/s cnt=%d cnt_empty=%d cnt_full=%d\n", flash_sz, dt, flash_sz/(dt/1000), cnt, cnt_empty,  cnt-cnt_empty);  
}

void erase_chip() {
  uint32_t const flash_sz = flash.size();
  Serial.printf("Erase chip %d bytes\n",flash_sz);  
  uint32_t dt = micros();
  flash.eraseChip();
  flash.waitUntilReady();
  dt = micros() - dt;
  Serial.printf("Erase chip %d bytes dt=%dus %dKB/s \n", flash_sz, dt, flash_sz/(dt/1000)); 
}

void write_pattern(uint32_t addr, uint32_t len, uint8_t pattern) {
  memset(bufwrite, (int)pattern, BUFSIZE);

  Serial.printf("Writing addr=%d len=%d pattern=0x%02X\n", (int)addr, (int)len, (int)pattern);
  uint32_t dt = micros();  
  for (uint32_t addr = 0; addr < len; addr += BUFSIZE) {
    flash.writeBuffer(addr, bufwrite, BUFSIZE);
  }
  dt = micros() - dt;
  Serial.printf("Write adr=%d len=%d dt=%dus\n", addr, len, dt);  
}

void findStart() {
  const uint32_t readlen = 32;
  uint32_t dt = micros(); 
  uint32_t len = flash.size();
  uint32_t adr_max = len - readlen;
  uint32_t adr_min = 0;
  uint8_t buf[readlen];
  while(adr_max != adr_min + 1) {
    uint32_t adr = adr_min + (adr_max - adr_min) / 2;
    flash.readBuffer(adr, buf, readlen);
    //Serial.printf("adr_min=%8d adr_max=%8d adr=%8d ", adr_min, adr_max, adr);
    //for(int i=0;i<16;i++) Serial.printf(" %02X",buf[i]);
    //Serial.println();
    for(int i=0;i<readlen;i++) if(buf[i]!=0xff) {
      adr_min = adr;
      break;
    }
    if(adr_min != adr) adr_max = adr;
  }
  dt = micros() - dt;
  Serial.printf("findStart=%d dt=%dus\n", adr_max, dt);
  //w25qxx_adr = ((w25qxx_adr / BB_BUF_SIZE) + 1) * BB_BUF_SIZE; //move to next BB_BUF_SIZE boundary
  //w25qxx_startadr = w25qxx_adr;
}


void findStartPage() {
  const uint32_t readlen = 256;
  const uint32_t pagelen = 256;  
  uint32_t dt = micros(); 
  uint32_t len = flash.size()/pagelen;
  uint32_t page_max = len - 1;
  uint32_t page_min = 0;
  uint8_t buf[readlen];
  while(page_max != page_min + 1) {
    uint32_t page = page_min + (page_max - page_min) / 2;
    flash.readBuffer(page*pagelen, buf, readlen);
    //Serial.printf("page_min=%8d page_max=%8d page=%8d ", page_min, page_max, page);
    //for(int i=0;i<16;i++) Serial.printf(" %02X",buf[i]);
    //Serial.println();
    for(int i=0;i<readlen;i++) if(buf[i]!=0xff) {
      page_min = page;
      break;
    }
    if(page_min != page) page_max = page;
  }
  dt = micros() - dt;
  Serial.printf("findStartPage=%d dt=%dus\n", page_max, dt);
  //w25qxx_adr = ((w25qxx_adr / BB_BUF_SIZE) + 1) * BB_BUF_SIZE; //move to next BB_BUF_SIZE boundary
  //w25qxx_startadr = w25qxx_adr;
}
