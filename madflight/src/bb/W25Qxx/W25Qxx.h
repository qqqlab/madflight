// madflight https://github.com/qqqlab/madflight
// Header only W25Q64/W25Q128 SPI Flash Memory class

// adapted from https://github.com/Tamakichi/Arduino-W25Q64

#pragma once

#include <SPI.h>

class W25Qxx {
  public:

enum {
  SPI_SLAVE_SEL_PIN =   10,
  MAX_BLOCKSIZE =       128,
  MAX_SECTORSIZE =      2048
};

enum {
  CMD_WRITE_STATUS_R =    0x01,
  CMD_PAGE_PROGRAM =      0x02,
  CMD_READ_DATA =         0x03,
  CMD_WRITE_DISABLE =     0x04,
  CMD_READ_STATUS_R1 =    0x05,
  CMD_WRIRE_ENABLE =      0x06,
  CMD_FAST_READ =         0x0B,
  CMD_SECTOR_ERASE =      0x20,
  CMD_QUAD_PAGE_PROGRAM = 0x32,
  CMD_READ_STATUS_R2 =    0x35,
  CMD_READ_DUAL_OUTPUT =  0x3B,
  CMD_READ_UNIQUE_ID =    0x4B,
  CMD_READ_QUAD_OUTPUT =  0x6B,
  CMD_BLOCK_ERASE32KB =   0x52,
  CMD_ERASE_SUPPEND =     0x75,
  CMD_ERASE_RESUME =      0x7A,
  CMD_MANUFACURER_ID =    0x90,
  CMD_JEDEC_ID =          0x9F,
  CMD_HIGH_PERFORM_MODE = 0xA3,
  CMD_RELEASE_PDOWN_ID =  0xAB,
  CMD_POWER_DOWN =        0xB9,
  CMD_READ_DUAL_IO =      0xBB,
  CMD_CHIP_ERASE =        0xC7,
  CMD_BLOCK_ERASE64KB =   0xD8,
  CMD_WORD_READ =         0xE3,
  CMD_READ_QUAD_IO =      0xEB,
  CMD_CNT_READ_MODE_RST = 0xFF
};

enum {
  SR1_BUSY_MASK	= 0x01,
  SR1_WEN_MASK	= 0x02
};

private:

uint8_t  _cspin ;
SPIClass*  _SPI = &SPI;
SPISettings  _SPISettings;

public:

void setSPIPort(SPIClass * SPI) {
   _SPI = SPI;
}
 
void begin(uint8_t cs, uint32_t frq) {
   _cspin = cs;
  pinMode( _cspin, OUTPUT); 
  deselect(); 
   _SPISettings = SPISettings(frq, MSBFIRST, SPI_MODE0);
   _SPI->begin();
}

void end() {
  powerDown();
  deselect();
   _SPI->end();
}

void select() {
   _SPI->beginTransaction( _SPISettings);
  digitalWrite( _cspin, LOW); 
}

void deselect() {
   digitalWrite( _cspin, HIGH); 
    _SPI->endTransaction();
}

uint8_t readStatusReg1() {
  uint8_t rc;
  select();
   _SPI->transfer(CMD_READ_STATUS_R1);
  rc =  _SPI->transfer(0xFF);
  deselect();
  return rc;
}

uint8_t readStatusReg2() {
  uint8_t rc;
  select();
   _SPI->transfer(CMD_READ_STATUS_R2);
  rc =  _SPI->transfer(0xFF);
  deselect();
  return rc;
}

void readManufacturer(uint8_t* d) {
  select();
   _SPI->transfer(CMD_JEDEC_ID);
  for (uint8_t i =0; i <3; i++) {
    d[i] =  _SPI->transfer(0x00);
  } 
  deselect();
}

int readSize() {
  uint8_t d[3];
  readManufacturer(d);
  return (2<<d[2]);
}

void readUniqieID(uint8_t* d) {
  select();
   _SPI->transfer(CMD_READ_UNIQUE_ID);
   _SPI->transfer(0x00);
   _SPI->transfer(0x00);
   _SPI->transfer(0x00);
   _SPI->transfer(0x00);
  for (uint8_t i =0; i <8; i++) {
    d[i] =  _SPI->transfer(0x00);
  }
 deselect(); 
}

bool IsBusy() {
  uint8_t r1;
  select();
   _SPI->transfer(CMD_READ_STATUS_R1);
  r1 =  _SPI->transfer(0xff);
  deselect();
  if(r1 & SR1_BUSY_MASK)
    return true;
  return false;
}

void powerDown() {
  select();
   _SPI->transfer(CMD_POWER_DOWN);
  deselect();
}

void writeEnable() {
  select();
   _SPI->transfer(CMD_WRIRE_ENABLE);
  deselect();
}

void writeDisable() {
  select();
   _SPI->transfer(CMD_WRITE_DISABLE);
  deselect();
}

uint16_t read(uint32_t addr, uint8_t *buf, uint16_t n){ 
  select();
   _SPI->transfer(CMD_READ_DATA);
   _SPI->transfer(addr>>16);
   _SPI->transfer((addr>>8) & 0xFF);
   _SPI->transfer(addr & 0xFF);
 
  uint16_t i;
  for(i = 0; i<n; i++ ) {
    buf[i] =  _SPI->transfer(0x00);
  }
  
  deselect();
  return i;
}

uint16_t fastread(uint32_t addr, uint8_t *buf, uint16_t n) {
  select();
   _SPI->transfer(CMD_FAST_READ);
   _SPI->transfer(addr>>16);
   _SPI->transfer((addr>>8) & 0xFF);
   _SPI->transfer(addr & 0xFF);
   _SPI->transfer(0x00);
  
  uint16_t i;
  for(i = 0; i<n; i++)
    buf[i] =  _SPI->transfer(0x00);
  
  deselect();  
  return i;
}

bool eraseSector(uint16_t sect_no, bool flgwait) {
  uint32_t addr = sect_no;
  addr<<=12;

  writeEnable();
  select(); 
   _SPI->transfer(CMD_SECTOR_ERASE);
   _SPI->transfer((addr>>16) & 0xff);
   _SPI->transfer((addr>>8) & 0xff);
   _SPI->transfer(addr & 0xff);
  deselect();

  while(IsBusy() & flgwait) {
    delay(1);
  }

  return true;
}

bool erase64Block(uint16_t blk_no, bool flgwait) {
  uint32_t addr = blk_no;
  addr<<=16;

  writeEnable();
  select(); 
   _SPI->transfer(CMD_BLOCK_ERASE64KB);
   _SPI->transfer((addr>>16) & 0xff);
   _SPI->transfer((addr>>8) & 0xff);
   _SPI->transfer(addr & 0xff);
  deselect();

  while(IsBusy() & flgwait) {
    delay(5);
  }

  return true;
}

bool erase32Block(uint16_t blk_no, bool flgwait) {

  uint32_t addr = blk_no;
  addr<<=15;

  writeEnable();  
  select(); 
   _SPI->transfer(CMD_BLOCK_ERASE32KB);
   _SPI->transfer((addr>>16) & 0xff);
   _SPI->transfer((addr>>8) & 0xff);
   _SPI->transfer(addr & 0xff);
  deselect();

  while(IsBusy() & flgwait) {
    delay(5);
 }
 
 return true;
}

bool eraseAll(bool flgwait) {
 writeEnable();  
 select(); 
  _SPI->transfer(CMD_CHIP_ERASE);
 deselect();

 while(IsBusy() & flgwait) {
    delay(50);
 }
 
 deselect();
 return true;
}

uint16_t pageWrite(uint32_t addr, uint8_t* data, uint16_t n, bool flgwait) {
  uint16_t i;
  writeEnable();  
  if (IsBusy()) {
    return 0;  
  }
  select();
  _SPI->transfer(CMD_PAGE_PROGRAM);
  _SPI->transfer((addr>>16) & 0xff);
  _SPI->transfer((addr>>8) & 0xff);
  _SPI->transfer(addr & 0xff);
  for (i=0; i < n; i++) {
     _SPI->transfer(data[i]);
  }  
  deselect();
  if(flgwait) while(IsBusy());
  return i;
}

};