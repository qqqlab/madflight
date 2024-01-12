/*========================================================================================================================
This file contains all necessary functions and code used for blackbox logging to avoid cluttering the main code

Each USE_BB_xxx section in this file defines:
 - bb.setup() - setup interface
 - bb.csvDump() - dump last log to serial debug port as csv (tab delimited text)
 - bb.erase() - erase flash

In addition there are these functions common for all blackbox variants:
 - bb.start() - start a logging session
 - bb.stop() - stop a logging session
 - bb.log_xxx() - call these functions to write to logger, modify/add functions as needed
========================================================================================================================*/

/*this section is commented out because this file is included last in madflight.ino
//global variables used by bb_xxx
extern float rcin_thro, rcin_roll, rcin_pitch, rcin_yaw; //rcin_thro 0(cutoff) to 1(full); rcin_roll, rcin_pitch, rcin_yaw -1(left,down) to 1(right,up) with 0 center stick
extern bool rcin_armed; //status of arm switch, true = armed
extern int rcin_aux; // six position switch connected to aux channel, values 0-5
extern bool out_armed; //motors will only run if this flag is true
extern float AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ;
extern float ahrs_roll, ahrs_pitch, ahrs_yaw; 
extern float roll_PID, pitch_PID, yaw_PID;
extern float out_command[HW_OUT_COUNT]; //Mixer outputs
extern GPS gps;
extern Battery bat;
extern Barometer baro;
*/

#define BB_USE_NONE 1
#define BB_USE_FLASH 2
#define BB_USE_MEMORY 3

#include "BlackBoxWriter.h"
#include "BlackBoxDecoder.h"

class BlackBox {

private:

  BlackBoxWriter bbw;

  enum {
    BB_REC_IMU,
    BB_REC_PID,
    BB_REC_BAT,
    BB_REC_BARO,
    BB_REC_GPS,
  };

public:

  void log_imu() {
    bbw.writeBeginRecord(BB_REC_IMU);
    bbw.writeUnsignedVB("ts",micros());
    bbw.writeSignedVB("ax*100",AccX*100); //G
    bbw.writeSignedVB("ay*100",AccY*100); //G
    bbw.writeSignedVB("az*100",AccZ*100); //G
    bbw.writeSignedVB("gx*10",GyroX*10); //dps
    bbw.writeSignedVB("gy*10",GyroY*10); //dps
    bbw.writeSignedVB("gz*10",GyroZ*10); //dps
    bbw.writeSignedVB("mx*100",MagX*100); //uT
    bbw.writeSignedVB("my*100",MagX*100); //uT
    bbw.writeSignedVB("mz*100",MagX*100); //uT  
    bbw.writeSignedVB("roll*10",ahrs_roll*10); //deg
    bbw.writeSignedVB("pitch*10",ahrs_pitch*10);; //deg
    bbw.writeSignedVB("yaw*10",ahrs_yaw*10);; //deg
    bbw.writeEndrecord();
  }

  void log_pid() {
    bbw.writeBeginRecord(BB_REC_PID);
    bbw.writeUnsignedVB("ts",micros());
    bbw.writeSignedVB("roll_PID*100",roll_PID*100); //-1 to +1
    bbw.writeSignedVB("pitch_PID*100",pitch_PID*100);; //-1 to +1
    bbw.writeSignedVB("yaw_PID*100",yaw_PID*100);; //-1 to +1
    bbw.writeEndrecord();
  }

  void log_bat() {
    bbw.writeBeginRecord(BB_REC_BAT);
    bbw.writeUnsignedVB("ts",micros());
    bbw.writeUnsignedVB("bat_mA",bat.i*1000); //Battery current (A)
    bbw.writeUnsignedVB("bat_mV",bat.v); //battery voltage (V)
    bbw.writeUnsignedVB("bat_mAh",bat.mah); //battery usage (Ah)
    bbw.writeUnsignedVB("bat_mWh",bat.wh*1000); //battery usage (Wh)
    bbw.writeEndrecord();
  }

  void log_baro() {
    bbw.writeBeginRecord(BB_REC_BARO);
    bbw.writeUnsignedVB("ts",micros());
    bbw.writeUnsignedVB("baro_pa",baro.press_pa); //Barometer pressure (Pa)
    bbw.writeSignedVB("baro_t*100",baro.temp_c*100); //barometer temp (C)
    bbw.writeEndrecord();
  }

  void log_gps() {
    bbw.writeBeginRecord(BB_REC_GPS);
    bbw.writeUnsignedVB("ts",micros()); 
    bbw.writeI32("lat",gps.lat);
    bbw.writeI32("lon",gps.lon);
    bbw.writeEndrecord();
  }

  void start() {
    //add here all loggers, this will write the header with record and field info
    log_imu();
    log_pid();
    log_bat();
    log_gps();
    bbw.start();
  }

  void stop() {
    bbw.stop();
  }

private:

  static void callback_SerialPrintChar(uint8_t c) {
    Serial.printf("%c",c);
  }


//=====================================================================================================================
// Logging to SPI FLASH
//=====================================================================================================================
#if BB_USE == BB_USE_FLASH
  //#define HW_PIN_FLASH_CS    PB3
  //extern SPIClass bb_spi;

  #define BB_BUF_SIZE 64 //write this many bytes per write cycle - needs to be a power of 2, max 256

//continue class BlackBox
public:

  void setup() {
    w25qxx_size = 0;
    w25qxx_adr = 0xffffffff;
    buf_len = 0;
    w25qxx.setSPIPort(bb_spi);
    w25qxx.begin(HW_PIN_BB_CS,18000000);  
    bbw.begin(callback_bbWriteChar);
    findStart();
    Serial.printf("BB_USE_FLASH size=%d start=%d\n", (int)w25qxx_size, (int)w25qxx_adr);
  }

  void csvDump() {
    stop();
    buf_idx = BB_BUF_SIZE;
    w25qxx_readadr = w25qxx_startadr;
    BlackBoxDecoder bbd;
    bbd.csv_decode(callback_bbReadChar, callback_SerialPrintChar);  
  }

  void erase() {
    Serial.println("Full flash erase started, this will take a while to complete...");
    w25qxx.eraseAll(true);
    Serial.println("Full flash erase completed.");
  }

private:

  void findStart() {
    w25qxx_size = w25qxx.readSize();
    w25qxx_adr = w25qxx_size>>1;
    uint32_t stepsize = w25qxx_adr>>1;
    uint8_t buf[16];
    while(stepsize>=16) {
      w25qxx.read(w25qxx_adr, buf, 16);
      int32_t step = -stepsize;
      for(int i=0;i<16;i++) if(buf[i]!=0xff) {
        step = +stepsize;
        break;
      }
      w25qxx_adr += step;
      stepsize >>= 1;
    }
    w25qxx_adr = ((w25qxx_adr / BB_BUF_SIZE) + 1) * BB_BUF_SIZE; //move to next BB_BUF_SIZE boundary
    w25qxx_startadr = w25qxx_adr;
  }

  //static callback
  #include "W25Qxx/W25Qxx.h"
  static W25Qxx w25qxx;
  static uint8_t buf[BB_BUF_SIZE];
  static uint32_t w25qxx_size;
  static uint32_t w25qxx_adr;
  static uint32_t w25qxx_startadr;
  static int buf_len;
  static int buf_idx;
  static uint32_t w25qxx_readadr;

  static uint8_t callback_bbReadChar() {
    if(buf_idx >= BB_BUF_SIZE) {
      w25qxx.read(w25qxx_readadr, buf, BB_BUF_SIZE);
      w25qxx_readadr += BB_BUF_SIZE;
      buf_idx = 0;
    }
    return buf[buf_idx++];
  }

  static void callback_bbWriteChar(uint8_t c) {
    if(buf_len < BB_BUF_SIZE) {
      buf[buf_len++] = c;
    }else{
      w25qxx.pageWrite(w25qxx_adr, buf, BB_BUF_SIZE, false); //non-blocking write
      w25qxx_adr += BB_BUF_SIZE;
      buf_len = 0;
    }
  }
};

BlackBox::W25Qxx BlackBox::w25qxx;
uint8_t BlackBox::buf[BB_BUF_SIZE];
uint32_t BlackBox::w25qxx_size;
uint32_t BlackBox::w25qxx_adr;
uint32_t BlackBox::w25qxx_startadr;
int BlackBox::buf_len;
int BlackBox::buf_idx;
uint32_t BlackBox::w25qxx_readadr;

//=====================================================================================================================
// Logging to RAM Memory
//=====================================================================================================================
#elif BB_USE == BB_USE_MEMORY

  #ifndef BB_MEMORY_BUF_SIZE
    #define BB_MEMORY_BUF_SIZE 60000
  #endif

//continue class BlackBox
public:

  void setup() {
    buf_len = 0;
    bbw.begin(callback_bbWriteChar);
    Serial.printf("BB_USE_MEMORY size=%d\n", BB_MEMORY_BUF_SIZE);
  }

  void csvDump() {
    stop();
    buf_idx = 0;
    BlackBoxDecoder bbd;
    bbd.csv_decode(callback_bbReadChar, callback_SerialPrintChar);  
  }

  void erase() {
    stop();
    setup();
  }
  
private:

  //static callback
  static uint8_t buf[BB_MEMORY_BUF_SIZE];
  static int buf_len;
  static int buf_idx;
  
  static void callback_bbWriteChar(uint8_t c) {
    if(buf_len < BB_MEMORY_BUF_SIZE) {
     buf[buf_len++] = c;
    }
  }

  static uint8_t callback_bbReadChar() {
    if(buf_idx < buf_len) {
      return buf[buf_idx++];
    }else{
      return 0xff;
    }
  }
};

uint8_t BlackBox::buf[BB_MEMORY_BUF_SIZE];
int BlackBox::buf_len;
int BlackBox::buf_idx;

//=====================================================================================================================
// None or undefined
//=====================================================================================================================
#elif BB_USE == BB_USE_NONE || !defined BB_USE
//continue class BlackBox
public:
  void bb_setup() {}
  void bb_csvDump() {}
  void erase() {}
};

//=====================================================================================================================
// Invalid value
//=====================================================================================================================
#else
  #error "invalid BB_USE value"
#endif

BlackBox bb;
