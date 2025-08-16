#define MF_MOD "BBX"

#include <Arduino.h> //Serial
#include "bbx.h"
#include "BinLogWriter.h"


//include all module interfaces for loggers
#include "../ahr/ahr.h"
#include "../alt/alt.h"
#include "../bar/bar.h"
#include "../bat/bat.h"
#include "../cfg/cfg.h"
#include "../gps/gps.h"
#include "../hal/hal.h"
#include "../imu/imu.h"
#include "../led/led.h"
#include "../bbx/bbx.h"
#include "../mag/mag.h"
#include "../out/out.h"
#include "../pid/pid.h"
#include "../rcl/rcl.h"
#include "../veh/veh.h"

//create global module instance
Bbx bbx;

//-------------------------------
// Blackbox interface
//-------------------------------

#ifdef ARDUINO_ARCH_RP2040
  #include "BbxGizmoSdspi_RP2040.h"

  int Bbx::gizmo_create() {
    //create gizmo
    switch(config.gizmo) {
      case Cfg::bbx_gizmo_enum::mf_NONE :
        break;
      case Cfg::bbx_gizmo_enum::mf_SDSPI :
        gizmo = new BbxGizmoSdspi(&config);
        break;
      case Cfg::bbx_gizmo_enum::mf_SDMMC :
        gizmo = new BbxGizmoSdspi(&config);
        break;
    }
    return 0;
  }

#elif defined ARDUINO_ARCH_ESP32

    #define BBX_USE_MMC
    #include "BbxGizmoSdspi+Sdmmc_ESP32.h"
    #undef BBX_USE_MMC
    #include "BbxGizmoSdspi+Sdmmc_ESP32.h"

  int Bbx::gizmo_create() {
    //create gizmo
    switch(config.gizmo) {
      case Cfg::bbx_gizmo_enum::mf_NONE :
        break;
      case Cfg::bbx_gizmo_enum::mf_SDSPI :
        gizmo = new BbxGizmoSdspi(&config);
        break;
      case Cfg::bbx_gizmo_enum::mf_SDMMC :
        gizmo = new BbxGizmoSdmmc(&config);
        break;
      return -1001;
        break;
    }
    return 0;
  }

#else
  int Bbx::gizmo_create() {
    if(config.gizmo != Cfg::bbx_gizmo_enum::mf_NONE) {
      Serial.println("\n" MF_MOD ": ERROR BBX not available for this processor\n");
      return -1001;
    }
    return 0;
  }
#endif


int Bbx::setup() {
  cfg.printModule(MF_MOD);

  //create gizmo
  delete gizmo;
  gizmo = nullptr;
  int rv = gizmo_create();
  if(rv!=0) return rv;

  //setup BinLogWriter
  BinLogWriter::setup();

  //setup gizmo
  if(!gizmo) return -1001;
  gizmo->setup();
  return 0;
}

void Bbx::start() {
  BinLogWriter::start();
}

void Bbx::stop() {
  BinLogWriter::stop();
}

void Bbx::erase() {
  if(!gizmo) return;
  stop();
  gizmo->erase();
}

void Bbx::dir() {
  if(!gizmo) return;
  gizmo->dir();
}

void Bbx::bench() {
  if(!gizmo) return;
  gizmo->bench();
}

void Bbx::info() {
  if(!gizmo) return;
  gizmo->info();
}

int Bbx::read(const char* filename, uint8_t **data) {
  if(!gizmo) return 0;
  return gizmo->read(filename, data);
}

//-------------------------------
// Loggers
//-------------------------------

void Bbx::log_bar() {
  BinLog bl("BARO");
  bl.TimeUS();                      //uint64_t TimeUS: Time since system startup [us]
  bl.u8("I", 0, 1, "instance");     //uint8_t I: barometer sensor instance number [-]
  bl.flt("Alt", bar.alt, 1, "m");   //float Alt: calculated altitude [m]
                                    //float AltAMSL: altitude AMSL
  bl.flt("Press", bar.press, 1, "Pa");      //float Press: measured atmospheric pressure [Pa]
  bl.i16("Temp", bar.temp, 1, "degC"); //int16_t Temp: measured atmospheric temperature [C]
                                    //float CRt: derived climb rate from primary barometer
                                    //uint32_t SMS: time last sample was taken
                                    //float Offset: raw adjustment of barometer altitude, zeroed on calibration, possibly set by GCS
                                    //float GndTemp: temperature on ground, specified by parameter or measured while on ground
                                    //uint8_t Health: true if barometer is considered healthy
} 

void Bbx::log_bat() {
  BinLog bl("BAT");
  bl.TimeUS();                      //uint64_t TimeUS: Time since system startup [us]
  bl.u8("I", 0, 1, "instance");     //uint8_t Inst: battery instance number [-]
  bl.flt("Volt", bat.v, 1, "V");    //float Volt: measured voltage [V]
                                    //float VoltR: estimated resting voltage
  bl.flt("Curr", bat.i, 1, "A");    //float Curr: measured current [A]
  bl.flt("CurrTot", bat.mah*1000, 1, "Ah");  //float CurrTot: consumed Ah, current * time [Ah]
  bl.flt("EnrgTot", bat.wh, 1, "Wh");  //float EnrgTot: consumed Wh, energy this battery has expended [Wh]
                                    //int16_t Temp: measured temperature
                                    //float Res: estimated battery resistance
                                    //uint8_t RemPct: remaining percentage
                                    //uint8_t H: health
                                    //uint8_t SH: state of health percentage.  0 if unknown
} 

//Ardupilot definition: { "GPS",
//"QBBIHBcLLeffffB", "TimeUS,I,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs,VZ,Yaw,U", 
//"s#-s-S-DUmnhnh-", 
//"F--C-0BGGB000--" , true }
void Bbx::log_gps() {
  BinLog bl("GPS");                 // Information received from GNSS systems attached to the autopilot
  bl.TimeUS();                      //uint64_t TimeUS 1e-6 [s]: Time since system startup [us]
  bl.u8("I", 0, 1, "instance");     //uint8_t I 0 [instance]: GPS instance number
  bl.u8("Status", gps.fix);         //uint8_t Status 0 []: GPS Fix type; 2D fix, 3D fix etc. --madflight 0:no fix, 1:fix 2:2D fix, 3:3D fix)
  //NOTE: gps.time is time in milliseconds since midnight UTC
  bl.u32("GMS", gps.time);          //uint32_t GMS 1e-3 [s]: milliseconds since start of GPS Week [ms]
  bl.u16("GWk", 2288);              //uint16_t GWk 0 []: weeks since 5 Jan 1980 [week]
  bl.u8("NSats", gps.sat, 1, "satellites"); //uint8_t NSats '0':1e0 ['S':satellites]: number of satellites visible [-]
  bl.i16x100("HDop", gps.hdop, 1, "m");     //int16_t*100 HDop '-':0 ['-']: horizontal dilution of precision [-]
  bl.i32latlon("Lat", gps.lat, 1e-7, "deglatitude");     //int32_t Lat 1e-7 ['D':deglatitude]: latitude [deg*10e7]
  bl.i32latlon("Lng", gps.lon, 1e-7, "deglongitude");     //int32_tLng 1e-7 ['U':deglongitude]: longitude [deg*10e7]
  bl.flt("Alt", gps.alt/1000.0, 1, "m");           //i32x100 Alt 'B':1e-2 ['m':m]: altitude [mm]
  bl.flt("Spd", gps.sog/1000.0, 1, "m/s");           //float Spd 1 ['n':m/s]: ground speed [mm/s]
  bl.i32("GCrs",gps.cog, 1, "degheading");     //float GCrs 1 ['h':degheading]: ground course [deg]
  bl.flt("VZ", gps.veld/1000.0, 1, "m/s");           //float VZ 1 ['n':m/s]: vertical speed [mm/s]
  bl.flt("Yaw", 0, 1, "degheading");           //float Yaw 1 ['h':degheading]: vehicle yaw
  bl.u8("U",1);                     //U: boolean value indicating whether this GPS is in use

  //non standard (use short names!!!)
  //bl.u32("D",gps.date);  //date as DDMMYY
}

/*
void Bbx::log_pos(float homeAlt, float OriginAlt) override {
  BinLog bl("POS");
  bl.TimeUS(); //uint64_t TimeUS 1e-6 [s]: Time since system startup [us]
  bl.i32latlon("Lat", gps.lat, 1e-7, "deglatitude");
  bl.i32latlon("Lng", gps.lon, 1e-7, "deglongitude");
  bl.flt("Alt", gps.alt/1000.0, 1, "m");;
  bl.flt("RelHomeAlt", gps.alt/1000.0 - homeAlt, 1, "m");
  bl.flt("RelOriginAlt", gps.alt/1000.0 - OriginAlt, 1, "m");
}
*/

//AHRS roll/pitch/yaw plus filtered+corrected IMU data
void Bbx::log_ahrs() {
  BinLog bl("AHRS"); 
  bl.TimeUS();
  bl.i16("ax",ahr.ax*1000, 1e-3, "G"); //G
  bl.i16("ay",ahr.ay*1000, 1e-3, "G"); //G
  bl.i16("az",ahr.az*1000, 1e-3, "G"); //G
  bl.i16("gx",ahr.gx*10, 1e-1, "deg/s"); //dps
  bl.i16("gy",ahr.gy*10, 1e-1, "deg/s"); //dps
  bl.i16("gz",ahr.gz*10, 1e-1, "deg/s"); //dps
  bl.i16("mx",ahr.mx*100, 1e-2, "uT"); //uT
  bl.i16("my",ahr.my*100, 1e-2, "uT"); //uT
  bl.i16("mz",ahr.mz*100, 1e-2, "uT"); //uT
  bl.i16("roll",ahr.roll*100, 1e-2, "deg"); //deg -180 to 180
  bl.i16("pitch",ahr.pitch*100, 1e-2, "deg");; //deg -90 to 90
  bl.i16("yaw",ahr.yaw*100, 1e-2, "deg");; //deg -180 to 180
}

void Bbx::log_att() {
  BinLog bl("ATT");
  bl.TimeUS();
  bl.i16("DesRoll",0, 1e-2, "deg");
  bl.i16("Roll",ahr.roll*100, 1e-2, "deg");
  bl.i16("DesPitch",0, 1e-2, "deg");
  bl.i16("Pitch",-ahr.pitch*100, 1e-2, "deg");
  bl.u16("DesYaw",0, 1e-2, "deg");
  bl.u16("Yaw",ahr.yaw*100, 1e-2, "deg");
  bl.u16x100("ErrRP",0);
  bl.u16x100("ErrYaw",0);
  bl.u8 ("AEKF",3);
}

//raw (unfiltered but corrected) IMU data
void Bbx::log_imu() {
  BinLog bl("IMU");
  bl.keepFree = QUEUE_LENGTH/4; //keep 25% of queue free for other messages
  bl.TimeUS(imu.ts);
  bl.i16("ax",(imu.ax - cfg.imu_cal_ax)*1000, 1e-3, "G"); //G
  bl.i16("ay",(imu.ay - cfg.imu_cal_ay)*1000, 1e-3, "G"); //G
  bl.i16("az",(imu.az - cfg.imu_cal_az)*1000, 1e-3, "G"); //G
  bl.i16("gx",(imu.gx - cfg.imu_cal_gx)*10, 1e-1, "deg/s"); //dps
  bl.i16("gy",(imu.gy - cfg.imu_cal_gy)*10, 1e-1, "deg/s"); //dps
  bl.i16("gz",(imu.gz - cfg.imu_cal_gz)*10, 1e-1, "deg/s"); //dps
  #if MAG_USE != MAG_USE_NONE
    //get from magnetometer
    bl.i16("mx",((mag.x - cfg.mag_cal_x) * cfg.mag_cal_sx)*100, 1e-2, "uT"); //uT
    bl.i16("my",((mag.y - cfg.mag_cal_y) * cfg.mag_cal_sy)*100, 1e-2, "uT"); //uT
    bl.i16("mz",((mag.z - cfg.mag_cal_z) * cfg.mag_cal_sz)*100, 1e-2, "uT"); //uT
  #else
    //get from imu
    if(imu.hasMag()) {
      bl.i16("mx",((imu.mx - cfg.mag_cal_x) * cfg.mag_cal_sx)*100, 1e-2, "uT"); //uT
      bl.i16("my",((imu.my - cfg.mag_cal_y) * cfg.mag_cal_sy)*100, 1e-2, "uT"); //uT
      bl.i16("mz",((imu.mz - cfg.mag_cal_z) * cfg.mag_cal_sz)*100, 1e-2, "uT"); //uT
    }
  #endif
  bl.i16("roll",ahr.roll*100, 1e-2, "deg"); //deg -180 to 180
  bl.i16("pitch",ahr.pitch*100, 1e-2, "deg");; //deg -90 to 90
  bl.i16("yaw",ahr.yaw*100, 1e-2, "deg");; //deg -180 to 180
}

void Bbx::log_mode(uint8_t fm, const char* name) {
  BinLog bl("MODE");
  bl.TimeUS();
  bl.u8flightmode("Mode",fm);
  bl.u8("ModeNum",fm);
  bl.u8("Rsn",1); //ModeReason
  bl.char16("Name",name); //extenstion to "standard" ArduPilot BinLog
}

void Bbx::log_msg(const char* msg) {
  BinLogWriter::log_msg(msg);
}

void Bbx::log_parm(const char* name, float value, float default_value) {
  BinLogWriter::log_parm(name, value, default_value);
}

//system status
void Bbx::log_sys() {
  BinLog bl("SYS");
  bl.TimeUS();
  bl.u32("BBm",BinLogWriter::missCnt);
  bl.u32("IMi",imu.interrupt_cnt);
  bl.i32("IMm",imu.interrupt_cnt - imu.update_cnt);
}

