



//BinLog uses 32bit microsecond timestamps (good for 1 hour of recording before wrap-around)

#define LOG_TYPE_LEN  64    // max number of message types
#define QUEUE_LENGTH  100   // max number of messages in the queue
#define MAX_MSG_LEN   89    // max message len is FMT with 89 (0x59) bytes



#include <Arduino.h> //Serial
#include "bbx.h"
#include "../hal/hal.h" //FreeRtos
#include "BinLogWriter.h"
#include "BinLog.h"
#include "../cfg/cfg.h"
#include "../tbx/RuntimeTrace.h"

namespace BinLogWriter {
  //prototypes
  static void bbx_task(void *pvParameters);
  void FMT_sendFMT();


  Command_t command = NONE;
  State_t state = READY;
  uint32_t startMicros = 0;
  uint32_t missCnt = 0;

  struct msg_t
  {
      uint8_t buf[MAX_MSG_LEN]; //NOTE: first byte overwritten with msg len when stuffed in queue
  };

  QueueHandle_t queue = nullptr;

  //StaticQueue_t xStaticQueue = {};
  //uint8_t ucQueueStorageArea[QUEUE_LENGTH * sizeof(msg_t)] = {};

  TaskHandle_t xHandle;

  uint32_t typeRegistry[LOG_TYPE_LEN] = {}; //MSB is FMT+FMTU written flag, 31 LSB is message name 
  uint8_t typeRegistry_len = 0;




  void stop() {
    command = STOP;
    //send zero length message to wake up BB task to process command
    uint8_t msg = 0;
    xQueueSend(queue, (void*)&msg, 0);
  }

  void start() {
    command = START;
    //send zero length message to wake up BB task to process command
    uint8_t msg = 0;
    xQueueSend(queue, (void*)&msg, 0);
  }

  void setup() {
    //queue = xQueueCreateStatic(QUEUE_LENGTH, sizeof(msg_t), ucQueueStorageArea, &xStaticQueue); //not available for STM32
    queue = xQueueCreate(QUEUE_LENGTH, sizeof(msg_t));

    if(xTaskCreate(bbx_task, "mf_BBX", MF_FREERTOS_DEFAULT_STACK_SIZE, NULL, uxTaskPriorityGet(NULL), &xHandle) != pdPASS ){
      Serial.println("BBX: Task creation failed");
    }
  }

  //-------------------------------
  // Message Queue
  //-------------------------------

  //append message to queue
  bool queueSend(uint8_t *buf, uint8_t len, uint8_t keepfree) {
    if(keepfree && uxQueueSpacesAvailable(queue) < keepfree) return false;
    //NOTE: random data will pad the message - improve this?
    buf[0] = len; //overwrite header1 with message length
    bool ok = ( xQueueSend(queue, (void*)buf, 0) == pdPASS );
    if(!ok) missCnt++;
    return ok;
  }

  //flush queue to bb filesystem
  void queueFlush() {
    msg_t msg;
    while( xQueueReceive(queue, &msg, 0) == pdPASS ) {
      //extract message length and restore header1
      uint8_t len = msg.buf[0];
      msg.buf[0] = HEAD_BYTE1;
      if(bbx.gizmo) bbx.gizmo->write(msg.buf, len);
    }
  }

  //send FMT-FMT message to queue
  void FMT_sendFMT() {
    struct PACKED {
      uint8_t h1 = HEAD_BYTE1;
      uint8_t h2 = HEAD_BYTE2;
      uint8_t type = 0x80;
      uint8_t msg_type = 0x80;
      uint8_t length = 0x59;
      char name[4] = "FMT";
      char format[16] = "BBnNZ";
      char labels[64] = "Type,Length,Name,Format,Columns";
    } FMT;
    queueSend((uint8_t*)&FMT, sizeof(FMT));
/*
    const uint8_t msg_fmt[] = {
    0xA3, 0x95, 0x80, 0x80, 0x59, 0x46, 0x4D, 0x54, 0x00, 0x42, 0x42, 0x6E, 0x4E, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x54, 0x79, 0x70, 0x65, 0x2C, 0x4C, 0x65, 0x6E, 0x67, 0x74, 0x68, 0x2C, 0x4E, 0x61, 0x6D, 0x65, 0x2C, 0x46, 0x6F, 0x72, 0x6D, 0x61, 0x74, 0x2C, 0x43, 0x6F, 0x6C, 0x75, 0x6D, 0x6E, 0x73, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    queueSend(msg_fmt, sizeof(msg_fmt));
*/
  }

  //-------------------------------
  // Messages used in headers (bypass started flag by setting isHeader)
  //-------------------------------

  void log_header_msg(const char* msg, bool isHeader = true) {
      BinLog bl("MSG");
      bl.isHeader = isHeader;
      bl.TimeUS();
      bl.char64("Message",msg);
  }

  void log_header_parm(const char* name, float value, float default_value, bool isHeader = true) {
      BinLog bl("PARM");  //PARM parameter value
      bl.isHeader = isHeader;
      bl.TimeUS(); //TimeUS: Time since system startup
      bl.char16("Name", name); //Name: parameter name
      bl.flt("Value", value); //Value: parameter value
      bl.flt("Default", default_value); //Default: default parameter value for this board and config
  }

  void log_header_mult(char id, float mult) {
      BinLog bl("MULT");  
      bl.isHeader = true;
      bl.TimeUS(); //uint64_t TimeUS: Time since system startup
      bl.i8("Id", id); //char Id: character referenced by FMTU
      bl.flt("Mult", mult); //double Mult: numeric multiplier
  }
  
  void log_header_unit(char id, const char *label) {
      BinLog bl("UNIT");  
      bl.isHeader = true;
      bl.TimeUS(); //uint64_t TimeUS: Time since system startup
      bl.i8("Id", id); //char Id: character referenced by FMTU
      bl.char16("Label", label); //char[64] Label: Unit - SI where available
  }

  bool queueSendFMTU(bool sendFMT, uint8_t type, const char *units, const char* multipliers) {
      BinLog bl("FMTU"); //Message defining units and multipliers used for fields of other messages
      bl.isHeader = true;
      bl.TimeUS(); //uint64_t TimeUS: Time since system startup
      bl.u8("FmtType", type); //char FmtType: numeric reference to associated FMT message
      bl.char16("UnitIds", units); //char[16] UnitIds: each character refers to a UNIT message.  The unit at an offset corresponds to the field at the same offset in FMT.Format
      bl.char16("MultIds", multipliers); //char[16] MultIds: each character refers to a MULT message.  The multiplier at an offset corresponds to the field at the same offset in FMT.Format
      
      bool queued = false;
      if(sendFMT) {
        queued = bl.msg_queueFMT(); //send FMT for FMTU
      }else{
        queued = bl.msg_queue(); //send FMTU
      }
      bl.inhibit = true; //don't send anything in destructor
      return queued;
  }

  void log_msg(const char* msg) {
    log_header_msg(msg, false);
  }

  void log_parm(const char* name, float value, float default_value) {
    log_header_parm(name, value, default_value, false);
  }

  //-------------------------------
  // Message type registry, gives out msg type IDs and keeps track if FMT/FMTU headers were written
  //-------------------------------

  uint8_t typeRegistry_find(const char *name, bool *fmt_write) {
    uint8_t i;
    uint32_t findname = 0;
    strncpy((char*)&findname, name, 4);
    for(i=0;i<typeRegistry_len;i++) {
      if((typeRegistry[i] & 0x7fffffff) == findname) {
        *fmt_write = ((typeRegistry[i] & 0x80000000) == 0); //get FMT written flag
        return i + 0x81;
      }
    }
    if(i<LOG_TYPE_LEN) {
      //add new type
      typeRegistry[typeRegistry_len] = findname;
      typeRegistry_len++;
      *fmt_write = true;
      return i + 0x81;
    }
    //too many types, return next type id but don't write FMT record
    *fmt_write = false;
    return i + 0x81;
  }
  
  void typeRegistry_FMT_was_sent(char typ){
    uint8_t i = typ - 0x81;
    if(i<LOG_TYPE_LEN) {
      typeRegistry[i] |= 0x80000000; //set FMT written flag
    }
  }




  void cmd_start() {
    if(state!=READY) return;
    state = STARTING;

    //empty the queue
    xQueueReset(queue);

    //clear type registry, force writing of FMT records
    typeRegistry_len = 0;

    //open bbfs (black box file system)
    if(!bbx.gizmo) return;
    if(!bbx.gizmo->writeOpen()) return; //bbfs emits error message

    //log file time start now
    startMicros = micros();
    missCnt = 0;

    //write headers (flush often)
    FMT_sendFMT(); //send FMT for FMT
    queueSendFMTU(true,0,"",""); //send FMT for FMTU
    log_header_msg("ArduPlane"); //this sets the vehicle type -> which drives the translaton of flightmode codes to names (among other things probably)
    //log_header_msg("ArduCopter");  //gives problems with plot.ardupilot.org
    //TODO log_header_msg(MADFLIGHT_VERSION);
    queueFlush();
    
    //write multipliers
    for(int i=0;i<_num_multipliers;i++) {
      log_header_mult(log_Multipliers[i].ID, log_Multipliers[i].mult);
      queueFlush();
    }
    
    //write units
    for(int i=0;i<_num_units;i++) {
      log_header_unit(log_Units[i].ID, log_Units[i].unit);
      queueFlush();
    }
    
    //write parameters (plot.ardupilot.org needs at least one)
    String name;
    float value = 0;
    int i = 0;
    while(cfg.getNameAndValue(i,&name,&value)) {
      log_header_parm(name.c_str(), value, 0);
      queueFlush();
      i++;
    }

    //allow logging
    state = STARTED;
  }
  
  void cmd_stop() {
    //stop logging
    state = READY;

    //flush queue to sdcard
    queueFlush();

    //close file
    if(bbx.gizmo) bbx.gizmo->close();
  }

  

  static void bbx_task(void *pvParameters) {
    (void)pvParameters;
    static RuntimeTrace runtimeTrace = RuntimeTrace("BBX");
    static msg_t msg;
    for(;;) {
      bool updated = (xQueueReceive(queue, &msg, portMAX_DELAY) == pdPASS);
      if(updated) {
        runtimeTrace.start();
        switch(command) {
        case START:
          command = NONE;
          cmd_start();
          break;
        case STOP:
          command = NONE;
          cmd_stop();
          break;
        case NONE:
          //extract message length and restore header1
          uint8_t len = msg.buf[0];
          msg.buf[0] = HEAD_BYTE1;
          if(bbx.gizmo) bbx.gizmo->write(msg.buf, len);
        }
      }
      runtimeTrace.stop(updated);
    }
  }

};
