//MF_Serial wrapper around SerialPIO with task based TX buffer

#pragma once

#include "hal_RP2040.h"
#include "../MF_Serial.h"
#include <stream_buffer.h>

void MF_SerialPIO_task( void * pvParameters );

class MF_SerialPIO : public MF_Serial {
  public:
    SerialPIO *_serial;
    StreamBufferHandle_t xStreamBuffer = NULL;


    MF_SerialPIO(SerialPIO *_serial, const char* taskname) {
      this->_serial = _serial;

      xStreamBuffer = xStreamBufferCreate(256, 1); // length, triggerlevel

      xTaskCreate( MF_SerialPIO_task,        // The function that implements the task
                   taskname,                 // Human readable name for the task
                   configMINIMAL_STACK_SIZE, // Stack size (in words!) (configMINIMAL_STACK_SIZE=256)
                   (void*)this,              // Task parameter
                   uxTaskPriorityGet(NULL),  // The priority at which the task is created
                   NULL );                   // No use for the task handle
    }

    void begin(int baud) override {
      _serial->begin(baud);
    }

    int read(uint8_t *buf, int len) override {
      return _serial->readBytes(buf, len);
    }

    int available() override {
      return _serial->available();
    }

    int availableForWrite() override {
      return xStreamBufferSpacesAvailable(xStreamBuffer);
    }

    int write(uint8_t *buf, int len) override {
      if(len <= 0) return 0;
      if(availableForWrite() < len) return 0;
      return xStreamBufferSend( xStreamBuffer, (const void *)buf, len, 0 ); //0 = no wait
    }
};

void MF_SerialPIO_task( void * pvParameters ) {
  MF_SerialPIO *self = (MF_SerialPIO*) pvParameters;
  uint8_t b;
  for( ;; ) {
      xStreamBufferReceive( self->xStreamBuffer, &b, 1, portMAX_DELAY );
      self->_serial->write(b);
  }
}
