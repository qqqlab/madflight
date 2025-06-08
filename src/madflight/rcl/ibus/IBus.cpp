/*
 * Interface to the RC IBus protocol
 * 
 * Based on original work from: https://gitlab.com/timwilkinson/FlySkyIBus
 * Extended to also handle sensors/telemetry data to be sent back to the transmitter,
 * interrupts driven and other features.
 *
 * This lib requires a hardware UART for communication
 * Another version using software serial is here https://github.com/Hrastovc/iBUStelemetry
 * 
 * Explaination of sensor/ telemetry prtocol here: 
 * https://github.com/betaflight/betaflight/wiki/Single-wire-FlySky-(IBus)-telemetry
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either  
 * version 2.1 of the License, or (at your option) any later version.
 *   
 * Created 12 March 2019 Bart Mellink
 * Updated 4 April 2019 to support ESP32
 * updated 13 jun 2019 to support STM32 (pauluzs)
 * Updated 21 Jul 2020 to support MBED (David Peverley) 
 */

#include <Arduino.h>
#include "IBus.h"

// pointer to the first class instance to be used to call the loop() method from timer interrupt
// will be initiated by class constructor, then daisy channed to other class instances if we have more than one
IBus* IBusfirst = NULL;

/*
 *  supports max 14 channels in this lib (with messagelength of 0x20 there is room for 14 channels)

  Example set of bytes coming over the iBUS line for setting servos: 
    20 40 DB 5 DC 5 54 5 DC 5 E8 3 D0 7 D2 5 E8 3 DC 5 DC 5 DC 5 DC 5 DC 5 DC 5 DA F3
  Explanation
    Protocol length: 20
    Command code: 40 
    Channel 0: DB 5  -> value 0x5DB
    Channel 1: DC 5  -> value 0x5Dc
    Channel 2: 54 5  -> value 0x554
    Channel 3: DC 5  -> value 0x5DC
    Channel 4: E8 3  -> value 0x3E8
    Channel 5: D0 7  -> value 0x7D0
    Channel 6: D2 5  -> value 0x5D2
    Channel 7: E8 3  -> value 0x3E8
    Channel 8: DC 5  -> value 0x5DC
    Channel 9: DC 5  -> value 0x5DC
    Channel 10: DC 5 -> value 0x5DC
    Channel 11: DC 5 -> value 0x5DC
    Channel 12: DC 5 -> value 0x5DC
    Channel 13: DC 5 -> value 0x5DC
    Checksum: DA F3 -> calculated by adding up all previous bytes, total must be FFFF
 */

void IBus::begin() {
  this->state = DISCARD;
  this->last = millis();
  this->ptr = 0;
  this->len = 0;
  this->chksum = 0;
  this->lchksum = 0;

  // we need to process the iBUS sensor protocol handler frequently enough (at least once each ms) to ensure the response data
  // from the sensor is sent on time to the receiver
  // if timerid==IBus_NOTIMER the user is responsible for calling the loop function
  this->IBusnext = IBusfirst;
  IBusfirst = this; 
}

// called from timer interrupt or mannually by user (if IBus_NOTIMER set in begin())
bool IBus::update(void) {
  bool ret = true;

  // if we have multiple instances of IBus, we (recursively) call the other instances loop() function
  if (IBusnext) ret = ret && IBusnext->update(); 

  // only process data already in our UART receive buffer 
  while (serial->available() > 0) {
    // only consider a new data package if we have not heard anything for >3ms
    uint32_t now = millis();
    if (now - last >= PROTOCOL_TIMEGAP){
      state = GET_LENGTH;
    }
    last = now;
    
    uint8_t v = serial->read();
    switch (state) {
      case GET_LENGTH:
        if (v <= PROTOCOL_LENGTH && v > PROTOCOL_OVERHEAD) {
          ptr = 0;
          len = v - PROTOCOL_OVERHEAD;
          chksum = 0xFFFF - v;
          state = GET_DATA;
        } else {
          state = DISCARD;
        }
        break;

      case GET_DATA:
        buffer[ptr++] = v;
        chksum -= v;
        if (ptr == len) {
          state = GET_CHKSUML;
        }
        break;
        
      case GET_CHKSUML:
        lchksum = v;
        state = GET_CHKSUMH;
        break;

      case GET_CHKSUMH:
        // Validate checksum
        if (chksum == (v << 8) + lchksum) {
          // Checksum is all fine Execute command - 
          uint8_t adr = buffer[0] & 0x0f;
          if (buffer[0]==PROTOCOL_COMMAND40) {
            // Valid servo command received - extract channel data
            for (uint8_t i = 1; i < PROTOCOL_CHANNELS * 2 + 1; i += 2) {
              channel[i / 2] = buffer[i] | (buffer[i + 1] << 8);
            }
            cnt_rec++;
          } else if (adr<=NumberSensors && adr>0 && len==1) {

            // all sensor data commands go here
            // we only process the len==1 commands (=message length is 4 bytes incl overhead) to prevent the case the
            // return messages from the UART TX port loop back to the RX port and are processed again. This is extra
            // precaution as it will also be prevented by the PROTOCOL_TIMEGAP required
           sensorinfo *s = &sensors[adr-1];
           uint8_t writeBuf[PROTOCOL_LENGTH];
           uint8_t writeLen = 0;
           delayMicroseconds(100);
            switch (buffer[0] & 0x0f0) {
              case PROTOCOL_COMMAND_DISCOVER: // 0x80, discover sensor
                cnt_poll++;  
                // echo discover command: 0x04, 0x81, 0x7A, 0xFF
                writeBuf[writeLen++] = 0x04;
                writeBuf[writeLen++] = PROTOCOL_COMMAND_DISCOVER + adr;
                //serial->write(0x04);
                //serial->write(PROTOCOL_COMMAND_DISCOVER + adr);
                chksum = 0xFFFF - (0x04 + PROTOCOL_COMMAND_DISCOVER + adr);
                break;
              case PROTOCOL_COMMAND_TYPE: // 0x90, send sensor type
                // echo sensortype command: 0x06 0x91 0x00 0x02 0x66 0xFF 
                writeBuf[writeLen++] = 0x06;
                writeBuf[writeLen++] = PROTOCOL_COMMAND_TYPE + adr;
                writeBuf[writeLen++] = s->sensorType;
                writeBuf[writeLen++] = s->sensorLength;
                //stream->write(0x06);
                //stream->write(PROTOCOL_COMMAND_TYPE + adr);
                //stream->write(s->sensorType);
                //stream->write(s->sensorLength);
                chksum = 0xFFFF - (0x06 + PROTOCOL_COMMAND_TYPE + adr + s->sensorType + s->sensorLength);
                break;
              case PROTOCOL_COMMAND_VALUE: // 0xA0, send sensor data
                cnt_sensor++;
                uint8_t t;
                // echo sensor value command: 0x06 0x91 0x00 0x02 0x66 0xFF 
                writeBuf[writeLen++] = t = 0x04 + s->sensorLength;
                //stream->write(t = 0x04 + s->sensorLength);
                chksum = 0xFFFF - t;
                writeBuf[writeLen++] = t = PROTOCOL_COMMAND_VALUE + adr;
                //stream->write(t = PROTOCOL_COMMAND_VALUE + adr);
                chksum -= t;
                writeBuf[writeLen++] = t = s->sensorValue & 0x0ff;
                //stream->write(t = s->sensorValue & 0x0ff);
                chksum -= t;
                writeBuf[writeLen++] = t = (s->sensorValue >> 8) & 0x0ff;
                //stream->write(t = (s->sensorValue >> 8) & 0x0ff); 
                chksum -= t;
                if (s->sensorLength==4) {
                  writeBuf[writeLen++] = t = (s->sensorValue >> 16) & 0x0ff;
                  //stream->write(t = (s->sensorValue >> 16) & 0x0ff); 
                  chksum -= t;
                  writeBuf[writeLen++] = t = (s->sensorValue >> 24) & 0x0ff;
                  //stream->write(t = (s->sensorValue >> 24) & 0x0ff); 
                  chksum -= t;                  
                }
                break;
              default:
                adr=0; // unknown command, prevent sending chksum
                break;
            }
            if (adr>0) {
              writeBuf[writeLen++] = chksum & 0x0ff;
              //stream->write(chksum & 0x0ff);
              writeBuf[writeLen++] = chksum >> 8;
              //stream->write(chksum >> 8);              
            }
            if (writeLen) {
              serial->write(writeBuf, writeLen);
            }
            writeLen = 0;
          }
        }
        state = DISCARD;
        break;

      case DISCARD:
      default:
        break;
    }
  }

  return ret;
}

uint16_t IBus::readChannel(uint8_t channelNr) {
  if (channelNr < PROTOCOL_CHANNELS) {
    return channel[channelNr];
  } else {
    return 0;
  }
}

uint8_t IBus::addSensor(uint8_t type, uint8_t len) {
  // add a sensor, return sensor number
  if (len!=2 && len!=4) len = 2;
  if (NumberSensors < SENSORMAX) {
    sensorinfo *s = &sensors[NumberSensors];
    s->sensorType = type;
    s->sensorLength = len;
    s->sensorValue = 0;
    NumberSensors++;
  }
  return NumberSensors;
}

void IBus::setSensorMeasurement(uint8_t adr, int32_t value) {
   if (adr<=NumberSensors && adr>0)
     sensors[adr-1].sensorValue = value;
}

