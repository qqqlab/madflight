#include <Arduino.h> //Serial
#include "msp.h"
#include "private/MspProcessor.h"

MspProcessor _msp;
MspMessage mspRequest;
MspResponse mspResponse;

bool Msp::process_byte(uint8_t c) {
    if(_msp.parse(c, mspRequest) && mspRequest.isReady() && mspRequest.isCmd()) {
        _msp.processCommand(mspRequest, mspResponse);

        uint8_t buf[256];
        size_t len = mspResponse.serialize(buf, 256);
        Serial.write(buf, len);
        //Serial.flush();

        mspRequest = MspMessage();
        mspResponse = MspResponse();

        return true;
    }
    return false;
}
