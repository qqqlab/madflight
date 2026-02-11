#include <Arduino.h> //Serial
#include "msp.h"
#include "private/MspProcessor.h"
#include "../../out/out.h"

MspProcessor _msp;
MspMessage mspRequest;
MspResponse mspResponse;

static uint8_t buf[256]; //TODO - remove me, integrate this into MspResponse::data buffer

bool Msp::process_byte(uint8_t c) {
    out.testmotor_enable(true); //reset output disable watchdog
    if(_msp.parse(c, mspRequest) && mspRequest.isReady() && mspRequest.isCmd()) {
        _msp.processCommand(mspRequest, mspResponse);

        size_t len = mspResponse.serialize(buf, sizeof(buf));
        Serial.write(buf, len);
        //Serial.flush();

        mspRequest = MspMessage();
        mspResponse = MspResponse();

        return true;
    }
    return false;
}
