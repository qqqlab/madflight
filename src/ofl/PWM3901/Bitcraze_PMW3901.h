//modified for madflight - add: SPIClass *_spi

/* PMW3901 Arduino driver
 * Copyright (c) 2017 Bitcraze AB
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef __BITCRAZE_PMW3901_H__
#define __BITCRAZE_PMW3901_H__

#include "Arduino.h"
#include <SPI.h>

#include <stdint.h>

class Bitcraze_PMW3901 {
public:
  Bitcraze_PMW3901(SPIClass* spi, uint8_t cspin);

  boolean begin(void);

  void readMotionCount(int16_t *deltaX, int16_t *deltaY);
  void enableFrameBuffer();
  void readFrameBuffer(char *FBuffer);

  void setLed(bool ledOn);

private:
  SPIClass *_spi;
  uint8_t _cs;

  void registerWrite(uint8_t reg, uint8_t value);
  uint8_t registerRead(uint8_t reg);
  void initRegisters(void);
};

#endif //__BITCRAZE_PMW3901_H__
