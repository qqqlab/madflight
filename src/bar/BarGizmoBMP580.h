/*=======================================================
Driver for BMP580 / BMP581 / BMP585 pressure sensor
=======================================================*/

#pragma once

#include "bar.h"
#include "../hal/MF_I2C.h"
#include "BMP580/SparkFun_BMP581_Arduino_Library.h"

/*
NORMAL MODE
ODR(Hz), max OSR_P, max OSR_T
240 2 4
220 4 2
140 8 2
80 16 4
45 32 2
20 64 32
10 128 64

CONTINOUS MODE
OSR_P Pressure oversampling, OSR_T Temperature oversampling, Typical pressure RMS noise at 100kPa, Typical ODR in CONTINUOUS mode
Lowest power
×1 ×1 0.78 Pa 498 Hz 
×2 ×1 0.58 Pa 374 Hz
Standard resolution
×4 ×1 0.41 Pa 255 Hz
×8 ×1 0.30 Pa 155 Hz
High resolution
×16 ×1 0.21 Pa 87 Hz
×32 ×2 0.15 Pa 46 Hz
×64 ×4 0.11 Pa 24 Hz
Highest resolution
×128 ×8 0.08 Pa 12 Hz
*/


class BarGizmoBMP580: public BarGizmo {
private:
  BMP581 pressureSensor;

public:
  const char* name() override {return "BMP580";}
  BarGizmoBMP580(MF_I2C *i2c, int8_t i2c_adr, uint32_t sample_rate) {
    (void) sample_rate; //TODO - currently fixed at 87 Hz (16x OSR_P, 1x OSR_T, CONTINUOUS mode)

    if(i2c_adr == 0) i2c_adr = 0x47; // fixed: 0x47 or 0x46
    
    // Check if sensor is connected and initialize
    int8_t err = BMP5_OK;
    int tries = 5;
    while(tries--) {
      err = pressureSensor.beginI2C(i2c_adr, i2c);
      if(err == BMP5_OK) break;
      delay(100);
    }
    if(err != BMP5_OK) {
      // Not connected, inform user
      Serial.printf("BAR: BMP580 ERROR - not connected, check wiring and I2C address (0x%02X) err=%d\n", i2c_adr, err);
      return;
    }

    err = pressureSensor.setMode(BMP5_POWERMODE_CONTINOUS);
    if(err != BMP5_OK) {
      // Interrupt settings failed, most likely a communication error (code -2)
      Serial.print("BAR: BMP580 ERROR - Set mode failed! Error code: ");
      Serial.println(err);
    }


    // Configure the BMP581 to trigger interrupts whenever a measurement is performed
    BMP581_InterruptConfig interruptConfig = {
      //.enable   = BMP5_INTR_ENABLE,    // Enable interrupt pin
      .enable   = BMP5_INTR_DISABLE,    // Disable interrupt pin
      .drive    = BMP5_INTR_PUSH_PULL, // Push-pull or open-drain
      .polarity = BMP5_ACTIVE_HIGH,    // Active low or high
      .mode     = BMP5_PULSED,         // Latch or pulse signal
      .sources  = {
        .drdy_en = BMP5_ENABLE,        // Trigger interrupts when data is ready
        .fifo_full_en = BMP5_DISABLE,  // Trigger interrupts when FIFO is full
        .fifo_thres_en = BMP5_DISABLE, // Trigger interrupts when FIFO threshold is reached
        .oor_press_en = BMP5_DISABLE   // Trigger interrupts when pressure goes out of range
      }
    };
    err = pressureSensor.setInterruptConfig(&interruptConfig);
    if(err != BMP5_OK) {
      // Interrupt settings failed, most likely a communication error (code -2)
      Serial.print("BAR: BMP580 ERROR - Interrupt settings failed! Error code: ");
      Serial.println(err);
    }

    bmp5_osr_odr_press_config c;
    c.press_en = BMP5_ENABLE;
    c.odr = BMP5_ODR_120_HZ; //ignored for CONTINOUS MODE
    c.osr_p = BMP5_OVERSAMPLING_16X;
    c.osr_t = BMP5_OVERSAMPLING_1X;
    pressureSensor.setOSRMultipliers(&c);
  }

  bool update(float *press, float *temp) override {
    if(micros() - ((BarState*)this)->ts < 10000) return false; //sample rate is 85Hz, 11.7ms, bail out if last sample was less than 10 ms ago

    // Variable to track errors returned by API calls
    int8_t err = BMP5_OK;

    // Get the interrupt status to know which condition triggered
    uint8_t interruptStatus = 0;
    err = pressureSensor.getInterruptStatus(&interruptStatus);
    if(err != BMP5_OK) {
      // Status get failed, most likely a communication error (code -2)
      //Serial.print("Get interrupt status failed! Error code: ");
      //Serial.println(err);
      return false;
    }

    // Check if this is the "data ready" interrupt condition
    if( (interruptStatus & BMP5_INT_ASSERTED_DRDY) == 0) {
      //no new measurement available
      return false;
    }

    // Get measurements from the sensor
    bmp5_sensor_data data = {0,0};
    err = pressureSensor.getSensorData(&data);
    if(err == BMP5_OK) {
      *press = data.pressure;
      *temp = data.temperature;
      return true;
    } else {
      // Acquisition failed, most likely a communication error (code -2)
      //Serial.print("Error getting data from sensor! Error code: ");
      //Serial.println(err);
      return false;
    }
  }

};
































