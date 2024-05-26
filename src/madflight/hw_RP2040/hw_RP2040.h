/*########################################################################################################################
This file contains all necessary functions and code for RP2040 hardware platforms

This file defines:
  HW_PIN_xxx -> The pin assignments
  *rcin_Serial -> Serial port for RCIN
  *spi -> SPI port
  *i2c -> I2C port
  HW_WIRETYPE -> the class to use for I2C
  hw_Setup() -> function to init the hardware
  HW_xxx and hw_xxx -> all other hardware platform specific stuff
########################################################################################################################*/

//======================================================================================================================//
//                    DEFAULT BOARD
//======================================================================================================================//
#ifndef HW_BOARD_NAME
  #include <madflight_board_default_RP2040.h>
#endif

//======================================================================================================================//
//  EEPROM
//======================================================================================================================//
#include <EEPROM.h>

void hw_eeprom_begin() {
  EEPROM.begin(4096);
}

uint8_t hw_eeprom_read(uint32_t adr) {
  return EEPROM.read(adr);
}

void hw_eeprom_write(uint32_t adr, uint8_t val) {
  EEPROM.write(adr, val);
}

void hw_eeprom_commit() {
  EEPROM.commit();
}

//======================================================================================================================//
//                    IMU
//======================================================================================================================//
#define IMU_EXEC IMU_EXEC_FREERTOS_OTHERCORE //use FreeRTOS on second core (Note: RP2040 IMU_EXEC_IRQ blocks the system)
#define IMU_FREERTOS_TASK_PRIORITY 7 //IMU Interrupt task priority, higher number is higher priority. Max priority on RP2040 is 7
#define HW_RP2040_USE_FREERTOS //enable FreeRTOS

//======================================================================================================================//
//                    hw_setup()
//======================================================================================================================//

//overclocking, supposedly works up to 270 MHz.
//#ifndef HW_RP2040_SYS_CLK_KHZ
//  #define HW_RP2040_SYS_CLK_KHZ 200000
//#endif

#ifdef HW_RP2040_USE_FREERTOS
  #include <FreeRTOS.h>  //FreeRTOS
  #include <semphr.h>    //FreeRTOS
#endif



void hw_setup() 
{ 
  //print hw info
  Serial.print("HW_RP2040 ");
  #ifdef HW_RP2040_USE_FREERTOS
    Serial.print("HW_RP2040_USE_FREERTOS ");
  #endif
  #ifdef HW_RP2040_SYS_CLK_KHZ
    set_sys_clock_khz(HW_RP2040_SYS_CLK_KHZ, true);
    Serial.printf(" HW_RP2040_SYS_CLK_KHZ %d",HW_RP2040_SYS_CLK_KHZ);
  #endif
  Serial.println();

  //I2C
  i2c->setSDA(HW_PIN_I2C_SDA);
  i2c->setSCL(HW_PIN_I2C_SCL);
  i2c->setClock(1000000);
  i2c->begin();

  //SPI
  spi->begin();
  bb_spi->begin();

  hw_eeprom_begin();
}

void hw_reboot() {

  //does not always work with FreeRTOS running
  watchdog_enable(10, false); //uint32_t delay_ms, bool pause_on_debug
  while(1);

  //does not always work with FreeRTOS running
  //watchdog_reboot(0, 0, 10); //uint32_t pc, uint32_t sp, uint32_t delay_ms
  //while(1);

  //alternate method - does not work...
  //#define AIRCR_Register (*((volatile uint32_t*)(PPB_BASE + 0x0ED0C)))
  //AIRCR_Register = 0x5FA0004;
}

inline uint32_t hw_get_core_num() {
  return get_core_num();
}