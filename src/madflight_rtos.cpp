/*==========================================================================================
MIT License

Copyright (c) 2026 https://madflight.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
===========================================================================================*/

#include "madflight_modules.h"

#if defined ARDUINO_ARCH_ESP32
#include <esp_task_wdt.h>
void hal_task_wdt_disable() {
  esp_task_wdt_init(5, false); //disable task watchdog
}
#else
void hal_task_wdt_disable() {}
#endif


extern void madflight_setup();

static void cli_task(void *pvParameters) {
  (void)pvParameters;
  for(;;) {
    hal_task_wdt_disable();
    cli.update(); // process CLI commands
    portYIELD();
  }
}

static void rcl_task(void *pvParameters) {
  (void)pvParameters;
  for(;;) {
    rcl.update(); // get rc radio commands
    portYIELD();
  }
}

static void sensor_task(void *pvParameters) {
  (void)pvParameters;
  for(;;) {
    if(bar.update()) bbx.log_bar(); // barometer
    mag.update(); // magnetometer
    if(gps.update()) bbx.log_gps(); // gps
    if(bat.update()) bbx.log_bat(); // battery consumption
    rdr.update(); // radar
    ofl.update(); // optical flow
    portYIELD();
  }
}

void madflight_rtos_setup() {
  madflight_setup();

  TaskHandle_t sensor_task_handle;
  TaskHandle_t rcl_task_handle;
  TaskHandle_t cli_task_handle;
  BaseType_t rv = pdPASS;
  if(rv == pdPASS) xTaskCreate(sensor_task, "mf_SENSOR", 2 * MF_FREERTOS_DEFAULT_STACK_SIZE, NULL, uxTaskPriorityGet(NULL), &sensor_task_handle);
  if(rv == pdPASS) xTaskCreate(rcl_task, "mf_RCL", 2 * MF_FREERTOS_DEFAULT_STACK_SIZE, NULL, uxTaskPriorityGet(NULL), &rcl_task_handle);
  if(rv == pdPASS) xTaskCreate(cli_task, "mf_CLI", 2 * MF_FREERTOS_DEFAULT_STACK_SIZE, NULL, uxTaskPriorityGet(NULL), &cli_task_handle);
  if(rv != pdPASS) madflight_panic("Task creation failed");
}
