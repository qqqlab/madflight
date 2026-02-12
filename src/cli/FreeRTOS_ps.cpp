/*==========================================================================================
FreeRTOS_ps.h - FreeRTOS Process List

MIT License

Copyright (c) 2024 https://madflight.com

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

#define FREERTOS_PS_MAX_TASKS 20 //max number of tasks to track

#include "FreeRTOS_ps.h"
#include "../hal/hal.h" //#include <FreeRTOS.h> and #include <task.h> (different platforms use different headers...)

#if configUSE_TRACE_FACILITY != 1
void freertos_ps(Print &p) {
  p.println("FreeRTOS_ps() needs configUSE_TRACE_FACILITY = 1 for uxTaskGetSystemState()");
}
#else

static char freertos_taskStatusChar(eTaskState eCurrentState) {
  switch(eCurrentState)
  {
    case eRunning:   return 'X';
    case eReady:     return 'R';
    case eBlocked:   return 'B';
    case eSuspended: return 'S';
    case eDeleted:   return 'D';
    default:         return '-'; // Should not get here, but it is included to prevent static checking errors.
  }
}

void freertos_ps(Print &p)
{
  static struct taskarr_s {
    TaskStatus_t pxTaskStatusArray[FREERTOS_PS_MAX_TASKS];
    uint32_t uxArraySize = 0;
    uint32_t ulTotalRunTime = 0;
  } taskarr[2];
  static uint8_t taskarrIdx = 0;
  static uint32_t ts_start = 0;

  taskarr_s &taold = taskarr[taskarrIdx];
  taskarrIdx = (taskarrIdx + 1) % 2;
  taskarr_s &tanew = taskarr[taskarrIdx];

  uint32_t now = micros();
  float dt = 1e-6 * (now - ts_start);
  ts_start = now;
  tanew.uxArraySize = uxTaskGetSystemState( tanew.pxTaskStatusArray, FREERTOS_PS_MAX_TASKS,  &tanew.ulTotalRunTime );

  uint32_t totalRunTime = tanew.ulTotalRunTime - taold.ulTotalRunTime;
  uint64_t tot = 0;

  p.printf("\n=== FreeRTOS Tasks - Measurement Period: %.2f seconds ===\n\n", dt);
  p.print("TID Name            CPU%  Free St Pr Ni");
  #if ( ( configUSE_CORE_AFFINITY == 1 ) && ( configNUMBER_OF_CORES > 1 ) )
    p.print(" Core");
  #endif
  p.println();

  float perc_sum = 0;
  for( uint32_t i = 0; i < tanew.uxArraySize; i++ )
  {
    TaskStatus_t &t = tanew.pxTaskStatusArray[i];

    uint32_t runtime = t.ulRunTimeCounter;
    for(uint32_t j=0;j<taold.uxArraySize;j++) {
      TaskStatus_t &told = taold.pxTaskStatusArray[j];
      if(told.xTaskNumber == t.xTaskNumber) {
        runtime -= (uint32_t)told.ulRunTimeCounter;
      }
    }
    float perc = (float)runtime / totalRunTime * 100;
    perc_sum += perc;
    p.printf("%3d %-12s%7.2f%% %5d  %c %2d %2d",
      (int)t.xTaskNumber,
      t.pcTaskName,
      (float)perc,
      (int)t.usStackHighWaterMark * sizeof(StackType_t),
      freertos_taskStatusChar(t.eCurrentState),
      (int)t.uxBasePriority,
      (int)t.uxCurrentPriority
    );
    #if ( ( configUSE_CORE_AFFINITY == 1 ) && ( configNUMBER_OF_CORES > 1 ) )
      p.printf(" %2lX",
        t.uxCoreAffinityMask & ((1<<configNUMBER_OF_CORES)-1)
      );
    #endif
    p.printf("\n");
    tot += runtime;
  }
  p.printf("    Total       %7.2f%%\n", perc_sum);
}
#endif //configUSE_TRACE_FACILITY != 1
