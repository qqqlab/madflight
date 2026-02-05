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
/*

Example output:

TID Name           CPU% Free S PR NI Core
  2 tCORE0         0.4%  728 X  4  4  1
  6 tIDLE1        99.8%  226 X  0  0  3
  5 tIDLE0        97.3%  228 R  0  0  3
  1 tUSB           2.5%  190 B  6  6  1
  3 tIdleCore0     0.0%   98 B  7  7  1
  4 tIdleCore1     0.0%   98 B  7  7  2
  7 tTmr Svc       0.0%  992 B  2  2  3

TID:  A number unique to the task. Note that this is not the task number that may be modified using vTaskSetTaskNumber() and uxTaskGetTaskNumber(), but a separate TCB-specific and unique identifier automatically assigned on task generation
Name: Task name. This value will be invalid if the task was deleted since the structure was populated!
CPU%: CPU core usage since last call to ps. On a dual core system CPU% should add up to 200%
Free: The minimum amount of stack space (in bytes) that has remained for the task since the task was created.  The closer this value is to zero the closer the task has come to overflowing its stack.
St:   Task State: X=Running, R=Ready, B=Blocked, S=Suspended, D=Deleted
Pr:   The priority to which the task will return if the task's current priority has been inherited to avoid unbounded priority inversion when obtaining a mutex.  Only valid if configUSE_MUTEXES is defined as 1 in FreeRTOSConfig.h.
Ni:   The priority at which the task was running.
Core: The core affinity mask for the task. 1=task will run only on core0, 2=only on core1, 3=both core0 and core1

NOTE: on RP2350/RP2040 the 32bit trace timer is running at F_CPU, with wraps in 30 seconds at 150MHz. Only when the calls between ps are less than 30 seconds you get reliable results for CPU%, otherwise it might look like this:

TID Name           CPU%  Free St Pr Ni Core
  8 IMU          557.2%   425  X  7  7  2
  2 CORE0        289.2%   592  X  4  4  1
  5 IDLE0         70.5%   230  R  0  0  3
  6 IDLE1         68.4%   230  R  0  0  3
  1 USB           81.0%   192  B  6  6  1
  4 IdleCore1      0.0%    96  B  7  7  2
  3 IdleCore0      0.0%    96  B  7  7  1
  7 Tmr Svc        0.0%   992  B  2  2  3

*/

#pragma once

void freertos_ps();
