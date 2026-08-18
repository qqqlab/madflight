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

#include "pid.h"
#include "PidGizmoEXPERIMENTAL.h"
#include "PidGizmoBASIC.h"
#include <Arduino.h>

void Pid::setup() {
    delete gizmo;
    gizmo = nullptr;
    switch(cfg.pid_gizmo) {
        case Cfg::pid_gizmo_enum::mf_BASIC:
            gizmo = new PidGizmoBASIC();
            break;
        case Cfg::pid_gizmo_enum::mf_EXPERIMENTAL:
            gizmo = new PidGizmoEXPERIMENTAL();
            break;
    }
    if(!gizmo) {
        Serial.printf("PID: ERROR invalid pid_gizmo\n");
    }else{
        Serial.printf("PID: %s\n",gizmo->name());
    }
}

Pid pid;
