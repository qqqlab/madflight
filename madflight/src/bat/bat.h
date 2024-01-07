/*========================================================================================================================
This file contains all necessary functions and code used for battery monitors to avoid cluttering the main code

Each USE_BAT_xxx section in this file defines:
 - int bat_setup() - init
 - int bat_loop() - updates bat_i, bat_v, bat_mah and bat_wh

========================================================================================================================*/

#pragma once


//curretly only adc battery monitor availble - TODO INA266
#include "adc/adc.h"