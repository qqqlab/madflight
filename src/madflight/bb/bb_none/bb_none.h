#pragma once

#include "../bb_interface.h"

class BlackBoxNone : public BlackBox {
};

BlackBoxNone bb_instance;
BlackBox &bb = bb_instance;
