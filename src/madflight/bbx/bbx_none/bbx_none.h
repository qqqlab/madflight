#pragma once

#include "../bbx_interface.h"

class BlackBoxNone : public BlackBox {
};

BlackBoxNone bbx_instance;
BlackBox &bbx = bbx_instance;
