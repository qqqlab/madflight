#pragma once

#include "../bbx.h"

class BlackBoxNone : public BlackBox {
};

BlackBoxNone bbx_instance;
BlackBox &bbx = bbx_instance;
