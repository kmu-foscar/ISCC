#include "Controller.h"

Lane_Detector* ld;

void Controller::testerCallback(const race::control_variables &msg) {
    ld->set_control_variables(msg.p_slope, msg.p_position);
}