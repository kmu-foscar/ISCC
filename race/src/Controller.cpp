#include "Controller.h"

Controller::Controller() {
    ld = new Lane_Detector();
}
Controller::~Controller() {
    delete ld;
}
void Controller::testerCallback(const race::control_variables &msg) {
    ld->set_control_variables(msg.p_slope, msg.p_position);
}
void Controller::generate_control_msg() {
    ld->operate();   
}

