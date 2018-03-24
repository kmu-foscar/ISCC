#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Lane_Detector.h"

class Controller {
private :
    Lane_Detector* ld;
public :
    Controller();
    ~Controller();
   static void testerCallback(const race::control_variables &msg);
    void generate_control_msg(race::drive_values* control_msg);    
};

#endif