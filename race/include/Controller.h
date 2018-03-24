#include "Lane_Detector.h"

class Controller {
public :
    void testerCallback(const race::control_variables &msg);
    void generate_control_msg(race::drive_values* control_msg);    
};