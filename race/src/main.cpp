#include "Lane_Detector.h"

Lane_Detector* ld;
race::drive_values control_msg;
ros::NodeHandle nh;
ros::Subscriber sub; 
ros::Publisher control_pub;

void testerCallback(const race::control_variables &msg) {
    ld->set_control_variables(msg.p_slope, msg.p_position);
}
void generate_control_msg(race::drive_values* control_msg) {

}
int main(int argc, char** argv) {
    ros::init(argc, argv, "Controller");
    ld = new Lane_Detector();
    sub = nh.subscribe("control_variables", 1000, testerCallback);
    control_pub = nh.advertise<race::drive_values>("Control", 1000);
    
    while(true) {
        ld->operate();
        generate_control_msg(&control_msg);
        control_pub.publish(control_msg);
        ros::spinOnce();
    }
    delete controller;
    return 0;
}
