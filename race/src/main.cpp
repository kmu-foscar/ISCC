#include "Controller.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "Controller");
    ros::NodeHandle nh;
    Controller* controller = new Controller();
    ros::Subscriber sub = nh.subscribe("control_variables", 1000, &(controller->testerCallback));
    ros::Publisher control_pub = nh.advertise<race::drive_values>("Control", 1000);
    race::drive_values control_msg;
    while(true) {
        controller->generate_control_msg(&control_msg);
        control_pub.publish(control_msg);
        ros::spinOnce();
    }
    delete controller;
    return 0;
}
