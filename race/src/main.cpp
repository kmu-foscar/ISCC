#include "Lane_Detector.hpp"
#include <algorithm>
Lane_Detector* ld;
race::drive_values control_msg;
ros::NodeHandle nh;
ros::Subscriber sub; 
ros::Publisher control_pub;
float p_steering = 0.05f;

void testerCallback(const race::control_variables &msg);
void generate_control_msg(race::drive_values* control_msg);

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
    delete ld;
    return 0;
}

void testerCallback(const race::control_variables &msg) {
    p_steering = msg.p_steering;
    p_throttle = msg.p_throttle;
}
void generate_control_msg(race::drive_values* control_msg) {
    int steering;
    int throttle;
    Point op;
    Point pa_1 = ld->p1;
    Point pa_2 = ld->p2;
    Point pb_1 = ld->p3;
    Point pb_2 = ld->p4;
    pb_1.x += 640;
    pb_2.x += 640;

    if(ld->get_intersectpoint(pa_1, pa_2, pb_1, pb_2, &op)) {
        float error_steering = 640 - op.x;
        steering = p_steering * error_steering; 
    } 
    else if(ld->is_left_error()) {
        steering = -30 * ld->get_right_slope();
    }
    else if(ld->is_right_error()) {
        steering = 30 * ld->get_left_slope();
    }
    else {
        steering = 0;
    }
    steering = min(max(steering, -100), 100);
    steering += 100;
    control_msg->steering = steering;
    control_msg->throttle = 1;
}
