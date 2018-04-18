#include "Lane_Detector.hpp"
#include "Look_Ahead.hpp"
#include <algorithm>
#include <ros/ros.h>
#include <race/drive_values.h>
#include <race/control_variables.h>
#include <signal.h>
#define CENTER_POINT 690
#define DEFAULT_SPEED 12
Lane_Detector* ld;
Look_Ahead* la;
race::drive_values control_msg;
ros::Subscriber sub; 
ros::Publisher control_pub;
float p_steering = -0.3f;
float p_steering_curve = 100.f;
int test_speed = 5;
int speed = DEFAULT_SPEED;
bool onoff = true;
void testerCallback(const race::control_variables &msg);
void generate_control_msg(race::drive_values* control_msg);

int main(int argc, char** argv) {
    ros::init(argc, argv, "Lane_Keeper");
    ros::NodeHandle nh;
    
    ld = new Lane_Detector();
    la = new Look_Ahead();
    sub = nh.subscribe("control_variables", 1000, testerCallback);
    control_pub = nh.advertise<race::drive_values>("Control", 1000);
    ld->init();
    la->init();
    while(ros::ok()) {
        ld->operate();
        la->operate();
        generate_control_msg(&control_msg);
        control_pub.publish(control_msg);
        ros::spinOnce();
    }
    delete ld;
    delete la;
    return 0;
}

void testerCallback(const race::control_variables &msg) {
    p_steering = msg.p_steering;
	p_steering_curve = msg.p_steering_curve;
    test_speed = msg.test_speed;
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
        float error_steering = CENTER_POINT - op.x;
        steering = p_steering * error_steering * (1/speed) * 5; 
    } 
    else if(ld->is_left_error()) {
        steering = -p_steering_curve * ld->get_right_slope() * (1/speed) * 5;
    }
    else if(ld->is_right_error()) {
        steering = p_steering_curve * ld->get_left_slope() * (1/speed) * 5;
    }
    else {
        steering = 0;
    }
    steering = min(max(steering, -100), 100);
    printf("steering : %d\n", steering);
    steering += 100;
    control_msg->steering = steering;
    control_msg->throttle = test_speed;
}
