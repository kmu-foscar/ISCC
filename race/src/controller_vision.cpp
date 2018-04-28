#include "Lane_Detector.hpp"
#include "Look_Ahead.hpp"
#include <algorithm>
#include <ros/ros.h>
#include <race/drive_values.h>
#include <race/control_variables.h>
#include <signal.h>
#define CENTER_POINT 690
#define CENTER_POINT_LA 320
#define MAX_SPEED 7
Lane_Detector* ld;
Look_Ahead* la;
race::drive_values control_msg;
ros::Subscriber sub; 
ros::Publisher control_pub;
float p_steering = -0.3f;
float p_steering_curve = 20.f;
float p_lookahead_curve = 10.f;
int test_speed = 5;
int speed = MAX_SPEED;
bool onoff = true;
void testerCallback(const race::control_variables &msg);
void generate_control_msg(race::drive_values* control_msg);
float cal_lookahead_op_error();
int main(int argc, char** argv) {
    ros::init(argc, argv, "Lane_Keeper");
    ros::NodeHandle nh;
    
    ld = new Lane_Detector();
    //la = new Look_Ahead();
    sub = nh.subscribe("control_variables", 1000, testerCallback);
    control_pub = nh.advertise<race::drive_values>("Control", 1000);
    ld->init();
    //la->init();
    while(ros::ok()) {
        ld->operate();
        //la->operate();
        generate_control_msg(&control_msg);
        control_pub.publish(control_msg);
        ros::spinOnce();
    }
    delete ld;
    //delete la;
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
    float op_error;
    Point op;
    Point pa_1 = ld->p1;
    Point pa_2 = ld->p2;
    Point pb_1 = ld->p3;
    Point pb_2 = ld->p4;
    
    pb_1.x += 640;
    pb_2.x += 640;

    if(ld->get_intersectpoint(pa_1, pa_2, pb_1, pb_2, &op)) {
        float error_steering = CENTER_POINT - op.x;
        steering = p_steering * error_steering * (float)(1/(float)speed) * 5; 
    } 
    else if(ld->is_left_error()) {
        steering = -p_steering_curve / ld->get_right_slope() * (float)(1/(float)speed) * 5;
    }
    else if(ld->is_right_error()) {
        steering = p_steering_curve / ld->get_left_slope() * (float)(1/(float)speed) * 5;
    }
    else {
        steering = 0;
    }
    steering = min(max(steering, -100), 100);
    printf("steering : %d\n", steering);
    steering += 100;
    //op_error = cal_lookahead_op_error();
    //speed = speed - (int)op_error;
    speed = min(max(speed, 3), MAX_SPEED);
    control_msg->steering = steering;
    control_msg->throttle = speed;
}
float cal_lookahead_op_error() {
    Point op;
    Point pa_1 = ld->p1;
    Point pa_2 = ld->p2;
    Point pb_1 = ld->p3;
    Point pb_2 = ld->p4;
    float error_op;
    
    if(la->get_intersectpoint(pa_1, pa_2, pb_1, pb_2, &op)) {
        error_op = CENTER_POINT_LA - op.x;
    }
    else if(ld->is_left_error()) {
        error_op = CENTER_POINT_LA - p_lookahead_curve / la->get_right_slope();
    }
    else if(ld->is_right_error()) {
        error_op = CENTER_POINT_LA + p_lookahead_curve / la->get_left_slope();
    }
    else {
        error_op = 0;
    }
    return error_op; 
}
