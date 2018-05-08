#include "Lane_Detector.hpp"
#include "Look_Ahead.hpp"
#include "config.h"
#include <algorithm>
#include <ros/ros.h>
#include <obstacle_detector/Obstacles.h>
#include <race/drive_values.h>
#include <std_msgs/Bool.h>
#include <signal.h>

#define CENTER_POINT 690
#define CENTER_POINT_LA 750
#define MAX_SPEED 12

Lane_Detector* ld;
Look_Ahead* la;
race::drive_values control_msg;
ros::Subscriber sub; 
ros::Publisher control_pub;
ros::Publisher return_sig_pub;
float p_steering = -0.3f;
float p_steering_curve = 20.f;
float p_lookahead_curve = 10.f;
float p_lookahead = 0.05f;
int steering;
int throttle;

bool lk_onoff = true;
bool cw_onoff = false;
bool do_onoff = false;
bool so_onoff = false;
bool ut_onoff = false;
bool pk_onoff = false;
int parking_state = 0;

void lk_onoffCallback(const std_msgs::Bool &msg);
void cw_onoffCallback(const std_msgs::Bool &msg);
void do_onoffCallback(const std_msgs::Bool &msg);
void so_onoffCallback(const std_msgs::Bool &msg);
void ut_onoffCallback(const std_msgs::Bool &msg);
void pk_onoffCallback(const std_msgs::Bool &msg);

void keep_lane_advanced(race::drive_values* control_msg); // base mode
void keep_lane(race::drive_values* control_msg); // parking, Dynamic obstacle, static obstacle, Uturn, Crosswalk  
float cal_lookahead_op_error();

int main(int argc, char** argv) {
    ros::init(argc, argv, "Lane_Keeper");
    ros::NodeHandle nh;
    
    ld = new Lane_Detector();
    la = new Look_Ahead();
    lk_onoff_sub = nh.subscirber("lk_onoff_msg", lk_onoffCallback);
    cw_onoff_sub = nh.subscirber("cw_onoff_msg", cw_onoffCallback);
    do_onoff_sub = nh.subscirber("do_onoff_msg", do_onoffCallback);
    so_onoff_sub = nh.subscirber("so_onoff_msg", so_onoffCallback);    
    ut_onoff_sub = nh.subscirber("ut_onoff_msg", ut_onoffCallback);
    pk_onoff_sub = nh.subscirber("pk_onoff_msg", pk_onoffCallback);
    
    control_pub = nh.advertise<race::drive_values>("Control", 1000);
    return_sig_pub = nh.advertise<std_msgs::Int16>("return_signal", 1);
    ld->init();
    la->init();
    while(ros::ok()) {
        if(lk_onoff) {
            ld->operate();
            la->operate(ld->originImg_left, ld->originImg_right);
            keep_lane_advanced(&control_msg);
        }
        else if(cw_onoff) {
            ld->operate();
        }
        else if(do_onoff) {
            ld->operate();
        }
        else if(so_onoff) {
            ld->operate();
        }
        else if(ut_onoff) {
            ld->operate();
        }
        else if(pk_onoff) {
            ld->operate();
        }

        control_pub.publish(control_msg);
        ros::spinOnce();
    }
    delete ld;
    delete la;
    return 0;
}

void lk_onoffCallback(const std_msgs::Bool &msg) {
    lk_onoff = msg.data;
}
void cw_onoffCallback(const std_msgs::Bool &msg) {
    cw_onoff = msg.data;
}
void do_onoffCallback(const std_msgs::Bool &msg) {
    do_onoff = msg.data;
}
void so_onoffCallback(const std_msgs::Bool &msg) {
    so_onoff = msg.data;
}
void ut_onoffCallback(const std_msgs::Bool &msg) {
    ut_onoff = msg.data;
}
void pk_onoffCallback(const std_msgs::Bool &msg) {
    pk_onoff = msg.data;
}
// lookahead 포함 lane keeping 함수.
void keep_lane_advanced(race::drive_values* control_msg) {
    int speed = MAX_SPEED;
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
    
    steering = min(max(steering, -100), 100);
    //printf("steering : %d\n", steering);
    steering += 100;
    op_error = cal_lookahead_op_error();
    //printf("lookahead : %f\n", op_error);
    speed = (int)round((float)speed - fabs(op_error * p_lookahead));
    speed = min(max(speed, 5), MAX_SPEED);
    printf("speed : %d\n", speed);
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
    pa_1.x += 200;
    pa_2.x += 200;
    pb_1.x += 640;
    pb_2.x += 640;
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

void keep_lane(race::drive_values* control_msg) {
    int speed = MAX_SPEED / 2;
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
    
    steering = min(max(steering, -100), 100);
    steering += 100;
    control_msg->steering = steering;
    control_msg->throttle = speed;
}