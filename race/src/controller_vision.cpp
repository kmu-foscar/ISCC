#include "Lane_Detector.hpp"
#include "Look_Ahead.hpp"
#include "config.h"
#include <algorithm>
#include <ros/ros.h>
#include <obstacle_detector/Obstacles.h>
#include <race/drive_values.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <signal.h>
#include <time.h>

#define CENTER_POINT 690
#define CENTER_POINT_LA 750
#define MAX_SPEED 12
#define PARKING_STATE_1_THRESHOLD 100
#define PARKING_STATE_2_THRESHOLD 100
#define PARKING_STATE_3_THRESHOLD 100
#define PARKING_STATE_4_THRESHOLD 100
#define PARKING_STATE_5_THRESHOLD 100
#define UTURN_THRESHOLD 1.5
using namespace std;

Lane_Detector* ld;
Look_Ahead* la;
race::drive_values control_msg;
std_msgs::Int16 return_msg;
ros::Subscriber sub; 
ros::Subscriber do_sub;
ros::Subscriber lk_onoff_sub;
ros::Subscriber cw_onoff_sub;
ros::Subscriber do_onoff_sub;
ros::Subscriber so_onoff_sub;
ros::Subscriber ut_onoff_sub;
ros::Subscriber pk_onoff_sub;

ros::Publisher control_pub;
ros::Publisher return_sig_pub;

// variables for dynamic obstacle 
int do_cnt = 0;
int obstacle_size;
obstacle_detector::Obstacles obstacles_data;
clock_t begin, end;

// variables for lane keeper
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
int parking_state;
int uturn_state;
bool is_parked;
void lk_onoffCallback(const std_msgs::Bool &msg);
void cw_onoffCallback(const std_msgs::Bool &msg);
void do_onoffCallback(const std_msgs::Bool &msg);
void so_onoffCallback(const std_msgs::Bool &msg);
void ut_onoffCallback(const std_msgs::Bool &msg);
void pk_onoffCallback(const std_msgs::Bool &msg);
void obstacleCallback(const obstacle_detector::Obstacles data); // dynamic obstacle callback
double obstacleDistance(geometry_msgs::Point &p1, geometry_msgs::Point &p2);
void keep_lane_advanced(race::drive_values* control_msg); // base mode
void keep_lane(race::drive_values* control_msg); // parking, Dynamic obstacle, static obstacle, Uturn, Crosswalk  
void do_operate();
void pk_operate();
void ut_operate();
float cal_lookahead_op_error();

int main(int argc, char** argv) {
    ros::init(argc, argv, "Lane_Keeper");
    ros::NodeHandle nh;
    
    ld = new Lane_Detector();
    la = new Look_Ahead();
    lk_onoff_sub = nh.subscribe("lk_onoff_msg", 1, lk_onoffCallback);
    cw_onoff_sub = nh.subscribe("cw_onoff_msg", 1, cw_onoffCallback);
    do_onoff_sub = nh.subscribe("do_onoff_msg", 1, do_onoffCallback);
    so_onoff_sub = nh.subscribe("so_onoff_msg", 1, so_onoffCallback);    
    ut_onoff_sub = nh.subscribe("ut_onoff_msg", 1, ut_onoffCallback);
    pk_onoff_sub = nh.subscribe("pk_onoff_msg", 1, pk_onoffCallback);
    
    do_sub = nh.subscribe("raw_obstacles", 1, obstacleCallback);

    control_pub = nh.advertise<race::drive_values>("Control", 1000);
    return_sig_pub = nh.advertise<std_msgs::Int16>("return_signal", 1);
    ld->init();
    la->init();
    parking_state = 0;
    uturn_state = 0;
    while(ros::ok()) {
        if(lk_onoff) {
            ld->operate();
            la->operate(ld->originImg_left, ld->originImg_right);
            keep_lane_advanced(&control_msg);
        }
        else if(cw_onoff) {
            // ld->operate();
            // Nayeon 
        }
        else if(do_onoff) {
            ld->operate();
            keep_lane(&control_msg);
            do_operate();
        }
        else if(so_onoff) {
            ld->operate();
            keep_lane(&control_msg);
            // Hanjeong
        }
        else if(ut_onoff) {
            ut_operate();
        }
        else if(pk_onoff) {
            pk_operate();
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

bool isExist(int do_cnt) {
  if(do_cnt > 80) 
  	return false;
  return true;
}
void obstacleCallback(const obstacle_detector::Obstacles data) {
    obstacle_size = data.circles.size();
    obstacles_data = data;
}
void ut_operate() {
    bool flag = false;
    geometry_msgs::Point s;
    switch (uturn_state) {
    case 0 :
    ld->operate();
    keep_lane(&control_msg);    
    for(int i = 0; i < obstacle_size; i++) {
        if(obstacles_data.circles[i].center.y > -0.55 && obstacles_data.circles[i].center.y < 0.55 && obstacles_data.circles[i].center.x > 0.015 && obstacles_data.circles[i].center.x < 5) {
            if(!flag || s.x > obstacles_data.circles[i].center.x){
                flag = true;
                s = obstacles_data.circles[i].center;
            }
        }
    }
    
    if(s.x <= UTURN_THRESHOLD) {
        uturn_state = 1;
    }
    break;

    case 1 :
    control_msg.steering = 0;
    control_msg.throttle = 5;
    if(obstacle_size == 0) {
        uturn_state = 2;
    }
    break;

    case 2 :
    control_msg.steering = 200;
    control_msg.throttle = 5;
    ld->operate();
    if(!ld->is_left_error() && !ld->is_right_error()) {
        return_msg.data = MODE_UTURN;
        return_sig_pub.publish(return_msg);
    }
    break;
    }
}
void do_operate() {
  bool exist_obstacle;
  obstacle_size > 0? exist_obstacle = true : exist_obstacle = false;

  if(exist_obstacle) {
   	printf("%s\n" , "stop!!");
	do_cnt = 0;
    control_msg.throttle = 0;
  }
  else do_cnt++;
	
  if(!isExist(do_cnt)) {
	do_cnt = 0;
    printf("%s\n" , "go!!");
    return_msg.data = MODE_DYNAMIC_OBSTACLE;
    return_sig_pub.publish(return_msg);
    control_msg.throttle = 7;
  }
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


void pk_operate() {
    switch (parking_state) {
    case -1:
    end = clock();
    if((end - begin)/CLOCKS_PER_SEC >= 2.5f) {
        if(is_parked){
            parking_state = 5;
        }
        parking_state = 1;
    }
    break;

    case 0 : 
    ld->parking_init();
    ld->operate();
    keep_lane(&control_msg);
    if(ld->parking_point2.y < PARKING_STATE_1_THRESHOLD) {
        parking_state = -1;
        begin = clock();
        control_msg.steering = 200; // right max steer
        control_msg.throttle = 5;
    }
    break;

    case 1 :
    ld->operate();
    if(!ld->is_left_error() && !ld->is_right_error()){
        parking_state = 2;
    }
    break;

    case 2 :
    ld->operate();
    keep_lane(&control_msg);
    if(ld->stop_parking.y > PARKING_STATE_2_THRESHOLD) {
        control_msg.throttle = 0;
        is_parked = true;
        parking_state = 3;
    }
    break;

    case 3 :
    ros::Duration(10).sleep(); 
    parking_state = 4;
    break;

    case 4 :
    ld->operate();
    keep_lane(&control_msg);
    control_msg.throttle = -5;
    if(ld->stop_parking.y < PARKING_STATE_4_THRESHOLD) {
        parking_state = -1;
        begin = clock();
        control_msg.steering = 200; // right max steer
        control_msg.throttle = -5;
    }
    break;

    case 5 :
    ld->operate();
    if(!ld->is_left_error() && !ld->is_right_error()){
        return_msg.data = MODE_PARKING;
        return_sig_pub.publish(return_msg);
        ld->parking_release();
    }
    break;    
    }
}

