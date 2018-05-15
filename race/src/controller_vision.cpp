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

#define CENTER_POINT 640
#define CENTER_POINT_LA 640
#define MAX_SPEED 12

#define PARKING_STATE_0_THRESHOLD 50.f
#define PARKING_STATE_2_THRESHOLD 50.f
#define PARKING_STATE_4_THRESHOLD 50.f
#define PARKING_LIDAR_THRESHOLD 5.f
#define UTURN_LIDAR_THRESHOLD 3.5
#define UTURN_VISION_THRESHOLD 50
#define CROSSWALK_THRESHOLD 200
#define STATIC_OBSTACLE_THRESHOLD 30
#define MAX 5
#define PI 3.1415
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
double begin, end;
// variables for lane keeper
float p_steering = -0.3f;
float p_steering_curve = 20.f;
float p_lookahead_curve = 10.f;
float p_lookahead = 0.05f;

// variables for static obstacle
int so_cnt = 0;
int isLeftObstacle, isRightObstacle;
bool isLeftStaticPass = false;
bool isRightStaticPass = false;
int cnt = 0;
int nayeon = -1;
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
int ut_cnt = 0;
int crosswalk_state;
int static_obstacle_state;
bool is_parked;
bool is_front_parking;
int cnt_pk = 0;
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
void cw_operate();
void so_operate();

float cal_lookahead_op_error();

int main(int argc, char** argv) {
    ros::init(argc, argv, "Lane_Keeper");
    ros::NodeHandle nh;

    ld = new Lane_Detector();
    la = new Look_Ahead();
    lk_onoff_sub = nh.subscribe("lk_onoff", 1, lk_onoffCallback);
    cw_onoff_sub = nh.subscribe("cw_onoff", 1, cw_onoffCallback);
    do_onoff_sub = nh.subscribe("do_onoff", 1, do_onoffCallback);
    so_onoff_sub = nh.subscribe("so_onoff", 1, so_onoffCallback);
    ut_onoff_sub = nh.subscribe("ut_onoff", 1, ut_onoffCallback);
    pk_onoff_sub = nh.subscribe("pk_onoff", 1, pk_onoffCallback);
    do_sub = nh.subscribe("raw_obstacles", 1, obstacleCallback);

    control_pub = nh.advertise<race::drive_values>("Control", 1000);
    return_sig_pub = nh.advertise<std_msgs::Int16>("return_signal", 1);
    ld->init();
    la->init();
    parking_state = 0;
    is_parked = false;
    is_front_parking = false;
    uturn_state = 0;
    static_obstacle_state = 0;
    isLeftObstacle = 0;
    isRightObstacle = 0;
    while(ros::ok()) {
        if(lk_onoff) {
            printf("lane_keeping_mode\n");
            ld->operate();
            la->operate(ld->originImg_left, ld->originImg_right);
            keep_lane_advanced(&control_msg);
            control_pub.publish(control_msg);
        }
        else if(cw_onoff) {
            printf("crosswalk_mode\n");
            cw_operate();
            control_pub.publish(control_msg);
        }
        else if(do_onoff) {
            printf("dynamic_obstacle_mode\n");
            ld->operate();
            keep_lane(&control_msg);
            do_operate();
            control_pub.publish(control_msg);
        }
        else if(so_onoff) {
            printf("static_obstacle_mode\n");
            ld->operate();
            so_operate();
            keep_lane(&control_msg);
            control_pub.publish(control_msg);
        }
        else if(ut_onoff) {
            printf("uturn_mode\n");
            ut_operate();
	    printf("uturun state : %d\n", uturn_state);
            control_pub.publish(control_msg);
        }
        else if(pk_onoff) {
            printf("parking_mode\n");
            printf("%d\n", parking_state);
            pk_operate();
            control_pub.publish(control_msg);
        }
	else {
	    ld->operate();
	}
        ros::spinOnce();
    }
    delete ld;
    delete la;
    return 0;
}

void lk_onoffCallback(const std_msgs::Bool &msg) {
    if(lk_onoff != msg.data)
    {
      ld->mode_change();
    }
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
    if(pk_onoff != msg.data) {
	if(msg.data) {
	    ld->parking_init();
	}
    }
    pk_onoff = msg.data;
}

bool isExist(int do_cnt) {
  if(do_cnt > 50)
  	return false;
  return true;
}
void obstacleCallback(const obstacle_detector::Obstacles data) {

	obstacle_size = data.circles.size();
    
  for(int i = 0; i < data.circles.size(); i++)
  {
      geometry_msgs::Point curPoint = data.circles[i].center;

      //x축 대칭
      curPoint.y = -curPoint.y;

      //y = x 대칭
      swap(curPoint.x, curPoint.y);

      if(curPoint.y == 0.0 || curPoint.x == 0.0)
          continue;

      double angle = atan2(curPoint.y, curPoint.x);
      
      //1,2사분면이 아니면
      if(angle < 0.0){
          obstacle_size--;
	  continue;
      }
      if((angle < -0.29 + 1.57) || (angle > 0.29 + 1.57))
          obstacle_size--;
   }
    obstacles_data = data;
}
void ut_operate() {
    bool flag = false;
    geometry_msgs::Point s;
    switch (uturn_state) {
    case 0 :
    
    ld->operate();
    keep_lane(&control_msg);
    printf("obstacle size : %d\n", obstacle_size);
    for(int i = 0; i < obstacle_size; i++) {
        if(obstacles_data.circles[i].center.y > -0.55f && obstacles_data.circles[i].center.y < 0.55f && obstacles_data.circles[i].center.x > 0.015f && obstacles_data.circles[i].center.x < 5.f) {
            if(!flag || s.x > obstacles_data.circles[i].center.x){
                flag = true;
                s = obstacles_data.circles[i].center;
            }
        }
    }
    printf("%d\n", s.x);
    if(flag && s.x <= UTURN_LIDAR_THRESHOLD) {
        uturn_state = 1;
    }
    break;

    case 1 :
    ld->operate();
    control_msg.steering = 0; // max left steering
    control_msg.throttle = 5; 
    if(obstacle_size < 5) {
        ut_cnt++;
        if(ut_cnt >= 40) {
            uturn_state = 2;
        }
    }
    else {
        ut_cnt = 0;
    }
    break;

    case 2 :
    control_msg.steering = 0;
    control_msg.throttle = 5;
    ld->uturn_mode_onoff = true;
    ld->operate();
    ld->stop_line();
    printf("stop_y : %d\n", ld->stop_y); 
    if(ld->stop_y >= UTURN_VISION_THRESHOLD) {
        uturn_state = 3;
	ld->uturn_mode_onoff = false;
    }
    break;

    case 3 :
    control_msg.steering = 200;
    control_msg.throttle = 5;
    ld->operate();
    if(!ld->is_left_error() && !ld->is_right_error()) {
        return_msg.data = RETURN_FINISH;
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
    return_msg.data = RETURN_FINISH;
    return_sig_pub.publish(return_msg);
    control_msg.throttle = 7;
  }
}
void cw_operate() {
	switch(crosswalk_state) {
	case 0 :
	ld->operate();
	keep_lane(&control_msg);
	ld->stop_line();
	if(ld->stop_y >= CROSSWALK_THRESHOLD){
		crosswalk_state = 1;
		control_msg.throttle = 0;
	}
	break;

	case 1 :
	ld->operate();
	ros::Duration(3).sleep();
	return_msg.data = RETURN_FINISH;
    return_sig_pub.publish(return_msg);
	break;
	}
}
void pk_operate() {
    geometry_msgs::Point car;
    car.x = 0; car.y = 0;
    int dist_car = 0;
    switch (parking_state) {
    case -1:
    control_msg.steering = 200; // right max steer
    control_msg.throttle = 5;
    end = ros::Time::now().toSec();
    if(end - begin >= 2.5f) {
        if(is_parked){
            parking_state = 5;
        }
        parking_state = 1;
    }
    break;

    case 0 :
    ld->operate();
    keep_lane(&control_msg);
    if(!is_front_parking && ld->parking_point1.y > 0) {
        for(int i = 0; i < obstacle_size; i++) {
            if(obstacles_data.circles[i].center.x > 0 && obstacleDistance(car, obstacles_data.circles[i].center) < PARKING_LIDAR_THRESHOLD) {
                ld->parking_release();
                is_front_parking = true;
		printf("front parking\n");
                parking_state = 6;
                break;
            }
        }
    }
    printf("stop_y : %f\n", ld->parking_point2.y);
    //printf("stop_y : %f\n", ld->cross_y);
    if(ld->parking_point2.y >= PARKING_STATE_0_THRESHOLD) {
        parking_state = -1;
	    ld->parking_state = 1;
        begin = ros::Time::now().toSec();
    }
    break;

    case 1 :
    ld->operate();
    control_msg.steering = 200; // right max steer
    control_msg.throttle = 5;
    if(!ld->is_left_error() && !ld->is_right_error()){
        parking_state = 2;
    }
    break;

    case 2 :
    ld->operate();
    keep_lane(&control_msg);
    if(ld->stop_parking.y >= PARKING_STATE_2_THRESHOLD) {
        control_msg.throttle = 0;
        is_parked = true;
        begin = ros::Time::now().toSec();
        parking_state = 3;
    }
    break;

    case 3 :
    ld->operate();
    control_msg.throttle = 0;
    control_msg.steering = 100;
    end = ros::Time::now().toSec();
    if(end - begin >= 10.f) {
        parking_state = 4;
    }
    break;

    case 4 :
    ld->operate();
    keep_lane(&control_msg);
    control_msg.throttle = -5;
    if(ld->stop_parking.y < PARKING_STATE_4_THRESHOLD) {
        parking_state = -1;
        begin = ros::Time::now().toSec();
        control_msg.steering = 200; // right max steer
        control_msg.throttle = -5;
    }
    break;

    case 5 :
    ld->parking_state = 2;    
    ld->operate();
    if(!ld->is_left_error() && !ld->is_right_error()){
        return_msg.data = RETURN_FINISH;
        return_sig_pub.publish(return_msg);
        ld->parking_release();
    }
    break;

    case 6 :
    ld->operate();
    keep_lane(&control_msg);
    if(obstacle_size == 0) {
        ld->parking_init();
        parking_state = 0;
    }
    break;
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
    const int Xshift = 350;
    const int Xspeed = 5;
    float op_error;
    Point op;
    Point pa_1 = ld->p1;
    Point pa_2 = ld->p2;
    Point pb_1 = ld->p3;
    Point pb_2 = ld->p4;

    pb_1.x += 640;
    pb_2.x += 640;

    if(so_onoff){
      //오른쪽 장애물 발견 시 오른쪽 카메라 차선을 Xshift 만큼 왼쪽으로 이동
      if(isRightObstacle == 1)
      {
          isRightStaticPass = true;
          speed = Xspeed;
          pb_1.x -= Xshift;
          pb_2.x -= Xshift;
      }
      //왼쪽 장애물 발견 시 왼쪽 카메라 차선을 Xshift 만큼 오른쪽으로 이동
      if(isLeftObstacle == 1)
      {
          isLeftStaticPass = true;
          speed = Xspeed;
          pa_1.x += Xshift;
          pa_2.x += Xshift;
      }
    }

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

void so_operate(){
  geometry_msgs::Point car;
  car.y = 0.0, car.x = 0.0;

  geometry_msgs::Point closestLeftPoint, closestRightPoint;
  closestLeftPoint.x = MAX, closestRightPoint.y = MAX;
  closestLeftPoint.y = MAX, closestRightPoint.x = MAX;

  double minimumL = MAX, minimumR = MAX;
  const double rightTheta = 0.52; // 30도
  double leftTheta = PI - rightTheta;

  //앞까지의 거리
  for(int i = 0; i < obstacles_data.circles.size(); i++)
  {
      geometry_msgs::Point curPoint = obstacles_data.circles[i].center;
      geometry_msgs::Point temp;

      //x축 대칭
      curPoint.y = -curPoint.y;

      //y = x 대칭
      swap(curPoint.x, curPoint.y);

      if(curPoint.y == 0.0 || curPoint.x == 0.0)
          continue;

      double angle = atan2(curPoint.y, curPoint.x);
      double dist = (curPoint.y) * (curPoint.y) + (curPoint.x) * (curPoint.x);

      //1,2사분면이 아니면
      if(angle < 0.0)
          continue;

      //30도이상 오른쪽
      else if(PI / 2 > angle && angle > rightTheta)
      {
          //오른쪽
          if(minimumR > dist) {
              minimumR = dist;
              closestRightPoint = curPoint;
          }
      }
      //120도이하 왼쪽
      else if(PI / 2 < angle && angle < leftTheta)
      {
          //왼쪽
          if(minimumL > dist){
              minimumL = dist;
              closestLeftPoint = curPoint;
          }
      }
  }

  //둘다 잡지 못했거나, 하나가 깜빡이면
  if(minimumL == minimumR)
  {
      if((isLeftObstacle || isRightObstacle) && cnt < 10)
      {
          cnt++;
          return;
      }
      isLeftObstacle = 0;
      isRightObstacle = 0;
      cnt = 0;
  }
  if(minimumL < minimumR) //왼쪽 장애물이 오른쪽 장애물보다 가까운데
  {
      if(isRightObstacle && cnt < 10) // 오른쪽 장애물이 깜박거려서 못본거였으면
      {
          cnt++;
          return; //그대로
      }
      if(isRightObstacle) //실제로 안보이게 된거라면
          isRightObstacle = 0;

      cnt = 0;
      isLeftObstacle = 1;
  }
  if(minimumR < minimumL) //오른쪽 장애물이 왼쪽 장애물보다 가까운데
  {
      if(isLeftObstacle && cnt < 10) // 왼쪽 장애물이 깜박였던거라면
      {
          cnt++;
          return; //지금 boolean 그대로
      }
      if(isLeftObstacle)
          isLeftObstacle = 0;
      cnt = 0;
      isRightObstacle = 1;
  }

  if(isLeftObstacle)
      printf("Find LeftObstacle! distance Obstacle : %d \n", closestLeftPoint.x);
  if(isRightObstacle)
      printf("Find RightObstacle! distance Obstacle : %d \n", closestRightPoint.x);

  switch(static_obstacle_state){
    case 0:
    if(isLeftStaticPass || isRightStaticPass){
      static_obstacle_state = 1;
    }
    break;

    case 1:
    if(isLeftStaticPass && isRightStaticPass){
      static_obstacle_state = 2;
    }
    break;

    case 2:
    if(so_cnt > STATIC_OBSTACLE_THRESHOLD){
      return_msg.data = RETURN_FINISH;
      return_sig_pub.publish(return_msg);
    }

    if(minimumL == minimumR)
      ++so_cnt;
    else
      so_cnt = 0;

    break;
  }
}

double obstacleDistance(geometry_msgs::Point &p1, geometry_msgs::Point &p2) {
    return sqrt((p2.y - p1.y) * (p2.y - p1.y) + (p2.x - p1.x) * (p2.x - p1.x));
}
