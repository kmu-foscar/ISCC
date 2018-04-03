#include <ros/ros.h>

#include <obstacle_detector/Obstacles.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <algorithm>
#include <stdlib.h>
#include <math.h>
#include <std_msgs/Int32.h>
#include <race/drive_values.h>

/* 제어 상수 */
#define K_FD 9 // Foward distance 상수
#define K_RD -180 // Right distance 상수
#define K_DG 1.2  // Difference of Gradient 상수
#define K_V 0.5 // velocity 상수

#define MAX_DISTANCE 5 // FOWARD DISTANCE의 최대 탐지 거리
#define MAX_DISTANCE_STATIC 2.5
#define DISTANCE_RIGHT_OFFSET 0.2

#define DETECT_RANGE 0.55 // 협로, 곡선, 주차용 MODE 1, 2, 4, 5

using namespace std;

ros::Publisher pub;

double distance_from_car(geometry_msgs::Point p1, geometry_msgs::Point p2){
  double gradient = (p1.x - p2.x) / (p1.y - p2.y);
  double distance = abs(-gradient * p1.y + p1.x) / sqrt(1 + gradient * gradient);
  return distance;
}
void calculator(const obstacle_detector::Obstacles data) {
  geometry_msgs::Point c1, c2, s;
  s.x = 50;
  s.y = 0;
  bool flag = false;
  int d_range = DETECT_RANGE;

  for(int i = 0; i < data.circles.size(); i++) {
    if(data.circles[i].center.y > -d_range && data.circles[i].center.y < d_range && data.circles[i].center.x > 0.015 && data.circles[i].center.x < MAX_DISTANCE) {
      if(!flag || s.x > data.circles[i].center.x){
        flag = true;
        s = data.circles[i].center;
      }
    }
  }
  // TODO : c1, c2는 y축 기준 0보다 작아야함
 
  c1 = data.circles[0].center; // 오른쪽에서 가장 가까운 점
  c2 = data.circles[1].center; // 오른쪽에서 두번째로 가까운 점
  double distance_right = distance_from_car(c1, c2); 
  double theta = -atan2(c1.x - c2.x, c1.y - c2.y) / M_PI * 180;
  double distance_front;
  if(flag)
    distance_front = sqrt(s.x * s.x + s.y * s.y);
  else
    distance_front = MAX_DISTANCE;
  double error_dr = (DISTANCE_RIGHT_OFFSET - distance_right);
  double error_theta = 90 - theta;
 
  double error_df = 1 / distance_front;
  if(theta < 0) {
    error_df *= -1;
  }
  
  
  double steer_pid_value = K_RD * error_dr + K_DG * error_theta + K_FD * error_df;
  double velocity_pid_value;
  velocity_pid_value = K_V * distance_front + 5;
 
  velocity_pid_value = min(velocity_pid_value, 10.0);
	
  if(steer_pid_value > 100) steer_pid_value = 100;
  else if(steer_pid_value < -100) steer_pid_value = -100; 
  race::drive_values msg;
  if(data.circles.size() > 4){
    msg.steering = steer_pid_value + 100;
    msg.throttle = 1;
  }
  else{
    msg.steering = 0;
    msg.throttle = 0;
  }
  printf("steering : %d\n", (int)steer_pid_value);
  pub.publish(msg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "Obstacle_Avoider");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("raw_obstacles", 1, calculator);
	pub = nh.advertise<race::drive_values>("Control", 1000);
	ros::spin();
}
