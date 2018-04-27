#include <ros/ros.h>

#include <obstacle_detector/Obstacles.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <algorithm>
#include <stdlib.h>
#include <math.h>
#include <std_msgs/Int32.h>
#include <race/drive_values.h>
#include <string>

ros::Publisher pub;
race::drive_values msg;
int cnt = 0;

bool isExist(int cnt) {
  if(cnt > 80) 
  	return false;
  return true;
}

void detect_obstacle(const obstacle_detector::Obstacles data) {
  
  bool exist_obstacle;
  int size = data.circles.size();

  size > 0? exist_obstacle = true : exist_obstacle = false;

  if(exist_obstacle) {
   	printf("%s\n" , "stop!!");
	cnt = 0;
    msg.throttle = 0;
  }
  else cnt++;
	
  if(!isExist(cnt)) {
	cnt = 0;
    printf("%s\n" , "go!!");
    msg.throttle = 1;
  }
  pub.publish(msg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "Obstacle_Avoider");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("raw_obstacles", 1, detect_obstacle);
	pub = nh.advertise<race::drive_values>("Control", 1000);
	ros::spin();
}
