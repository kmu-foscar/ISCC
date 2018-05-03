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

void detect_obstacle(const obstacle_detector::Obstacles data) {
  
  bool exist_obstacle;
  int size = data.circles.size();
  string message;

  size>0?exist_obstacle = true:exist_obstacle = false;
 
  race::drive_values msg;
  
  if(exist_obstacle) {
    message="detect obstacle -> stop!!";
    msg.throttle = 0;
  }
  else {
    message="no obstacle -> go!!";
    msg.throttle = 1;
  }	
  printf("%s\n", message);
  pub.publish(msg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "Obstacle_Avoider");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("raw_obstacles", 1, detect_obstacle);
	pub = nh.advertise<race::drive_values>("Control", 1000);
	ros::spin();
}
