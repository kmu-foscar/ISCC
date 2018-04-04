#include <ros/ros.h>
#include <obstacle_detector/Obstacles.h>
#include <geometry_msgs/Point.h>
#include <algorithm>
#include <stdlib.h>
#include <math.h>
#include <std_msgs/Int32.h>
#include <race/drive_values.h>

using namespace std;

/* 제어 상수 */
#define K_FD 5 // Foward distance 상수
#define K_DG 0.7  // Difference of Gradient 상수
#define K_V 0.5 // velocity 상수


#define DETECT_RANGE 0.55 

#define MAX_DISTANCE 5	// FOWARD DISTANCE의 최대 탐지 거리

ros::Publisher pub;

double distance_from_car(geometry_msgs::Point p1, geometry_msgs::Point p2) {
	double distance = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
	return distance;
}

bool isExist(const obstacle_detector::Obstacles &data, geometry_msgs::Point &p) {
	
	bool flag = false;
	p.x = 50, p.y = 0;
	for(int i = 0; i < data.circles.size(); ++i) {
		if((data.circles[i].center.x < 0.02) &&
			(data.circles[i].center.y < DETECT_RANGE) &&
			(data.circles[i].center.y > (-1) * DETECT_RANGE))
				if(!flag || p.x > data.circles[i].center.x) {
					flag = true;
					p = data.circles[i].center;
				}
	}
	if(flag == true) return true;
	return false;
}

void calculator(const obstacle_detector::Obstacles data) {

	geometry_msgs::Point c, front;
	c.x = 0, c.y = 0;

	double distanceR = distance_from_car(data.circles[0].center, 	c);
	double distanceL = distance_from_car(data.circles[data.circles.size()-1].center, c);	
	double distanceF = sqrt(pow(front.x - c.x, 2) + pow(front.y - c.y, 2));
	double error_distance = (distanceR - distanceL);

	double steer_pid_value = K_DG * error_distance;
	double speed_pid_value;

	if(!isExist(data, front)) { //go straight
		steer_pid_value = 0;
		speed_pid_value = K_V * distanceF + 3;
	}

	else if(distanceR > distanceL) { //turn right
		if(steer_pid_value > 100.0) steer_pid_value = 100.0;
		speed_pid_value = K_V * distanceF + 1;
	}

	else if(distanceR < distanceL) { //turn left
		if(steer_pid_value < -100.0) steer_pid_value = -100.0;
		speed_pid_value = K_V * distanceF + 1;
	}
	race::drive_values msg;
	msg.steering = steer_pid_value + 100;
	msg.throttle = speed_pid_value;
	printf("steering : %f speed : %f\n", steer_pid_value, speed_pid_value);
	pub.publish(msg);
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "PID_Contoller_node");
	ros::NodeHandle nh;
	
	ros::Subscriber sub = nh.subscribe("raw_obstacles", 1, calculator);
	pub = nh.advertise<race::drive_values> ("Controller", 100);
	ros::spin();

}

