#include <iostream>
#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Time.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;
int laser_flag = 0;
/*void timeCallback(const std_msgs::Time::ConstPtr& msg){
	ROS_INFO("%d", msg->data);
}

void ackermannCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg){
	ROS_INFO("%f, %f", msg->drive.speed, msg->drive.steering_angle);
}
*/
void control_sec(ros::Duration _control_time, float _steering_angle, float _speed){
	ackermann_msgs::AckermannDriveStamped ackermann_data;
	ackermann_data.drive.steering_angle = _steering_angle;
	ackermann_data.drive.speed = _speed;

	ros::NodeHandle nh;
	ros::Publisher ackermann_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann", 10);
	ros::Time start_time = ros::Time::now();
	ros::Duration time_control(_control_time);
	ros::Rate r(5);
	while(ros::Time::now() - start_time < time_control){
		ackermann_pub.publish(ackermann_data);
		ros::spinOnce();
		cout << "steer: " << _steering_angle << " speed: " << _speed << endl;
		r.sleep();
	}
	ackermann_data.drive.steering_angle = 0.0f;
	ackermann_data.drive.speed = 0.0f;
	ackermann_pub.publish(ackermann_data);
	ros::spinOnce();
}
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	int count = 0;
	laser_flag = 0;
	ROS_INFO("TEST");

	for(int i = 180; i < 200; i++){
		if (msg->ranges[i] < 1)
			count++;
	}
	if(count > 15)
		laser_flag = 1;
}

void parking(){
	ros::NodeHandle nh;
	ros::Publisher ackermann_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann", 10);
	ackermann_msgs::AckermannDriveStamped ackermann_data;
	while(!laser_flag){
		ROS_INFO("TEST2");
		ackermann_data.drive.steering_angle = 0.0f;
		ackermann_data.drive.speed = 2.0f;
		ackermann_pub.publish(ackermann_data);
	}
	control_sec(ros::Duration(0.0), 0.0, 0.0);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "control_sec");
	ros::NodeHandle nh;
	ros::Subscriber scanSub;

	scanSub=nh.subscribe<sensor_msgs::LaserScan>("/scan",10,laserCallback);
	parking();
/*
	control_sec(ros::Duration(5.0), 10.0, -2.0);
	control_sec(ros::Duration(5.0), -10.0, 0.0);
	control_sec(ros::Duration(5.0), 10.0, 2.0);
	control_sec(ros::Duration(1.0), 0.0, 0.0);
*/
	return 0;
}
