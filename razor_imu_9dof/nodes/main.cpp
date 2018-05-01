#include <ros/ros.h>
#include <stdio.h>

ros::Publisher pub;
ros::Subscriber sub;

//void print(const ) {
//	printf("%d\n", msg.linear_acceleration.x);
//}

int main(int argc, char** argv) {
	printf("Hello world!\n");

	ros::init(argc, argv, "main_node");
	ros::NodeHandle nh;
//	sub = nh.subscribe("imu_node.py", 1000, print);
	ros::spin();
}
