#include <ros/ros.h>
#include <obstacle_detector/Obstacles.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <algorithm>
#include <stdlib.h>
#include <math.h>
#include <std_msgs/Int32.h>

using namespace std;

ros::Publisher pub;

double distance(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
	double dist = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
	return sqrt(dist);
}


