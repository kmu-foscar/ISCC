#include <ros/ros.h>

#define MODE_BASE 0 // Lane Keeping Mode
#define MODE_CROSSWALK 1
#define MODE_STATIC_OBSTACLE 2
#define MODE_DYNAMIC_OBSTACLE 3
#define MODE_NARROW 4
#define MODE_CURVE 5
#define MODE_UTURN 6
#define MODE_PARKING 7


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "Central_Controller");
    ros::NodeHandle nh;
}