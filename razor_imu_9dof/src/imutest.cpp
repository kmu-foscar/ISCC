#include <ros/ros.h>
#include <stdlib.h>

ros::Publisher pub;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_node");

  printf("imu msg test!!\n");

  return 0;
}
