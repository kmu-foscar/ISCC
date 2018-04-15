#include <ros/ros.h>
#include <stdlib.h>
#include <signal.h>
#include <sensor_msgs/Imu.h>

ros::Subscriber sub;

float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

float accel_x = 0.0f;
float accel_y = 0.0f;
float accel_z = 0.0f;

void testerCallback(const sensor_msgs::Imu &msg);

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;

  sub = nh.subscribe("imu", 1000, testerCallback);

  while(ros::ok())
  {
    printf("%f , %f , %f :: %f , %f , %f\n", roll, pitch, yaw, accel_x , accel_y , accel_z);
    ros::spinOnce();
  }

  return 0;
}

void testerCallback(const sensor_msgs::Imu &msg)
{
  roll = msg.angular_velocity.x;
  pitch = msg.angular_velocity.y;
  yaw = msg.angular_velocity.z;

  accel_x = msg.linear_acceleration.x;
  accel_y = msg.linear_acceleration.y;
  accel_z = msg.linear_acceleration.z;
}
