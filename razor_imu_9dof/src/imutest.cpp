#include <ros/ros.h>
#include <stdlib.h>
#include <signal.h>
#include <sensor_msgs/Imu.h>
#include <math.h>
#include <queue>
#include <time.h>

#define PI 3.14159265

using namespace std;

ros::Subscriber sub;

clock_t prev_time, cur_time;

//float gyro[3];
// queue<float> accel_x;
// queue<float> accel_y;
// queue<float> accel_z;
// queue<float> magnetic_x;
// queue<float> magnetic_y;

float cur_position_x = 0.f;
float cur_position_y = 0.f;

float prev_velocity_x = 0.f;
float prev_velocity_y = 0.f;
float cur_velocity_x = 0.f;
float cur_velocity_y = 0.f;

float prev_accel_x = 0.f;
float prev_accel_y = 0.f;

float accel[3];
float magnetic[3];

float theta = 0.;

void testerCallback(const sensor_msgs::Imu &msg)
{

  prev_accel_x = accel[0];
  prev_accel_y = accel[1];

  accel[0] = msg.linear_acceleration.x * 9.8;
  accel[1] = msg.linear_acceleration.y * 9.8;
  accel[2] = msg.linear_acceleration.z * 9.8;

  magnetic[0] = msg.angular_velocity.x;
  magnetic[1] = msg.angular_velocity.y;
  magnetic[2] = msg.angular_velocity.z;

  if(accel[0] > 0.02)
  {
    prev_velocity_x = cur_velocity_x;
    cur_velocity_x += (accel[0] - prev_accel_x) / 0.1;
  }

  if(accel[1] > 0.02)
  {
    prev_velocity_y = cur_velocity_y;
    cur_velocity_y += (accel[1] - prev_accel_y) / 0.1;
  }

  cur_position_x = (cur_velocity_x - prev_velocity_x) / 0.1;
  cur_position_y = (cur_velocity_y - prev_velocity_y) / 0.1;
}

void calcAzimuth(float v[3])
{
  theta = atan2(v[1] , v[0]) * 180 / PI;
}

void SaveIMUData(const char* filename)
{
  FILE *fp = fopen(filename, "at");
  if(fp)
  {
    // fprintf(fp, "%10.6f %10.6f %10.6f\n", );
  }
  fclose(fp);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;

  sub = nh.subscribe("imu", 1000, testerCallback);

  while(ros::ok())
  {
    //SaveIMUData("imu_test.txt");
    calcAzimuth(magnetic);
    printf("%f , %f\n", cur_position_x, cur_position_y);
    printf("%f\n", sqrt((cur_position_x*cur_position_x) + (cur_position_y * cur_position_y)));
    //printf("%f", theta);
    printf("\n");

    ros::spinOnce();
  }

  return 0;
}
