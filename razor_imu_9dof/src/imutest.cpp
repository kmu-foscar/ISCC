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

float cur_lat = 37.24474403;
float cur_lon = 126.7682516;

float delta_position_x = 0.f;
float delta_position_y = 0.f;

float prev_velocity_x = 0.f;
float prev_velocity_y = 0.f;
float cur_velocity_x = 0.f;
float cur_velocity_y = 0.f;

float prev_accel_x = 0.f;
float prev_accel_y = 0.f;

float moving_dist = 0.f;

float accel[3];
float magnetic[3];

float theta = 0.;

int countX = 0;
int countY = 0;
int time_stamp = -1;

void SaveIMUData(const char* filename)
{
  FILE *fp = fopen(filename, "at");
  if(fp)
  {
    fprintf(fp, "%d : %10.6f %10.6f\n", time_stamp, cur_lat, cur_lon);
  }
  fclose(fp);
}

void calcGPS()
{
  cur_lat += (moving_dist * sin(theta)) * (0.00001 / 1.1);
  cur_lon += (moving_dist * cos(theta)) * (0.00001 / 0.9);
}

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

  if(countX >= 10)
  {
    prev_accel_x = accel[0] = 0;
    cur_velocity_x = 0;
    cur_velocity_y = 0;
  }
  if(countY >= 10)
  {
    prev_accel_y = accel[1] = 0;
    cur_velocity_x = 0;
    cur_velocity_y = 0;
  }

  if(accel[0] > 0.15 || accel[0] < -0.15)
  {
    prev_velocity_x = cur_velocity_x;
    cur_velocity_x += (accel[0] - prev_accel_x) / 0.1;
    countX = 0;
  } else countX++;

  if(accel[1] > 0.15 || accel[1] < -0.15)
  {
    prev_velocity_y = cur_velocity_y;
    cur_velocity_y += (accel[1] - prev_accel_y) / 0.1;
    countY = 0;
  } else countY++;

  delta_position_x = ((cur_velocity_x + prev_velocity_y) / 2) * 0.1;
  delta_position_y = ((cur_velocity_y + prev_velocity_y) / 2) * 0.1;

  moving_dist = sqrt(delta_position_x * delta_position_x + delta_position_y * delta_position_y);

  calcGPS();
  SaveIMUData("GPSdata.txt");
}

void calcAzimuth(float v[3])
{
  theta = atan2(v[1] , v[0]) * 180 / PI;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;

  sub = nh.subscribe("imu", 1000, testerCallback);

  while(ros::ok())
  {
    calcAzimuth(magnetic);
    printf("%d : %10.6f , %10.6f\n", ++time_stamp, cur_lat, cur_lon);

    ros::spinOnce();
  }

  return 0;
}
