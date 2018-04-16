#include <ros/ros.h>
#include <stdlib.h>
#include <signal.h>
#include <sensor_msgs/Imu.h>
#include <math.h>

ros::Subscriber sub;

float gyro[3];
float accel[3];
float _HDR_I[3] = {0.,};
bool _w_lp_init = false;
bool _w_dl_init = false;
float _w_lp[3];
float _w_dl[3];
float _T = 0.005;
float _tau = 0.01;

void testerCallback(const sensor_msgs::Imu &msg)
{
  gyro[0] = msg.angular_velocity.x;
  gyro[1] = msg.angular_velocity.y;
  gyro[2] = msg.angular_velocity.z;

  accel[0] = msg.linear_acceleration.x;
  accel[1] = msg.linear_acceleration.y;
  accel[2] = msg.linear_acceleration.z;
}

 void LowpPassfilter(float v[3])
 {
   if(!_w_lp_init){
     _w_lp[0] = v[0];
     _w_lp[1] = v[1];
     _w_lp[2] = v[2];

     _w_lp_init = true;
   }

   _w_lp[0] = (_T*v[0] + _tau*_w_lp[0])/(_T + _tau);
   _w_lp[1] = (_T*v[1] + _tau*_w_lp[1])/(_T + _tau);
   _w_lp[2] = (_T*v[2] + _tau*_w_lp[2])/(_T + _tau);

   v[0] = _w_lp[0];
   v[1] = _w_lp[1];
   v[2] = _w_lp[2];
 }

int SIGN(float v)
{
  if(v > 0.) return 1;
  if(v < 0.) return -1;
  return 0;
}

void HDR(float v[3], double threshold, double ic)
{
  v[0] += _HDR_I[0];
  v[1] += _HDR_I[1];
  v[2] += _HDR_I[2];

  if((-threshold < v[0] && v[0] < threshold) &&
  (-threshold < v[1] && v[1] < threshold) &&
  (-threshold < v[2] && v[2] < threshold)){
    _HDR_I[0] -= SIGN(v[0]) * ic;
    _HDR_I[1] -= SIGN(v[1]) * ic;
    _HDR_I[2] -= SIGN(v[2]) * ic;
  }
}

void Delagging(float v[3])
{
  if(!_w_dl_init)
  {
    _w_dl[0] = v[0];
    _w_dl[1] = v[1];
    _w_dl[2] = v[2];

    _w_dl_init = true;
  }

  float wd[3];

  wd[0] = v[0] + _tau/_T*(v[0] - _w_dl[0]);
  wd[1] = v[1] + _tau/_T*(v[1] - _w_dl[1]);
  wd[2] = v[2] + _tau/_T*(v[2] - _w_dl[2]);

  _w_dl[0] = v[0];
  _w_dl[1] = v[1];
  _w_dl[2] = v[2];

  v[0] = wd[0];
  v[1] = wd[1];
  v[2] = wd[2];
}

void SaveIMUData(const char* filename)
{
  FILE *fp = fopen(filename, "at");
  if(fp)
  {
    fprintf(fp, "%10.6f %10.6f %10.6f\n", gyro[0], gyro[1], gyro[2]);
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
    LowpPassfilter(gyro);
    HDR(gyro, 3, 0.001);
    Delagging(gyro);
    //SaveIMUData("imu_test.txt");

    printf("%f , %f , %f :: %f , %f , %f\n", gyro[0], gyro[1], gyro[2], accel[0] , accel[1] , accel[2]);
    printf("\n");

    ros::spinOnce();
  }

  return 0;
}
