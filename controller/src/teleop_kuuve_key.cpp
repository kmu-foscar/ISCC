#include <iostream>
#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

using namespace std;

int main(int argc, char** argv){
	ros::init(argc, argv, "teleop_kuuve_key");
	ros::NodeHandle nh;

	ros::Publisher ackermann_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann", 10);
	ros::Rate loop_rate(5);

	ackermann_msgs::AckermannDriveStamped ackermann_data;
	ackermann_data.drive.steering_angle = 0;
	ackermann_data.drive.speed = 0;
	while(ros::ok()){
		ackermann_data.drive.steering_angle += 1;
//		ackermann_data.drive.speed += 0.5;
		ackermann_data.drive.speed = 5;
		if(ackermann_data.drive.steering_angle >= 28) ackermann_data.drive.steering_angle = -28;
//		if(ackermann_data.drive.speed >= 15) ackermann_data.drive.speed = 0;
		cout << "send:" << ackermann_data.drive.steering_angle << endl;
		ackermann_pub.publish(ackermann_data);
		ros::spinOnce();
		loop_rate.sleep();	
	}
	return 0;
}
/*
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <iostream>
 */
/*
   int getch()
   {
   static struct termios oldt, newt;
   tcgetattr( STDIN_FILENO, &oldt);           // save old settings
   newt = oldt;
   newt.c_lflag &= ~(ICANON);                 // disable buffering      
   tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

   int c = getchar();  // read character (non-blocking)

   tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
   return c;
   }

   int main(int argc, char** argv){
   ros::init(argc, argv, "teleop_kuuve_key");
   ackermann_msgs::AckermannDriveStamped ackermann;

   while (ros::ok())
   {
   int c = getch();   // call your non-blocking input function
   if (c == 'w'){}
   else if (c == 's'){}
   if (c == 'a'){}
   else if(c == 'd'){}
   }

   }
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <iostream>

class KuuveTeleop
{
public:
KuuveTeleop();

private:
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
void publish();

ros::NodeHandle ph_, nh_;

int linear_, angular_, deadman_axis_;
double l_scale_, a_scale_;
ros::Publisher vel_pub_;
ros::Subscriber joy_sub_;

geometry_msgs::Twist last_published_;
boost::mutex publish_mutex_;
bool deadman_pressed_;
bool zero_twist_published_;
ros::Timer timer_;

};

TurtlebotTeleop::TurtlebotTeleop():
ph_("~"),
linear_(1),
angular_(0),
deadman_axis_(4),
l_scale_(0.3),
a_scale_(0.9)
{
ph_.param("axis_linear", linear_, linear_);
ph_.param("axis_angular", angular_, angular_);
ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
ph_.param("scale_angular", a_scale_, a_scale_);
ph_.param("scale_linear", l_scale_, l_scale_);

deadman_pressed_ = false;
zero_twist_published_ = false;

vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TurtlebotTeleop::joyCallback, this);

timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&TurtlebotTeleop::publish, this));
}

void TurtlebotTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
	geometry_msgs::Twist vel;
	vel.angular.z = a_scale_*joy->axes[angular_];
	vel.linear.x = l_scale_*joy->axes[linear_];
	last_published_ = vel;
	deadman_pressed_ = joy->buttons[deadman_axis_];
}

void TurtlebotTeleop::publish()
{
	boost::mutex::scoped_lock lock(publish_mutex_);

	if (deadman_pressed_)
	{
		vel_pub_.publish(last_published_);
		zero_twist_published_=false;
	}
	else if(!deadman_pressed_ && !zero_twist_published_)
	{
		vel_pub_.publish(*new geometry_msgs::Twist());
		zero_twist_published_=true;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "turtlebot_teleop");
	TurtlebotTeleop turtlebot_teleop;

	ros::spin();
}
*/
