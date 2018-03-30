#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8MultiArray.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <string>
#include <stdlib.h>
#include <cstdlib>
#include <cstring>
#include <sstream>


serial::Serial ser;
using namespace std;

unsigned char alive=0x00;
unsigned char gear = 0x00;
unsigned char speed_0 = 0x00;
unsigned char speed_1 = 0x00;
unsigned char steer_0 = 0x00;
unsigned char steer_1 = 0x00;
unsigned char front_break = 0x01;

int main (int argc, char** argv){
	//std_msgs::UInt8MultiArray msg;
	//msg.data.resize(14);

	//set variable
	string serial_input;

	unsigned int steer_total = 0;
	unsigned char str[14] = {0x53,0x54,0x58,0x01,0x00,0x00,speed_0, speed_1 ,steer_0,steer_1,front_break,alive,0x0D,0x0A};

	ros::init(argc, argv, "serial_example_node");
	ros::NodeHandle nh;

	//ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
//	ros::Publisher read_pub = nh.advertise<std_msgs::String>("serial_data", 1000);
	//ros::Subscriber sub = nh.subscribe("ackermann", 10, ackermannCallback);

	try{
		ser.setPort("/dev/ttyUSB2");
		ser.setBaudrate(115200);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(to);
		ser.open();
	}
	catch (serial::IOException& e){
		ROS_ERROR_STREAM("Unable to open port ");
		return -1;
	}

	if(ser.isOpen()) ROS_INFO_STREAM("Serial Port initialized");
	else return -1;

	//ros::Rate loop_rate(5);

	//std::string str;

	while(ros::ok()){
		size_t num_write = 14;

		//size_t num_rev = 18;
		//uint8_t receive[18];	
		//write packet

		if(ser.available()){
			std_msgs::String result;
			serial_input = ser.readline(); 

//			cout << "alive:" << (unsigned int)alive << endl;
//			cout << "read:" << serial_input[0] << endl;
			cout << "-------------------START--------------" << endl;
			cout << "gear:" << (int)serial_input[5] << endl;
			cout << "speed0:" << (int)serial_input[6] << endl;
			cout << "speed1:" << (int)serial_input[7] << endl;
			cout << "steer0:" << (int)serial_input[8] << endl;
			cout << "steer1:" << (int)serial_input[9] << endl;
			cout << "enc0:" << (int)serial_input[11] << endl;
			cout << "enc1:" << (int)serial_input[12] << endl;
			cout << "enc2:" << (int)serial_input[13] << endl;
			cout << "enc3:" << (int)serial_input[14] << endl;
			cout << "alive:" << (int)serial_input[15] << endl;
//			read_pub.publish(result);
		}
	 }
 }
