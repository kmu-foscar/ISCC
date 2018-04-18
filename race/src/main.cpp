#include <ros/ros.h>
#include <race/sign_classes.h>
#include <std_msgs/Bool.h>

#define MODE_BASE 0 // Lane Keeping Mode
#define MODE_CROSSWALK 1
#define MODE_STATIC_OBSTACLE 2
#define MODE_DYNAMIC_OBSTACLE 3
#define MODE_NARROW 4
#define MODE_CURVE 5
#define MODE_UTURN 6
#define MODE_PARKING 7

ros::Publisher pub;
ros::Subscriber sub; 

void signCallback(const race::sign_classes &msg) {
    unsigned char sign_class = msg.sign_class;
    std_msgs::Bool lk_onoff_msg = std_msgs::Bool();
    printf("%d\n", sign_class);
    switch (sign_class) {
    case 0 :
        lk_onoff_msg.data = true;
        pub.publish(lk_onoff_msg);
        break;
    case 1 :
        lk_onoff_msg.data = false;
        pub.publish(lk_onoff_msg);
        break;
    }
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "Central_Controller");
    ros::NodeHandle nh;
    sub = nh.subscribe("sign_classes", 1000, signCallback);
    pub = nh.advertise<std_msgs::Bool> ("lk_onoff", 1);
    ros::spin();
}