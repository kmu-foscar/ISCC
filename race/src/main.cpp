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

ros::Publisher pub, pub2, pub3, pub4, pub5, pub6;
ros::Subscriber sub, sub2; 

std_msgs::Bool lk_onoff_msg = std_msgs::Bool();
std:msgs::Bool sc_onoff_msg = std_msgs::Bool();
std:msgs::Bool oa_onoff_msg = std_msgs::Bool();
std:msgs::Bool cw_onoff_msg = std_msgs::Bool();
std:msgs::Bool pk_onoff_msg = std_msgs::Bool();
std:msgs::Bool ut_onoff_msg = std_msgs::Bool();

void signCallback(const race::sign_classes &msg) {
    unsigned char sign_class = msg.sign_class;
    printf("%d\n", sign_class);
    switch (sign_class) {
    case MODE_BASE :
        lk_onoff_msg.data = true;
        sc_onoff_msg.data = true;
        oa_onoff_msg.data = false;
        cw_onoff_msg.data = false;
        pk_onoff_msg.data = false;
        ut_onoff_msg.data = false;
    case MODE_CROSSWALK :
        lk_onoff_msg.data = false;
        sc_onoff_msg.data = false;
        oa_onoff_msg.data = false;
        cw_onoff_msg.data = true;
        pk_onoff_msg.data = false;
        ut_onoff_msg.data = false;
    case MODE_STATIC_OBSTACLE :
        lk_onoff_msg.data = false;
        sc_onoff_msg.data = false;
        oa_onoff_msg.data = true;
        cw_onoff_msg.data = false;
        pk_onoff_msg.data = false;
        ut_onoff_msg.data = false;        
    case MODE_DYNAMIC_OBSTACLE : 
        lk_onoff_msg.data = false;
        sc_onoff_msg.data = false;
        oa_onoff_msg.data = true;
        cw_onoff_msg.data = false;
        pk_onoff_msg.data = false;
        ut_onoff_msg.data = false;
    case MODE_NARROW :
        lk_onoff_msg.data = false;
        sc_onoff_msg.data = false;
        oa_onoff_msg.data = true;
        cw_onoff_msg.data = false;
        pk_onoff_msg.data = false;
        ut_onoff_msg.data = false;
    case MODE_CURVE :
        lk_onoff_msg.data = false;
        sc_onoff_msg.data = false;
        oa_onoff_msg.data = true;
        cw_onoff_msg.data = false;
        pk_onoff_msg.data = false;
        ut_onoff_msg.data = false;
    case MODE_UTURN :
        lk_onoff_msg.data = false;
        sc_onoff_msg.data = false;
        oa_onoff_msg.data = false;
        cw_onoff_msg.data = false;
        pk_onoff_msg.data = false;
        ut_onoff_msg.data = true;
    case MODE_PARKING :
        lk_onoff_msg.data = false;
        sc_onoff_msg.data = false;
        oa_onoff_msg.data = false;
        cw_onoff_msg.data = false;
        pk_onoff_msg.data = true;
        ut_onoff_msg.data = false;
    default :
        pub.publish(lk_onoff_msg);
        pub2.publish(sc_onoff_msg);
        pub3.publish(oa_onoff_msg);
        pub4.publish(cw_onoff_msg);
        pub5.publish(pk_onoff_msg);
        pub6.publish(ut_onoff_msg);
    }
}
void returnCallback(const race::Bool &msg) {
    bool return_msg = msg.data;
    if(return_msg) {
        sc_onoff_msg = true;
        pub2.publish(sc_onoff_msg);
    }
    // TODO later : implementaton for exception when each module return false because of error
}
int main(int argc, char** argv) 
{
    ros::init(argc, argv, "Central_Controller");
    ros::NodeHandle nh;
    sub = nh.subscribe("sign_classes", 1, signCallback);
    sub2 = nh.subscribe("return_signal", 1, returnCallback);
    pub = nh.advertise<std_msgs::Bool> ("lk_onoff", 1);
    pub2 = nh.advertise<std_msgs::Bool> ("sc_onoff", 1);
    pub3 = nh.advertise<std_msgs::Bool> ("oa_onoff", 1);
    pub4 = nh.advertise<std_msgs::Bool> ("cw_onoff", 1);
    pub5 = nh.advertise<std_msgs::Bool> ("pk_onoff", 1);
    pub6 = nh.advertise<std_msgs::Bool> ("ut_onoff", 1);
    
    ros::spin();
}