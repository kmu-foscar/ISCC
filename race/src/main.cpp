    #include <ros/ros.h>
    #include <race/sign_classes.h>
    #include <std_msgs/Bool.h>
    #include <std_msgs/Int16.h>
    #include "config.h"

    ros::Publisher pub, pub2, pub3, pub4, pub5, pub6, pub7, pub8;
    ros::Subscriber sub, sub2;

    std_msgs::Bool lk_onoff_msg = std_msgs::Bool(); // Lane Keeper
    std_msgs::Bool sc_onoff_msg = std_msgs::Bool(); // Sign Classifier
    std_msgs::Bool oa_onoff_msg = std_msgs::Bool(); // Obstacle Avoider
    std_msgs::Bool cw_onoff_msg = std_msgs::Bool(); // Crosswalk
    std_msgs::Bool pk_onoff_msg = std_msgs::Bool(); // Parking
    std_msgs::Bool ut_onoff_msg = std_msgs::Bool(); // U-Turn
    std_msgs::Bool so_onoff_msg = std_msgs::Bool(); // Static Obstacle
    std_msgs::Bool do_onoff_msg = std_msgs::Bool(); // Dynamic Obstacle
    
    int return_sig;
    //
    void publish_msgs() {
        pub.publish(lk_onoff_msg);
        pub2.publish(sc_onoff_msg);
        pub3.publish(oa_onoff_msg);
        pub4.publish(cw_onoff_msg);
        pub5.publish(pk_onoff_msg);
        pub6.publish(ut_onoff_msg);
        pub7.publish(so_onoff_msg);
        pub8.publish(do_onoff_msg);
    }
    void signCallback(const std_msgs::Int16 &msg) {
        unsigned char sign_class = msg.data;
        printf("%d\n", sign_class);
        switch (sign_class) {
        case MODE_CROSSWALK :
            lk_onoff_msg.data = false;
            sc_onoff_msg.data = false;
            oa_onoff_msg.data = false;
            cw_onoff_msg.data = true;
            pk_onoff_msg.data = false;
            ut_onoff_msg.data = false;
            so_onoff_msg.data = false;
            do_onoff_msg.data = false;
            break;
        case MODE_STATIC_OBSTACLE :
            lk_onoff_msg.data = false;
            sc_onoff_msg.data = false;
            oa_onoff_msg.data = false;
            cw_onoff_msg.data = false;
            pk_onoff_msg.data = false;
            ut_onoff_msg.data = false;
            so_onoff_msg.data = true;
            do_onoff_msg.data = false;
            break;
        case MODE_DYNAMIC_OBSTACLE :
            lk_onoff_msg.data = false;
            sc_onoff_msg.data = false;
            oa_onoff_msg.data = false;
            cw_onoff_msg.data = false;
            pk_onoff_msg.data = false;
            ut_onoff_msg.data = false;
            so_onoff_msg.data = false;
            do_onoff_msg.data = true;
            break;
        case MODE_NARROW :
            lk_onoff_msg.data = false;
            sc_onoff_msg.data = false;
            oa_onoff_msg.data = true;
            cw_onoff_msg.data = false;
            pk_onoff_msg.data = false;
            ut_onoff_msg.data = false;
            so_onoff_msg.data = false;
            do_onoff_msg.data = false;
            break;
        case MODE_CURVE :
            lk_onoff_msg.data = false;
            sc_onoff_msg.data = false;
            oa_onoff_msg.data = true;
            cw_onoff_msg.data = false;
            pk_onoff_msg.data = false;
            ut_onoff_msg.data = false;
            so_onoff_msg.data = false;
            do_onoff_msg.data = false;
            break;
        case MODE_UTURN :
            lk_onoff_msg.data = false;
            sc_onoff_msg.data = false;
            oa_onoff_msg.data = false;
            cw_onoff_msg.data = false;
            pk_onoff_msg.data = false;
            ut_onoff_msg.data = true;
            so_onoff_msg.data = false;
            do_onoff_msg.data = false;
            break;
        case MODE_PARKING :
            lk_onoff_msg.data = false;
            sc_onoff_msg.data = false;
            oa_onoff_msg.data = false;
            cw_onoff_msg.data = false;
            pk_onoff_msg.data = true;
            ut_onoff_msg.data = false;
            so_onoff_msg.data = false;
            do_onoff_msg.data = false;
            break;
        }
        publish_msgs();
    }
    void returnCallback(const std_msgs::Int16 &msg) {
        return_sig = msg.data;
        if(return_sig > 0) {
            lk_onoff_msg.data = true;
            sc_onoff_msg.data = true;
            oa_onoff_msg.data = false;
            cw_onoff_msg.data = false;
            pk_onoff_msg.data = false;
            ut_onoff_msg.data = false;
            so_onoff_msg.data = false;
            do_onoff_msg.data = false;
        }
        publish_msgs();
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
        pub7 = nh.advertise<std_msgs::Bool> ("so_onoff", 1);
        pub8 = nh.advertise<std_msgs::Bool> ("do_onoff", 1);

        ros::spin();
    }
