    #include <ros/ros.h>
    #include <race/sign_classes.h>
    #include <std_msgs/Bool.h>
    #include <std_msgs/Int16.h>
    #include "config.h"

    int miyak[8] = {MODE_CROSSWALK, MODE_NARROW, MODE_STATIC_OBSTACLE, MODE_DYNAMIC_OBSTACLE, MODE_CURVE, MODE_UTURN, MODE_PARKING, MODE_BASE};
    int sequence = 0;
    ros::Publisher pub, pub2, pub3, pub4, pub5, pub6, pub7, pub8;
    ros::Subscriber sub, sub_seq;

    std_msgs::Bool lk_onoff_msg = std_msgs::Bool(); // Lane Keeper
    std_msgs::Bool oa_onoff_msg = std_msgs::Bool(); // Obstacle Avoider
    std_msgs::Bool cw_onoff_msg = std_msgs::Bool(); // Crosswalk
    std_msgs::Bool pk_onoff_msg = std_msgs::Bool(); // Parking
    std_msgs::Bool ut_onoff_msg = std_msgs::Bool(); // U-Turn
    std_msgs::Bool so_onoff_msg = std_msgs::Bool(); // Static Obstacle
    std_msgs::Bool do_onoff_msg = std_msgs::Bool(); // Dynamic Obstacle
    
    int return_sig;

    void publish_msgs() {
        pub.publish(lk_onoff_msg);
        pub3.publish(oa_onoff_msg);
        pub4.publish(cw_onoff_msg);
        pub5.publish(pk_onoff_msg);
        pub6.publish(ut_onoff_msg);
        pub7.publish(so_onoff_msg);
        pub8.publish(do_onoff_msg);
    }
    
    void returnCallback(const std_msgs::Int16 &msg) {
        return_sig = msg.data;
        if(return_sig == RETURN_FINISH) {
            sequence++;
            printf("sequence : %d\n", sequence);
            switch (miyak[sequence]) {
                case MODE_NARROW :
                printf("Branch Road\n");
                lk_onoff_msg.data = false;
                oa_onoff_msg.data = true;
                cw_onoff_msg.data = false;
                pk_onoff_msg.data = false;
                ut_onoff_msg.data = false;
                so_onoff_msg.data = false;
                do_onoff_msg.data = false;  
                break;  
                case MODE_DYNAMIC_OBSTACLE :
                printf("Dynamic Obstacle\n");
                lk_onoff_msg.data = false;
                oa_onoff_msg.data = false;
                cw_onoff_msg.data = false;
                pk_onoff_msg.data = false;
                ut_onoff_msg.data = false;
                so_onoff_msg.data = false;
                do_onoff_msg.data = true;    
                break;
                case MODE_STATIC_OBSTACLE :
                printf("Static Obstacle\n");                
                lk_onoff_msg.data = false;
                oa_onoff_msg.data = false;
                cw_onoff_msg.data = false;
                pk_onoff_msg.data = false;
                ut_onoff_msg.data = false;
                so_onoff_msg.data = true;
                do_onoff_msg.data = false;    
                break;
                case MODE_CURVE :
                printf("S-Curve\n");
                lk_onoff_msg.data = false;
                oa_onoff_msg.data = true;
                cw_onoff_msg.data = false;
                pk_onoff_msg.data = false;
                ut_onoff_msg.data = false;
                so_onoff_msg.data = false;
                do_onoff_msg.data = false;    
                break;
                case MODE_UTURN :
                printf("U-Turn\n");
                lk_onoff_msg.data = false;
                oa_onoff_msg.data = false;
                cw_onoff_msg.data = false;
                pk_onoff_msg.data = false;
                ut_onoff_msg.data = true;
                so_onoff_msg.data = false;
                do_onoff_msg.data = false;    
                break;
                case MODE_PARKING :
                printf("Parking\n");
                lk_onoff_msg.data = false;
                oa_onoff_msg.data = false;
                cw_onoff_msg.data = false;
                pk_onoff_msg.data = true;
                ut_onoff_msg.data = false;
                so_onoff_msg.data = false;
                do_onoff_msg.data = false;    
                break;
                case MODE_BASE :
                printf("Base\n");
                lk_onoff_msg.data = true;
                oa_onoff_msg.data = false;
                cw_onoff_msg.data = false;
                pk_onoff_msg.data = false;
                ut_onoff_msg.data = false;
                so_onoff_msg.data = false;
                do_onoff_msg.data = false;    
                break;
            }
        }
        else if(return_sig == RETURN_STOP) {
            printf("Lidar Stop\n");
            lk_onoff_msg.data = true;
            oa_onoff_msg.data = true;
            cw_onoff_msg.data = false;
            pk_onoff_msg.data = false;
            ut_onoff_msg.data = false;
            so_onoff_msg.data = false;
            do_onoff_msg.data = false;   
        }
        else if(return_sig == RETURN_OPERATE) {
            printf("Lidar Start\n");
            lk_onoff_msg.data = false;
            oa_onoff_msg.data = true;
            cw_onoff_msg.data = false;
            pk_onoff_msg.data = false;
            ut_onoff_msg.data = false;
            so_onoff_msg.data = false;
            do_onoff_msg.data = false;   
        }
        publish_msgs();
    }
    void sequenceCallback(const std_msgs::Int16 &msg) {
        sequence = msg.data;
        switch (miyak[sequence]) {
            case MODE_NARROW :
            lk_onoff_msg.data = false;
            oa_onoff_msg.data = true;
            cw_onoff_msg.data = false;
            pk_onoff_msg.data = false;
            ut_onoff_msg.data = false;
            so_onoff_msg.data = false;
            do_onoff_msg.data = false;    
            case MODE_DYNAMIC_OBSTACLE :
            lk_onoff_msg.data = false;
            oa_onoff_msg.data = false;
            cw_onoff_msg.data = false;
            pk_onoff_msg.data = false;
            ut_onoff_msg.data = false;
            so_onoff_msg.data = false;
            do_onoff_msg.data = true;    
            break;
            case MODE_STATIC_OBSTACLE :
            lk_onoff_msg.data = false;
            oa_onoff_msg.data = false;
            cw_onoff_msg.data = false;
            pk_onoff_msg.data = false;
            ut_onoff_msg.data = false;
            so_onoff_msg.data = true;
            do_onoff_msg.data = false;    
            break;
            case MODE_CURVE :
            lk_onoff_msg.data = false;
            oa_onoff_msg.data = true;
            cw_onoff_msg.data = false;
            pk_onoff_msg.data = false;
            ut_onoff_msg.data = false;
            so_onoff_msg.data = false;
            do_onoff_msg.data = false;    
            break;
            case MODE_UTURN :
            lk_onoff_msg.data = false;
            oa_onoff_msg.data = false;
            cw_onoff_msg.data = false;
            pk_onoff_msg.data = false;
            ut_onoff_msg.data = true;
            so_onoff_msg.data = false;
            do_onoff_msg.data = false;    
            break;
            case MODE_PARKING :
            lk_onoff_msg.data = false;
            oa_onoff_msg.data = false;
            cw_onoff_msg.data = false;
            pk_onoff_msg.data = true;
            ut_onoff_msg.data = false;
            so_onoff_msg.data = false;
            do_onoff_msg.data = false;    
            break;
            case MODE_BASE :
            lk_onoff_msg.data = true;
            oa_onoff_msg.data = false;
            cw_onoff_msg.data = false;
            pk_onoff_msg.data = false;
            ut_onoff_msg.data = false;
            so_onoff_msg.data = false;
            do_onoff_msg.data = false;    
            break;
        }
        publish_msgs();
    }
    int main(int argc, char** argv)
    {
        ros::init(argc, argv, "Central_Controller");
        ros::NodeHandle nh;
        sub = nh.subscribe("return_signal", 1, returnCallback);
        sub_seq = nh.subscribe("sequence", 1, sequenceCallback);
        pub = nh.advertise<std_msgs::Bool> ("lk_onoff", 1);
        pub3 = nh.advertise<std_msgs::Bool> ("oa_onoff", 1);
        pub4 = nh.advertise<std_msgs::Bool> ("cw_onoff", 1);
        pub5 = nh.advertise<std_msgs::Bool> ("pk_onoff", 1);
        pub6 = nh.advertise<std_msgs::Bool> ("ut_onoff", 1);
        pub7 = nh.advertise<std_msgs::Bool> ("so_onoff", 1);
        pub8 = nh.advertise<std_msgs::Bool> ("do_onoff", 1);
        if(sequence == 0) {
            printf("Crosswalk\n");
            lk_onoff_msg.data = false;
            oa_onoff_msg.data = false;
            cw_onoff_msg.data = true;
            pk_onoff_msg.data = false;
            ut_onoff_msg.data = false;
            so_onoff_msg.data = false;
            do_onoff_msg.data = false;
            publish_msgs();   
            pub.publish(lk_onoff_msg);
            pub4.publish(cw_onoff_msg);
        }
        ros::spin();
    }
