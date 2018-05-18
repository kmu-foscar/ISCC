#include <ros/ros.h>

double begin, end;

int main(int argc, char** argv) {
    ros::init(argc, argv, "Time_test");
    ros::NodeHandle nh;
    printf("start\n");
    begin = ros::Time::now().toSec();
    while(ros::ok()) {
        end = ros::Time::now().toSec();
        if (end - begin >= 3.f) {
            printf("end\n");
            break;
        }
        ros::spinOnce();
    }
    return 0;
}
