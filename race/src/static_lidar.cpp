#include "Lane_Detector.hpp"
#include "Look_Ahead.hpp"
#include <algorithm>
#include <ros/ros.h>
#include <race/drive_values.h>
#include <race/control_variables.h>
#include <signal.h>
#include <cmath>

#define CENTER_POINT 690
#define CENTER_POINT_LA 750
#define MAX_SPEED 12

#define PI 3.1415
#define O_DIST 4
#define O_DIST_POW O_DIST * O_DIST

Lane_Detector* ld;
Look_Ahead* la;
race::drive_values control_msg;
ros::Subscriber sub, sub2;
ros::Publisher control_pub;

float p_steering = -0.3f;
float p_steering_curve = 20.f;
float p_lookahead_curve = 10.f;
float p_lookahead = 0.05f;
int steering;
int throttle;
int test_speed = 5;
int cnt;

bool onoff = true;
void testerCallback(const race::control_variables &msg);
void generate_control_msg(race::drive_values* control_msg);
float cal_lookahead_op_error();

int isLeftObstacle, isRightObstacle;

int main(int argc, char** argv) {
    ros::init(argc, argv, "Lane_Keeper");
    ros::NodeHandle nh;

    ld = new Lane_Detector();
    la = new Look_Ahead();
    sub = nh.subscribe("control_variables", 1000, testerCallback);
    control_pub = nh.advertise<race::drive_values>("Control", 1000);
    ld->init();

    isLeftObstacle = 0;
    isRightObstacle = 0;
    cnt = 0;
    while(ros::ok()) {
        ld->operate();
        la->operate(ld->originImg_left, ld->originImg_right);

        sub2 = nh.subscribe("raw_obstacles", 1, calculator);
        generate_control_msg(&control_msg);

        control_pub.publish(control_msg);
        ros::spinOnce();
    }
    delete ld;
    delete la;
    return 0;
}

void testerCallback(const race::control_variables &msg) {
    p_steering = msg.p_steering;
	p_steering_curve = msg.p_steering_curve;
    test_speed = msg.test_speed;
}

void calculator(obstacle_detector::Obstacles data)
{
    geometry_msgs::Point car;
    car.y = 0.0, car.x = 0.0;

    geometry_msgs::Point closestLeftPoint, closestRightPoint;
    closestLeftPoint.x = MAX, closestRightPoint.y = MAX;
    closestLeftPoint.y = MAX, closestRightPoint.x = MAX;

    double minimumL = MAX, minimumR = MAX;
    const double rightTheta = 0.52; // 30도
    double leftTheta = PI - rightTheta;

    //앞까지의 거리
    for(int i = 0; i < data.circles.size(); i++)
    {
        geometry_msgs::Point curPoint = data.circles[i].center;
        geometry_msgs::Point temp;

        //x축 대칭
        curPoint.y = -curPoint.y;

        //y = x 대칭
        swap(curPoint.x, curPoint.y);

        if(curPoint.y == 0.0 || curPoint.x == 0.0)
            continue;

        double angle = atan2(curPoint.y, curPoint.x);
        double dist = (curPoint.y) * (curPoint.y) + (curPoint.x) * (curPoint.x);

        //1,2사분면이 아니면
        if(angle < 0.0)
            continue;

        //30도이상 오른쪽
        else if(PI / 2 > angle && angle > rightTheta)
        {
            //오른쪽
            if(minimumR > dist) {
                minimumR = dist;
                closestRightPoint = curPoint;
            }
        }
        //120도이하 왼쪽
        else if(PI / 2 < angle && angle < leftTheta)
        {
            //왼쪽
            if(minimumL > dist){
                minimumL = dist;
                closestLeftPoint = curPoint;
            }
        }
    }

    //둘다 잡지 못했거나, 하나가 깜빡이면
    if(minimumL == minimumR)
    {
        if((isLeftObstacle || isRightObstacle) && cnt < 10)
        {
            cnt++;
            return;
        }
        isLeftObstacle = 0;
        isRightObstacle = 0;
        cnt = 0;
    }
    if(minimumL < minimumR) //왼쪽 장애물이 오른쪽 장애물보다 가까운데
    {
        if(isRightObstacle && cnt < 10) // 오른쪽 장애물이 깜박거려서 못본거였으면
        {
            cnt++;
            return; //그대로
        }
        if(isRightObstacle) //실제로 안보이게 된거라면
            isRightObstacle = 0;

        cnt = 0;
        isLeftObstacle = 1;
    }
    if(minimumR < minimumL) //오른쪽 장애물이 왼쪽 장애물보다 가까운데
    {
        if(isLeftObstacle && cnt < 10) // 왼쪽 장애물이 깜박였던거라면
        {
            cnt++;
            return; //지금 boolean 그대로
        }
        if(isLeftObstacle)
            isLeftObstacle = 0;
        cnt = 0;
        isRightObstacle = 1;
    }

    if(isLeftObstacle)
        printf("Find LeftObstacle! distance Obstacle : %d \n", closestLeftPoint.x);
    if(isRightObstacle)
        printf("Find RightObstacle! distance Obstacle : %d \n", closesRightPoint.x);
}
void generate_control_msg(race::drive_values* control_msg) {
    int speed = MAX_SPEED;
    const int Xshift = 320;
    const int Xspeed = 5;
    float op_error;
    Point op;
    Point pa_1 = ld->p1;
    Point pa_2 = ld->p2;
    Point pb_1 = ld->p3;
    Point pb_2 = ld->p4;

    pb_1.x += 640;
    pb_2.x += 640;

    //오른쪽 장애물 발견 시 오른쪽 카메라 차선을 Xshift 만큼 왼쪽으로 이동
    if(isRightObstacle == 1)
    {
        speed = Xspeed;
        pb_1.x -= Xshift;
        pb_2.x -= Xshift;
    }
    //왼쪽 장애물 발견 시 왼쪽 카메라 차선을 Xshift 만큼 오른쪽으로 이동
    if(isLeftObstacle == 1)
    {
        speed = Xspeed;
        pa_1.x += Xshift;
        pa_2.x += Xshift;
    }

    if(ld->get_intersectpoint(pa_1, pa_2, pb_1, pb_2, &op)) {
        float error_steering = CENTER_POINT - op.x;
        steering = p_steering * error_steering * (float)(1/(float)speed) * 5;
    }
    else if(ld->is_left_error()) {
        steering = -p_steering_curve / ld->get_right_slope() * (float)(1/(float)speed) * 5;
    }
    else if(ld->is_right_error()) {
        steering = p_steering_curve / ld->get_left_slope() * (float)(1/(float)speed) * 5;
    }

    steering = min(max(steering, -100), 100);
    //printf("steering : %d\n", steering);
    steering += 100;
    op_error = cal_lookahead_op_error();
    //printf("lookahead : %f\n", op_error);
    speed = (int)round((float)speed - fabs(op_error * p_lookahead));
    speed = min(max(speed, 5), MAX_SPEED);
    printf("speed : %d\n", speed);
    control_msg->steering = steering;
    control_msg->throttle = speed;
}
float cal_lookahead_op_error() {
    Point op;
    Point pa_1 = ld->p1;
    Point pa_2 = ld->p2;
    Point pb_1 = ld->p3;
    Point pb_2 = ld->p4;
    float error_op;
    pa_1.x += 200;
    pa_2.x += 200;
    pb_1.x += 640;
    pb_2.x += 640;
    if(la->get_intersectpoint(pa_1, pa_2, pb_1, pb_2, &op)) {
        error_op = CENTER_POINT_LA - op.x;
    }
    else if(ld->is_left_error()) {
        error_op = CENTER_POINT_LA - p_lookahead_curve / la->get_right_slope();
    }
    else if(ld->is_right_error()) {
        error_op = CENTER_POINT_LA + p_lookahead_curve / la->get_left_slope();
    }
    else {
        error_op = 0;
    }
    return error_op;
}
