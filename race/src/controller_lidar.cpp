#include <ros/ros.h>
#include <obstacle_detector/Obstacles.h>
#include <geometry_msgs/Point.h>
#include <algorithm>
#include <stdlib.h>
#include <math.h>
#include <config.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <race/drive_values.h>
using namespace std;

//장애물 인식범위
#define O_LIMIT 2
//장애물간 거리
#define O_B_LIMIT 1.1
#define O_B_LIMIT_POW O_B_LIMIT * O_B_LIMIT
#define O_LIMIT_POW O_LIMIT * O_LIMIT
#define PI 3.141592
#define MAX 10 

//hyepro K_L only 1.3 K_R 1.0
#define K_L 1.6
#define K_R 1.6
#define OBSTACLE_THRESHOLD 30

ros::Publisher pub;
ros::Publisher return_sig_pub;
ros::Subscriber oa_onoff_sub;
race::drive_values msg;
std_msgs::Int16 return_msg;

bool oa_onoff = false;
bool isdetected = false;
int oa_cnt = 0;

int vo_size;

void oa_onoffCallback(const std_msgs::Bool &msg)
{
    oa_onoff = msg.data;
}

//점들 사이 거리
double ObstacleDistanceSquare(geometry_msgs::Point &p1, geometry_msgs::Point &p2)
{
    return (p2.y - p1.y) * (p2.y - p1.y) + (p2.x - p1.x) * (p2.x - p1.x);
}

//Data를 이용하여 점 구하기
//없어도 구해야 된다 다음 함수가 실행될 수 있게
void FindClosestPointLR(obstacle_detector::Obstacles &data, bool find[],
                        geometry_msgs::Point &pl1, geometry_msgs::Point &pl2,
                        geometry_msgs::Point &pr1, geometry_msgs::Point &pr2)
{
    int size = data.circles.size();
    //차 위치
    geometry_msgs::Point car;
    car.y = 0.0, car.x = 0.0;

    //왼쪽에서 제일 가까운 점과 오른쪽에서 제일 가까운 점
    geometry_msgs::Point closestLeftPoint, closestRightPoint;
    closestLeftPoint.x = MAX, closestRightPoint.y = MAX;

    //찾은 최소 거리
    double minimumL = MAX, minimumR = MAX;
    //좌우의 가장 가까운 점을 찾는다.
    for(int i = 0; i < size; i++)
    {
        geometry_msgs::Point curPoint = data.circles[i].center;
        //분모나 분자가 0이면 atan 작동 안됨.
        if(curPoint.y == 0.0 || curPoint.x == 0.0)
            continue;
        //return이 [-PI, PI]
        double angle = atan2(curPoint.y, curPoint.x);

        //1,2사분면이 아니면 다음 포인트
        if(angle < 0.0)
            continue;

        //점과 차 사이의 거리
        double dist = ObstacleDistanceSquare(car, curPoint);

        //오른쪽 PI/2 보다 크면 원래 왼쪽구역이지만, x축이 +,- 뒤집혀 있기 때문에 오른쪽구역이다.
        if(angle > PI / 2.0){
            if(minimumR > dist) {
                minimumR = dist;
                closestRightPoint = curPoint;
            }
        }
        //왼쪽
        else{
            if(minimumL > dist){
                minimumL = dist;
                closestLeftPoint = curPoint;
            }
        }
    }
    //제일 가까운 오른쪽 장애물이 O_LIMIT_POW보다 크면 무시.
    if(minimumR < O_LIMIT_POW)
    {
        find[2] = true;
        pr1 = closestRightPoint;
    }
    //왼쪽에 있으면
    if(minimumL < O_LIMIT_POW)
    {
        find[0] = true;
        pl1 = closestLeftPoint;
    }

    minimumL = MAX, minimumR = MAX;
    closestLeftPoint.x = MAX, closestRightPoint.y = MAX;

    //구한 점들과 가장 가까운 점들을 구한다.
    //좌우의 가장 가까운 점을 찾는다.
    for(int i = 0; i < size; i++)
    {
        geometry_msgs::Point curPoint = data.circles[i].center;
        //분모나 분자가 0이면 atan 작동 안됨.
        if(curPoint.y == 0.0 || curPoint.x == 0.0)
            continue;
        //return이 [-PI, PI]
        double angle = atan2(curPoint.y, curPoint.x);

        //1,2사분면이 아니면 다음 포인트
        if(angle < 0.0)
            continue;

        //오른쪽 PI/2 보다 크면 원래 왼쪽구역이지만, x축이 +,- 뒤집혀 있기 때문에 오른쪽구역이다.
        if(angle > PI / 2.0){
            double dist = ObstacleDistanceSquare(pr1, curPoint);

            if(minimumR > dist && dist) {
                minimumR = dist;
                closestRightPoint = curPoint;
            }
        }
        //왼쪽
        else{
            double dist = ObstacleDistanceSquare(pl1, curPoint);

            if(minimumL > dist && dist){
                minimumL = dist;
                closestLeftPoint = curPoint;
            }
        }
    }

    //오른쪽에 있으면
    if(minimumR < O_B_LIMIT_POW)
    {
        find[3] = true;
        pr2 = closestRightPoint;
    }
    //왼쪽에 있으면
    if(minimumL < O_B_LIMIT_POW)
    {
        find[1] = true;
        pl2 = closestLeftPoint;
    }
}
//기울기 계산
void CalSlopeLine(double &lineSlope, geometry_msgs::Point &p1, geometry_msgs::Point &p2)
{
    lineSlope = (p2.y - p1.y) / (p2.x - p1.x);
}

//수직 기울기 계산
void VelticalityLine(double &lineSlope)
{
    lineSlope = (-1.0) / lineSlope;
}

//사이각 계산 return radian
double FindSteering(double &lineSlopeR, double &lineSlopeL)
{
    double angleR, angleL;
    angleR = atan2(-lineSlopeR, -1.0);
    angleL = atan2(lineSlopeL, 1.0);
    double ret = (angleL + angleR) / 2;
    return ret < 0 ? PI + ret : ret;
}
void calculator(obstacle_detector::Obstacles data)
{
    if(!oa_onoff){
        return;
    }

    if(!isdetected) // OA mode starting condition
    {
        isdetected = data.circles.size() >= 10 ? true : false;
	if(isdetected) {
	    return_msg.data = RETURN_OPERATE;
            return_sig_pub.publish(return_msg);
	}
	else {
	    return_msg.data = RETURN_STOP;
            return_sig_pub.publish(return_msg);
	}
    }

    //left obstacle pointer
    geometry_msgs::Point pl1, pl2;
    //right obstacle pointer
    geometry_msgs::Point pr1, pr2;

    //left Slope, Right Slope
    double lineSlopeL, lineSlopeR;
    vo_size = 0;
    //y 축과 x축을 바꾼다.
    geometry_msgs::Point car;
    car.x = 0; car.y = 0;
    for(int i = 0; i < data.circles.size(); i++)
    {
	double temp = data.circles[i].center.x;
	data.circles[i].center.x = data.circles[i].center.y;
	data.circles[i].center.y = temp;
	if(ObstacleDistanceSquare(car, data.circles[i].center) <= 25.f) {
		vo_size++;
	}
    }
    if(isdetected && vo_size == 0)
        ++oa_cnt;
    else
        oa_cnt = 0;
    if(isdetected && oa_cnt >= OBSTACLE_THRESHOLD) { // oa off
        return_msg.data = RETURN_FINISH;
        return_sig_pub.publish(return_msg);
	isdetected = false;
    }
    //점을 찾으면 true 못찾으면 false
    //left point 0, left point2 1, right point 2, right point2 3
    bool find[4] = { false, false, false, false };
	FindClosestPointLR(data, find, pl1, pl2, pr1, pr2);

    double steering, RFlag, LFlag;


    //좌우 첫번째 장애물만 검출 시
    if(!find[1] || !find[3])
    {
        geometry_msgs::Point p;
        p.x = (pl1.x + pr1.x) / 2;
        p.y = (pl1.y + pr1.y) / 2;

        steering = atan2(p.y, p.x);
    }
    //좌우 두번째 장애물 중 오른쪽만 검출시
    else if(!find[1] && find[3])
    {
        geometry_msgs::Point p;
        p.x = (pr1.x + pr2.x) / 2;
        p.y = (pr1.y + pr2.y) / 2;

        p.x = (p.x + pl1.x) / 2;
        p.y = (p.y + pl1.y) / 2;

        steering = atan2(p.y, p.x);
    }
    //좌우 두번째 장애물 중 왼쪽만 검출시
    else if(!find[3] && find[1])
    {
        geometry_msgs::Point p;
        p.x = (pl1.x + pl2.x) / 2;
        p.y = (pl1.y + pl2.y) / 2;

        p.x = (p.x + pr1.x) / 2;
        p.y = (p.y + pr1.y) / 2;

        steering = atan2(p.y, p.x);
    }
    //네 점 다 검출 시
    else if(find[1] && find[3])
    {
        CalSlopeLine(lineSlopeL, pl1, pl2);
        CalSlopeLine(lineSlopeR, pr1, pr2);

        VelticalityLine(lineSlopeL);
        VelticalityLine(lineSlopeR);

        steering = FindSteering(lineSlopeR, lineSlopeL);
    }
    steering = steering / PI * 200;
    steering -= 100;
    if(steering <= 0)
	steering *= K_L; // -100 * K_S < steering * K_S < 100 * K_S
    else
	steering *= K_R;

    msg.steering = steering + 100;
    if(msg.steering < 0)
	msg.steering = 0;
    else if(msg.steering > 200)
	msg.steering = 200;
    msg.steering -= 6;   
    msg.throttle = 7;
    
    //printf("steering : %d speed : %d\n", msg.steering, 1);
    if(isdetected) {
    	pub.publish(msg);
	printf("steering : %d speed : %d\n", msg.steering, msg.throttle);
    }
}

int main(int argc,	 char* argv[])
{
    ros::init(argc, argv, "PID_Contoller_node");
	ros::NodeHandle nh;
  oa_onoff_sub = nh.subscribe("oa_onoff", 1, oa_onoffCallback);
	ros::Subscriber sub = nh.subscribe("raw_obstacles", 1, calculator);
	pub = nh.advertise<race::drive_values> ("Control", 1000);
	return_sig_pub = nh.advertise<std_msgs::Int16>("return_signal", 1);
	ros::spin();
}
