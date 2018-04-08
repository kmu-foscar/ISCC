#include <ros/ros.h>
#include <obstacle_detector/Obstacles.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <race/drive_values.h>
#include <algorithm>
#include <stdlib.h>
#include <math.h>
 
#define O_LIMIT 1
#define PI 3.141592
#define MAX 12345
 
ros::Publisher pub;
 
using namespace std;

//점들 사이 거리
double ObstacleDistanceSquare(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    return (p2.y - p1.y) * (p2.y - p1.y) + (p2.x - p1.x) * (p2.x - p1.x);
}
 
//Data를 이용하여 점 구하기
//없어도 구해야 된다 다음 함수가 실행될 수 있게
void FindClosestPointLR(obstacle_detector::Obstacles data, bool find[], geometry_msgs::Point pl1, 
    geometry_msgs::Point pl2, geometry_msgs::Point pr1, geometry_msgs::Point pr2)
{
    int size = data.circles.size();
    geometry_msgs::Point car;
    car.y = 0, car.x = 0;
    geometry_msgs::Point closestLeftPoint, closestRightPoint;
    double minimumL = MAX, minimumR = MAX;
    
    //좌우의 가장 가까운 점을 찾는다.
    for(int i = 0; i < size; i++)
    {
        geometry_msgs::Point curPoint = data.circles[i].center;
        if(curPoint.y == 0 || curPoint.x == 0)
            continue;
        //[-PI, PI]
        double angle = atan2(curPoint.x / curPoint.y, 1.0);
        if(angle < 0)
            continue;
 
        double dist = ObstacleDistanceSquare(car, curPoint);  
        //오른쪽
        if(angle < PI / 2.0)
            if(minimumR > dist) {
                minimumR = dist;
                closestRightPoint = curPoint;
            }
        //왼쪽
        else 
            if(minimumL > dist){
                minimumL = dist;
                closestLeftPoint = curPoint;
            }
    }
 
    //오른쪽에 있으면
    if(minimumR < O_LIMIT)
    {
        find[2] = true;
        pr1 = closestRightPoint;
    }
    //왼쪽에 있으면
    if(minimumL < O_LIMIT)
    {
        find[0] = true;
        pl1 = closestLeftPoint;
    }
    
    minimumL = MAX, minimumR = MAX;
 
    //구한 점들과 가장 가까운 점들을 구한다.
    for(int i = 0; i < size; i++)
    {
        geometry_msgs::Point curPoint = data.circles[i].center;
        if(curPoint.y == 0 || curPoint.x == 0)
            continue;
        
        double angle = atan2(curPoint.x / curPoint.y, 1.0);
        
        if(angle < 0)
            continue;
 
        //오른쪽
        if(angle < PI / 2.0){
            double dist = ObstacleDistanceSquare(pr1, curPoint);  
 
            if(minimumR > dist && !dist) {
                minimumR = dist;
                closestRightPoint = curPoint;
            }
        }
        else{
            double dist = ObstacleDistanceSquare(pl1, curPoint);  
        
            if(minimumL > dist && !dist){
                minimumL = dist;
                closestLeftPoint = curPoint;
            }
        }
    }
 
    //오른쪽에 있으면
    if(minimumR < O_LIMIT)
    {
        find[3] = true;
        pr2 = closestRightPoint;
    }
    //왼쪽에 있으면
    if(minimumL < O_LIMIT)
    {
        find[1] = true;
        pl2 = closestLeftPoint;
    }
}
//좌우 기울기 계산
void CalSlopeLine(double lineSlope,geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    lineSlope = (p2.x - p1.x) / (p2.y - p1.y);
}
 
//수직 기울기 계산
void VelticalityLine(double &lineSlope)
{
    lineSlope = (-1.0) / lineSlope;
}
 
//사이각 계산 return radian
double AngleBetweenLine(double &lineSlopeL, double &lineSlopeR)
{
    return atan2(lineSlopeL, 1.0) - atan2(lineSlopeR, 1.0);
}
//조향값 계산 return radian
double FindSteering( double &lineSlopeL, double &angle)
{
    double newSlope = tan(atan2(lineSlopeL, 1.0) - angle / 2.0);
    return atan2(newSlope, 1.0);
}
void calculator(const obstacle_detector::Obstacles data)
{
    //left obstacle pointer
    geometry_msgs::Point pl1, pl2;
    //right obstacle pointer
    geometry_msgs::Point pr1, pr2;
 
    //left Slope, Right Slope
    double lineSlopeL, lineSlopeR;
 
    //점을 찾으면 1 못찾으면 0
    //left point 0, left point2 1, right point 3, right point2 4
    bool find[4] = { false, false, false, false };
    FindClosestPointLR(data, find, pl1, pl2, pr1, pr2);
    
    double angle, steering;
    if(!find[0] || !find[3])
        return;
    //왼쪽 오른쪽 한개씩만 검출 시
    if(!find[2] && !find[4])
    {
 
        geometry_msgs::Point p;
        p.x = (pr1.x - pl1.x);
        p.y = (pr1.y - pl1.y);
 
        steering = atan2(p.x / p.y, 1.0);
    }
 
    //오른쪽만 검출시
    else if(!find[2])
    {
        CalSlopeLine(lineSlopeR, pr1, pr2);
        VelticalityLine(lineSlopeR);
        
        lineSlopeL = pl1.x / pl1.y;
 
        angle = AngleBetweenLine(lineSlopeL, lineSlopeR);
        steering = FindSteering(lineSlopeL, angle);
    }
    //왼쪽만 검출시
    else if(!find[4])
    {   
        CalSlopeLine(lineSlopeL, pl1, pl2);
        VelticalityLine(lineSlopeL);
        
        lineSlopeR = pr1.x / pr1.y;
 
        angle = AngleBetweenLine(lineSlopeL, lineSlopeR);
        steering = FindSteering(lineSlopeL, angle);
    }
    //둘 다 검출 시
    else
    {
        CalSlopeLine(lineSlopeL, pl1, pl2);
        CalSlopeLine(lineSlopeR, pr1, pr2);
        
        VelticalityLine(lineSlopeL);
        VelticalityLine(lineSlopeR);
        
        angle = AngleBetweenLine(lineSlopeL, lineSlopeR);
        steering = FindSteering(lineSlopeL, angle);
    }
 
    race::drive_values msg;
    msg.steering = ((steering / PI) * 100) + 100;
    msg.throttle = 2;
    printf("steering : %f speed : %f\n", steering, 2.0);
    pub.publish(msg);
}
 
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "PID_Contoller_node");
    ros::NodeHandle nh;
	
    ros::Subscriber sub = nh.subscribe("raw_obstacles", 1, calculator);
    pub = nh.advertise<race::drive_values> ("Control", 100);
	ros::spin();
}
