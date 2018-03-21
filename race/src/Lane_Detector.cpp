#include <iostream>
#include <fstream>
#include <ctime>
#include <queue>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <unistd.h>
#include <highgui.h>
#include "opencv2/opencv.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <ros/ros.h>
#include <race/drive_values.h>
#include <race/control_variables.h>

#define P_SLOPE 2.0f
#define P_POSITON -2.0f

using namespace cv;
using namespace std;

const CvScalar COLOR_BLUE = CvScalar(255, 0, 0);
const CvScalar COLOR_RED = CvScalar(0, 0, 255);

queue <int> buffer_x;

class Lane_Detector {
private :
    Point p1, p2, p3, p4;
    float p_slope;
    float p_position;
    float left_slope;
    float right_slope;
    float left_length;
    float right_length;
    VideoCapture capture_left;
    VideoCapture capture_right;
    Mat originImg_left;
    Mat originImg_right;
    bool left_error;
    bool right_error;
    Mat grayImg1, grayImg2, otsu, sobelX_Img, sobelY_Img, sobel_Img1, sobel_Img2,
        imageROI1, imageROI2, blured1, blured2, mask, openingImg1, openingImg2,
        cannyImg1, cannyImg2, houghImg1, houghImg2;
    void v_roi(Mat& img, Mat& img_ROI, const Point& p1, const Point& p2);
    void region_of_interest_L(Mat& img, Mat& img_ROI);
    void region_of_interest_R(Mat& img, Mat& img_ROI);
    bool hough_left(Mat& img, Point* p1, Point* p2);
    bool hough_right(Mat& img, Point* p1, Point* p2);
    bool get_intersectpoint(const Point& AP1, const Point& AP2,
                           const Point& BP1, const Point& BP2, Point* IP);
    float get_slope(const Point& p1, const Point& p2);
    int position(const Point P1, const Point P2);
public :
    Lane_Detector(){}
    void init();
    void operate();
    void set_control_variables(float p_slope, float p_position);
    float get_left_slope();
    float get_right_slope();
    float get_left_length();
    float get_right_length();
};
void Lane_Detector::set_control_variable(float p_slope, float p_position) {
    this->p_slope = p_slope;
    this->p_position = p_position;
}
int Lane_Detector::position(const Point P1, const Point P3) {
    float x_L;
    float x_R;
    const float y = 480;

    x_L = (y - P1.y + left_slope * P1.x) / left_slope;
    left_length = 640 - x_L;

    x_R = (y - P3.y + right_slope * P3.x) / right_slope;
    right_length = x_R;
}

float Lane_Detector::get_left_length() {
    return left_length;
}

float Lane_Detector::get_right_length() {
    return right_length;
}

float Lane_Detector::get_left_slope() {
    return left_slope;
}
float Lane_Detector::get_right_slope() {
    return right_slope;
}

void Lane_Detector::init(){
  capture_left = VideoCapture(2);
  capture_right = VideoCapture(1);

  mask = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1, 1));

  left_error = false;
  right_error = false;
  left_length = 0;
  right_length = 0;
  p_slope = P_SLOPE;
  p_position = P_POSITON;
}

void Lane_Detector::operate(){
    capture_left >> originImg_left;
    capture_right >> originImg_right;

    if (originImg_left.empty()){
      cerr << "Empty Left Image" << endl;
      return;
    }
    if (originImg_right.empty()){
      cerr << "Empty right Image" << endl;
      return;
    }

    cvtColor(originImg_left, grayImg1, COLOR_BGR2GRAY);
    cvtColor(originImg_right, grayImg2, COLOR_BGR2GRAY);

    threshold(grayImg1, blured1, 200, 255, THRESH_BINARY );
    threshold(grayImg2, blured2, 200, 255, THRESH_BINARY );


    if(!left_error){
      v_roi(blured1, imageROI1, p1, p2);
    }
    else{
      region_of_interest_L(blured1, imageROI1);
    }
    if(!right_error){
      v_roi(blured2, imageROI2, p3, p4);
    }
    else{
      region_of_interest_R(blured2, imageROI2);
    }

    morphologyEx(imageROI1, openingImg1, MORPH_OPEN, mask);
    morphologyEx(imageROI2, openingImg2, MORPH_OPEN, mask);

    Sobel(openingImg1, sobelX_Img, CV_8U, 1, 0);
    Sobel(openingImg1, sobelY_Img, CV_8U, 0, 1);
    sobel_Img1 = abs(sobelX_Img) + abs(sobelY_Img);

    Sobel(openingImg2, sobelX_Img, CV_8U, 1, 0);
    Sobel(openingImg2, sobelY_Img, CV_8U, 0, 1);
    sobel_Img2 = abs(sobelX_Img) + abs(sobelY_Img);

    Canny(grayImg1, cannyImg1, 150, 300);
    Canny(grayImg2, cannyImg2, 150, 300);

    //
    // imshow("canny", cannyImg1);
    // imshow("canny1", cannyImg2);

    left_error = hough_left(sobel_Img1, &p1, &p2);
    right_error = hough_right(sobel_Img2, &p3, &p4);
    
    circle(originImg_left, Point(640, 480), 20, COLOR_BLUE, 5);
    circle(originImg_right, Point(0, 480), 20, COLOR_BLUE, 5);

    circle(originImg_left, Point(left_length, 480), 20, COLOR_BLUE, 5);
    circle(originImg_right, Point(right_length, 480), 20, COLOR_BLUE, 5);

    line(originImg_left, p1, p2, COLOR_RED, 4, CV_AA);
    line(originImg_right, p3, p4, COLOR_RED, 4, CV_AA);

    cout << p1.x << " " << p1.y << " " << p2.x << " " << p2.y << endl;

    left_slope = get_slope(p1, p2);
    right_slope = get_slope(p3, p4);
    position(p1, p3);
    imshow("Left", originImg_left);
    imshow("Right", originImg_right);
    if(waitKey(10) == 0){
      return;
    }
}

void Lane_Detector::v_roi(Mat& img, Mat& img_ROI, const Point& p1, const Point& p2){

  Point a = Point(p1.x - 30, p1.y);
  Point b = Point(p1.x + 30, p1.y);
  Point c = Point(p2.x + 30, p2.y);
  Point d = Point(p2.x - 30, p2.y);

  vector <Point> Left_Point;

  Left_Point.push_back(a);
  Left_Point.push_back(b);
  Left_Point.push_back(c);
  Left_Point.push_back(d);

  Mat roi(img.rows, img.cols, CV_8U, Scalar(0));

  fillConvexPoly(roi, Left_Point, Scalar(255));

  Mat filteredImg_Left;
  img.copyTo(filteredImg_Left, roi);


  img_ROI = filteredImg_Left.clone();

  // imshow("V_ROI", img_ROI);
}


void Lane_Detector::region_of_interest_L(Mat& img, Mat& img_ROI){
  Point a = Point(0, img.rows/2);
  Point b = Point(img.cols, 0);
  Point c = Point(img.cols, img.rows);
  Point d = Point(0, img.rows);

  vector <Point> Left_Point;

  Left_Point.push_back(a);
  Left_Point.push_back(b);
  Left_Point.push_back(c);
  Left_Point.push_back(d);


  Mat roi(img.rows, img.cols, CV_8U, Scalar(0));

  fillConvexPoly(roi, Left_Point, Scalar(255));

  Mat filteredImg_Left;
  img.copyTo(filteredImg_Left, roi);

  img_ROI = filteredImg_Left.clone();

  // imshow("Img_ROI", img_ROI);
}

void Lane_Detector::region_of_interest_R(Mat& img, Mat& img_ROI){
  Point a = Point(img.cols, img.rows);
  Point b = Point(img.cols, img.rows/2);
  Point c = Point(0, 0);
  Point d = Point(0, img.rows);

  vector <Point> Left_Point;

  Left_Point.push_back(a);
  Left_Point.push_back(b);
  Left_Point.push_back(c);
  Left_Point.push_back(d);


  Mat roi(img.rows, img.cols, CV_8U, Scalar(0));

  fillConvexPoly(roi, Left_Point, Scalar(255));

  Mat filteredImg_Left;
  img.copyTo(filteredImg_Left, roi);


  img_ROI = filteredImg_Left.clone();

  Left_Point.push_back(c);
  // imshow("Img_ROIa", Img_ROI);
}

float Lane_Detector::get_slope(const Point& p1, const Point& p2){

  float slope;

  slope = ((float) p2.y - (float) p1.y) / ((float) p2.x - (float) p1.x);

  return slope;
}

bool Lane_Detector::get_intersectpoint(const Point& AP1, const Point& AP2,
                       const Point& BP1, const Point& BP2, Point* IP)
{
    double t;
    double s;
    double under = (BP2.y-BP1.y)*(AP2.x-AP1.x)-(BP2.x-BP1.x)*(AP2.y-AP1.y);
    if(under==0) return false;

    double _t = (BP2.x-BP1.x)*(AP1.y-BP1.y) - (BP2.y-BP1.y)*(AP1.x-BP1.x);
    double _s = (AP2.x-AP1.x)*(AP1.y-BP1.y) - (AP2.y-AP1.y)*(AP1.x-BP1.x);

    t = _t/under;
    s = _s/under;

    if(t<0.0 || t>1.0 || s<0.0 || s>1.0) return false;
    if(_t==0 && _s==0) return false;

    IP->x = AP1.x + t * (double)(AP2.x-AP1.x);
    IP->y = AP1.y + t * (double)(AP2.y-AP1.y);

    return true;
}

bool Lane_Detector::hough_left(Mat& img, Point* p1, Point* p2){

  vector<Vec2f> linesL;

  Point point1;
  Point point2;

  int count = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  int threshold = 60;

  for (int i = 10; i > 0; i--){
    HoughLines(img, linesL, 1, CV_PI / 180, threshold, 0, 0, 0, CV_PI /2);
    if (linesL.size() > 1){
      for (size_t i = 0; i < linesL.size(); i++){
        count ++;
        float rho = linesL[i][0];
        float theta = linesL[i][1];
        double a = cos(theta), b = sin(theta);
        double x0 = a * rho, y0 = b * rho;

        int _x1 = int(x0 + 1000*(-b));
        int _y1 = int(y0 + 1000*(a));
        int _x2 = int(x0 - 1000*(-b));
        int _y2 = int(y0 - 1000*(a));

        point1.x = _x1; point1.y = _y1;
        point2.x = _x2; point2.y = _y2;


        x1 += point1.x;
        y1 += point1.y;

        x2 += point2.x;
        y2 += point2.y;

      }
      break;
    }
  }

  if (count != 0){

    x1 /= count;
    y1 /= count;

    x2 /= count;
    y2 /= count;

    p1->x = x1; p1->y = y1;
    p2->x = x2; p2->y = y2;

    return false;

  }

  return true;

}

bool Lane_Detector::hough_right(Mat& img, Point* p1, Point* p2){

  vector<Vec2f> linesR;

  Point point1;
  Point point2;

  int count = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  int threshold = 30;

  for (int i = 10; i > 0; i--){
    HoughLines(img, linesR, 1, CV_PI / 180, threshold, 0, 0, CV_PI/2, CV_PI);
    if (linesR.size() > 1){
      for (size_t i = 0; i < linesR.size(); i++){
        count ++;
        float rho = linesR[i][0];
        float theta = linesR[i][1];
        double a = cos(theta), b = sin(theta);
        double x0 = a * rho, y0 = b * rho;

        int _x1 = int(x0 + 1000*(-b));
        int _y1 = int(y0 + 1000*(a));
        int _x2 = int(x0 - 1000*(-b));
        int _y2 = int(y0 - 1000*(a));

        point1.x = _x1; point1.y = _y1;
        point2.x = _x2; point2.y = _y2;


        x1 += point1.x;
        y1 += point1.y;

        x2 += point2.x;
        y2 += point2.y;

      }
      break;
    };
  }

  if (count != 0){

    x1 /= count;
    y1 /= count;

    x2 /= count;
    y2 /= count;

    p1->x = x1; p1->y = y1;
    p2->x = x2; p2->y = y2;

    return false;

  }

  return true;
}

void testerCallback(const race::control_variables &msg) {
    set_control_variables(msg.p_slope, msg.p_position);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Controller");
    ros::Subscriber sub = nh.subscribe("control_variables", 1000, testerCallback);
    ros::NodeHandle nh;
    ros::Publisher control_pub = nh.advertise<race::drive_values>("Control", 1000);
    race::drive_values control_msg;
    Lane_Detector* ld = new Lane_Detector();
    ld->init();
    
    while(true) {
        ld->operate();
        float error_slope = ld->get_left_slope() + ld->get_right_slope();
        float error_position = ld->get_left_length() - ld->get_right_length();
        cout << "position : " << error_position << endl;
        int control = p_slope * error_slope + 15 + p_position * error_position;
        if(control > 100) control = 100;
        if(control < -100) control = -100;
        control_msg.steering = control + 100;
        control_msg.throttle = 1550;
        cout << "============================" << endl;
        cout << "control : " << control << endl;
        cout << "left : " << ld->get_left_slope() << endl;
        cout << "right : " << ld->get_right_slope() << endl;
        cout << "left_length : " << ld->get_left_length() << endl;
        cout << "right_length : " << ld->get_right_length() << endl;
        cout << "============================" << endl;
        control_pub.publish(control_msg);
        ros::spinOnce();
    }
    delete ld;
    return 0;
}
