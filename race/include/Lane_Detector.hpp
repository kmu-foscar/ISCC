#ifndef LANE_DETECTOR_H
#define LANE_DETECTOR_H

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
#include <string.h>
#include <sys/time.h>
using namespace cv;
using namespace std;

const CvScalar COLOR_BLUE = CvScalar(255, 0, 0);
const CvScalar COLOR_RED = CvScalar(0, 0, 255);

const Vec3b RGB_WHITE_LOWER = Vec3b(100, 100, 190);
const Vec3b RGB_WHITE_UPPER = Vec3b(255, 255, 255);
const Vec3b RGB_YELLOW_LOWER = Vec3b(225, 180, 0);
const Vec3b RGB_YELLOW_UPPER = Vec3b(255, 255, 170);
const Vec3b HSV_YELLOW_LOWER = Vec3b(10, 20, 130);
const Vec3b HSV_YELLOW_UPPER = Vec3b(30, 140, 255);

const Vec3b HLS_YELLOW_LOWER = Vec3b(20, 120, 80);
const Vec3b HLS_YELLOW_UPPER = Vec3b(45, 200, 255);

string to_string(int n) {
	stringstream s;
	s << n;
	return s.str();
}

class Lane_Detector {
private :
    float left_slope;
    float right_slope;
    float left_length;
    float right_length;

    VideoCapture capture_left;
    VideoCapture capture_right;
    VideoWriter output_video;

    Mat originImg_left;
    Mat originImg_right;

    bool left_error;
    bool right_error;

		Mat img_hsv, filterImg1, filterImg2, binaryImg1, binaryImg2,
				imageROI1, imageROI2, blured1, blured2, mask, openingImg1, openingImg2,
				cannyImg1, cannyImg2, houghImg1, houghImg2;

    void v_roi(Mat& img, Mat& img_ROI, const Point& p1, const Point& p2);
    void region_of_interest_L(Mat& img, Mat& img_ROI);
    void region_of_interest_R(Mat& img, Mat& img_ROI);
    bool hough_left(Mat& img, Point* p1, Point* p2);
    bool hough_right(Mat& img, Point* p1, Point* p2);
    float get_slope(const Point& p1, const Point& p2);
    int position(const Point P1, const Point P2);
public :
    Point p1, p2, p3, p4;
    Lane_Detector(){}
    void init();
    void operate();
    float get_left_slope();
    float get_right_slope();
    float get_left_length();
    float get_right_length();
    bool is_left_error();
    bool is_right_error();
    bool get_intersectpoint(const Point& AP1, const Point& AP2,
                           const Point& BP1, const Point& BP2, Point* IP);
};

bool Lane_Detector::is_left_error() {
  return left_error;
}
bool Lane_Detector::is_right_error() {
  return right_error;
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
  string path = "/home/nvidia/ISCC_Videos/";
  struct tm* datetime;
  time_t t;
  t = time(NULL);
  datetime = localtime(&t);
  string s_t = path.append(to_string(datetime->tm_year + 1900)).append("-").append(to_string(datetime->tm_mon + 1)).append("-").append(to_string(datetime->tm_mday)).append("_").append(to_string(datetime->tm_hour)).append(":").append(to_string(datetime->tm_min)).append(":").append(to_string(datetime->tm_sec)).append(".avi");

  capture_left = VideoCapture(2);
  capture_right = VideoCapture(1);
  output_video.open(s_t, VideoWriter::fourcc('X', 'V', 'I', 'D'), 20, Size(1280, 480), true);
  mask = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1, 1));

  left_error = false;
  right_error = false;
  left_length = 0;
  right_length = 0;
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


	GaussianBlur(originImg_left, filterImg1, Size(5, 5), 0);
	GaussianBlur(originImg_right, filterImg2, Size(5, 5), 0);

	cvtColor(filterImg1, img_hsv, COLOR_BGR2HSV);

	imshow("img_hsv", img_hsv);

	inRange(img_hsv, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER, binaryImg1);
	inRange(originImg_right, RGB_WHITE_LOWER, RGB_WHITE_UPPER, binaryImg2);

	imshow("mask", binaryImg1);

	Canny(binaryImg1, cannyImg1, 130, 270);
	Canny(filterImg2, cannyImg2, 130, 270);

	imshow("canny1", cannyImg1);

	Mat initORI1;
	Mat initORI2;

	region_of_interest_L(originImg_left, initORI1);
	region_of_interest_R(originImg_right, initORI2);

	imshow("initORI1", initORI1);
	imshow("initORI2", initORI2);



	if(!left_error){
		Mat ddd;
		v_roi(originImg_left, ddd, p1, p2);
		imshow("v_roi_check", ddd);
		v_roi(cannyImg1, imageROI1, p1, p2);
	}
	else{
		region_of_interest_L(cannyImg1, imageROI1);
	}

	if(!right_error){
		v_roi(cannyImg2, imageROI2, p3, p4);
	}
	else{
		region_of_interest_R(cannyImg2, imageROI2);
	}

	left_error = hough_left(imageROI1, &p1, &p2);
	right_error = hough_right(imageROI2, &p3, &p4);

	line(originImg_left, p1, p2, COLOR_RED, 4, CV_AA);
	line(originImg_right, p3, p4, COLOR_RED, 4, CV_AA);

	cout << p1.x << " " << p1.y << " " << p2.x << " " << p2.y << endl;

	left_slope = get_slope(p1, p2);
	right_slope = get_slope(p3, p4);
	position(p1, p3);

	Mat a;
	Mat b;
	Mat c;

	resize(originImg_left, a, Size(640, 480), 0, 0, CV_INTER_LINEAR);
	resize(originImg_right, b, Size(640, 480), 0, 0, CV_INTER_LINEAR);
	hconcat(a, b, c);

	imshow("result", c);

	// output_video << c;
	if(waitKey(10) == 0){
		return;
	}
}

void Lane_Detector::v_roi(Mat& img, Mat& img_ROI, const Point& p1, const Point& p2){


  float slope = get_slope(p1, p2);
  float alphaY = 50.f / sqrt(slope*slope + 1);
  float alphaX = slope * alphaY;

	Point a(p1.x - alphaX, p1.y + alphaY );
  Point b(p1.x + alphaX, p1.y - alphaY );
  Point c(p2.x, p2.y);

  vector <Point> Left_Point;

  Left_Point.push_back(a);
  Left_Point.push_back(b);
  Left_Point.push_back(c);

  Mat roi(img.rows, img.cols, CV_8U, Scalar(0));

  fillConvexPoly(roi, Left_Point, Scalar(255));

  Mat filteredImg_Left;
  img.copyTo(filteredImg_Left, roi);


  img_ROI = filteredImg_Left.clone();

}


void Lane_Detector::region_of_interest_L(Mat& img, Mat& img_ROI){
  Point a = Point(0, 0);
  Point b = Point(0, img.rows);
  Point c = Point(img.cols, img.rows/5);
  Point d = Point(img.cols, 0);

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

}

void Lane_Detector::region_of_interest_R(Mat& img, Mat& img_ROI){
	Point a = Point(0, 0);
	Point b = Point(0, img.rows/5);
	Point c = Point(img.cols, img.rows);
	Point d = Point(img.cols, 0);

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
}

float Lane_Detector::get_slope(const Point& p1, const Point& p2){

  float slope;

	if(p2.y - p1.y != 0.0){
		slope = ((float) p2.y - (float) p1.y) / ((float) p2.x - (float) p1.x);
	}
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
  int threshold = 60;

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


#endif
