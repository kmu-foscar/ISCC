#ifndef LOOK_AHEAD_H
#define LOOK_AHEAD_H

#include <iostream>
#include <fstream>
#include <ctime>
#include <queue>
#include <cv.h>
#include <unistd.h>
#include <highgui.h>
#include "opencv2/opencv.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"


using namespace cv;
using namespace std;

const CvScalar COLOR_BLUE = CvScalar(255, 0, 0);
const CvScalar COLOR_RED = CvScalar(0, 0, 255);

class Look_Ahead {
private :
    float left_slope;
    float right_slope;
    float left_length;
    float right_length;
    VideoCapture capture;
    Mat originImg, originImg_left, originImg_right;
    bool left_error;
    bool right_error;
    Mat grayImg, filterImg1, filterImg2, otsu, sobelX_Img, sobelY_Img, sobel_Img1, sobel_Img2,
        imageROI1, imageROI2, bluredImg, blured1, blured2, mask, openingImg1, openingImg2,
        cannyImg1, cannyImg2, houghImg1, houghImg2;
    VideoWriter outputVideo;

    void v_roi(Mat& img, Mat& img_ROI, const Point& p1, const Point& p2);
    void region_of_interest_L(Mat& img, Mat& img_ROI);
    void region_of_interest_R(Mat& img, Mat& img_ROI);
    bool hough_left(Mat& img, Point* p1, Point* p2);
    bool hough_right(Mat& img, Point* p1, Point* p2);
    float get_slope(const Point& p1, const Point& p2);
    int position(const Point P1, const Point P2);
public :
    Point p1, p2, p3, p4;
    Look_Ahead(){}
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

bool Look_Ahead::is_left_error() {
  return left_error;
}
bool Look_Ahead::is_right_error() {
  return right_error;
}
int Look_Ahead::position(const Point P1, const Point P3) {
    float x_L;
    float x_R;
    const float y = 480;

    x_L = (y - P1.y + left_slope * P1.x) / left_slope;
    left_length = 640 - x_L;

    x_R = (y - P3.y + right_slope * P3.x) / right_slope;
    right_length = x_R;
}

float Look_Ahead::get_left_length() {
    return left_length;
}

float Look_Ahead::get_right_length() {
    return right_length;
}

float Look_Ahead::get_left_slope() {
    return left_slope;
}
float Look_Ahead::get_right_slope() {
    return right_slope;
}

void Look_Ahead::init(){
  capture = VideoCapture("/home/hwancheol/Downloads/ISCC_sample_.avi");
  outputVideo.open("ouput.avi", VideoWriter::fourcc('X', 'V', 'I', 'D'), 30, Size(640, 60), true);

  mask = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1, 1));

  left_error = false;
  right_error = false;
  left_length = 0;
  right_length = 0;
}

void Look_Ahead::operate(){
    capture >> originImg;

    if (originImg.empty()){
      cerr << "Empty Image" << endl;
      return;
    }
    //imshow("original", originImg);
    originImg_left = originImg(Rect(320, 0, 320, 60));
    originImg_right = originImg(Rect(640, 0, 320, 60));
    Mat Img_copy_L = originImg_left;
		Mat Img_copy_R = originImg_right;
    Mat imgHSV1, imgThresholded1;
    cvtColor(Img_copy_L, imgHSV1, CV_BGR2HSV);
    inRange(imgHSV1, Scalar(10, 20, 50), Scalar(30, 200, 255), imgThresholded1); 
    bitwise_and(Img_copy_L, Img_copy_L, mask = imgThresholded1);
		GaussianBlur(Img_copy_L, filterImg1, Size(3, 3), 0);
		GaussianBlur(Img_copy_R, filterImg2, Size(5, 5), 0);
    Canny(filterImg1, cannyImg1, (filterImg1.rows + filterImg1.cols) / 4, (filterImg1.rows + filterImg1.cols) / 2) ;
    Canny(filterImg2, cannyImg2, (filterImg2.rows + filterImg2.cols) / 4, (filterImg2.rows + filterImg2.cols) / 2) ;
		imshow("canny1", cannyImg1);
		imshow("canny2", cannyImg2);

    if(!left_error){
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

    left_error = hough_left(cannyImg1, &p1, &p2);
    right_error = hough_right(cannyImg2, &p3, &p4);

    line(Img_copy_L, p1, p2, COLOR_RED, 4, CV_AA);
    line(Img_copy_R, p3, p4, COLOR_RED, 4, CV_AA);

    cout << p1.x << " " << p1.y << " " << p2.x << " " << p2.y << endl;

    left_slope = get_slope(p1, p2);
    right_slope = get_slope(p3, p4);
    position(p1, p3);
		Mat a;
		Mat b;
		Mat c;
		resize(Img_copy_L, a, Size(320, 60), 0, 0, CV_INTER_LINEAR);
		resize(Img_copy_R, b, Size(320, 60), 0, 0, CV_INTER_LINEAR);
		hconcat(a, b, c);
    outputVideo << c;
    imshow("result", c);
    if(waitKey(10) == 0){
      return;
    }
}

void Look_Ahead::v_roi(Mat& img, Mat& img_ROI, const Point& p1, const Point& p2){


  float slope = get_slope(p1, p2);
  float alphaY = 50.f / sqrt(slope*slope + 1);
  float alphaX = slope * alphaY;

  Point a(p1.x - alphaX, p1.y + alphaY );
  Point b(p1.x + alphaX, p1.y - alphaY );
  Point c(p2.x + alphaX, p2.y - alphaY );
  Point d(p2.x - alphaX, p2.y + alphaY );

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


void Look_Ahead::region_of_interest_L(Mat& img, Mat& img_ROI){
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

void Look_Ahead::region_of_interest_R(Mat& img, Mat& img_ROI){
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

float Look_Ahead::get_slope(const Point& p1, const Point& p2){

  float slope;

  slope = ((float) p2.y - (float) p1.y) / ((float) p2.x - (float) p1.x);

  return slope;
}

bool Look_Ahead::get_intersectpoint(const Point& AP1, const Point& AP2,
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

bool Look_Ahead::hough_left(Mat& img, Point* p1, Point* p2){

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

bool Look_Ahead::hough_right(Mat& img, Point* p1, Point* p2){
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
