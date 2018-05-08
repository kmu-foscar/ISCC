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
#include "Lane_Detector.hpp"

using namespace cv;
using namespace std;

class Look_Ahead : public Lane_Detector{
private :
  Mat croppedImg1, croppedImg2;
  bool hough_left(Mat& img, Point* p1, Point* p2);
  bool hough_right(Mat& img, Point* p1, Point* p2);
  void v_roi(Mat& img, Mat& img_ROI, const Point& p1, const Point& p2);
  void region_of_interest_L(Mat& img, Mat& img_ROI);
  void region_of_interest_R(Mat& img, Mat& img_ROI);

public :
  Look_Ahead();
  void operate(Mat originImg_left, Mat originImg_right);
};
Look_Ahead::Look_Ahead() {
  mask = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1, 1));
  left_error = false;
  right_error = false;
  left_length = 0;
  right_length = 0;
  output_video.open("lookahead.avi", VideoWriter::fourcc('X', 'V', 'I', 'D'), 20, Size(1280, 480), true);
}
void Look_Ahead::operate(Mat originImg_left_LD, Mat originImg_right_LD) {
  originImg_left_LD.copyTo(originImg_left);
  originImg_right_LD.copyTo(originImg_right);
  croppedImg1 = originImg_left(Rect(200, 0, 440, 40));
  croppedImg2 = originImg_right(Rect(0, 0, 440, 40));
  GaussianBlur(croppedImg1, filterImg1, Size(3, 3), 0);
	GaussianBlur(croppedImg2, filterImg2, Size(3, 3), 0);

	cvtColor(filterImg1, img_hsv, COLOR_BGR2HSV);

	inRange(img_hsv, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER, binaryImg1);
	inRange(croppedImg2, RGB_WHITE_LOWER, RGB_WHITE_UPPER, binaryImg2);

	Canny(binaryImg1, cannyImg1, (binaryImg1.rows + binaryImg1.cols)/4, (binaryImg1.rows + binaryImg1.cols)/2);
	Canny(binaryImg2, cannyImg2, (binaryImg2.rows + binaryImg2.cols)/4, (binaryImg2.rows + binaryImg2.cols)/2);

	if(!left_error){
		v_roi(cannyImg1, initROI1, p1, p2);
	}
	else{
		region_of_interest_L(cannyImg1, initROI1);
	}

	if(!right_error){
    v_roi(cannyImg2, initROI2, p4, p3);
	}
	else{
		region_of_interest_R(cannyImg2, initROI2);
	}

	left_error = hough_left(initROI1, &p1, &p2);
	right_error = hough_right(initROI2, &p3, &p4);
  Point p1_copy = p1;
  p1_copy.x += 200;
  Point p2_copy = p2;
  p2_copy.x += 200;
	line(originImg_left, p1_copy, p2_copy, COLOR_BLUE, 4, CV_AA);
	line(originImg_right, p3, p4, COLOR_BLUE, 4, CV_AA);

	//cout << p1.x << " " << p1.y << " " << p2.x << " " << p2.y << endl;

	left_slope = get_slope(p1, p2);
	right_slope = get_slope(p3, p4);
	position(p1, p3);

	Mat a;
	Mat b;
	Mat c;

	resize(originImg_left, a, Size(640, 480), 0, 0, CV_INTER_LINEAR);
	resize(originImg_right, b, Size(640, 480), 0, 0, CV_INTER_LINEAR);
	hconcat(a, b, c);
#ifdef DEBUG	
	imshow("canny_la1", cannyImg1);
	imshow("canny_la2", cannyImg2);
	imshow("binaryImg1_la", binaryImg1);
	imshow("binaryImg2_la", binaryImg2);
	imshow("lookahead", c);
#endif

	//output_video << c;
	if(waitKey(10) == 0){
		return;
	}
}
bool Look_Ahead::hough_left(Mat& img, Point* p1, Point* p2){

  vector<Vec2f> linesL;

  int count = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  int threshold = 20;

  for (int i = 10; i > 0; i--){
    HoughLines(img, linesL, 1, CV_PI / 180, threshold, 0, 0, 0, CV_PI /2);
    int clusterCount = 2;
    Mat h_points = Mat(linesL.size(), 1, CV_32FC2);
    Mat labels, centers;
    if (linesL.size() > 1){
      for (size_t i = 0; i < linesL.size(); i++){
        count ++;
        float rho = linesL[i][0];
        float theta = linesL[i][1];
        double a = cos(theta), b = sin(theta);
        double x0 = a * rho, y0 = b * rho;
        h_points.at<Point2f>(i, 0) = Point2f(rho, (float)(theta*100));
      }
      kmeans(h_points, clusterCount, labels,
            TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 10, 1.0),
               3, KMEANS_RANDOM_CENTERS, centers);

      Point mypt1 = centers.at<Point2f>(0,0);

      float rho = mypt1.x;
      float theta = (float)mypt1.y/100;
      double a = cos(theta), b = sin(theta);
      double x0 = a * rho, y0 = b * rho;


        int _x1 = int(x0 + 1000*(-b));
        int _y1 = int(y0 + 1000*(a));
        int _x2 = int(x0 - 1000*(-b));
        int _y2 = int(y0 - 1000*(a));

        x1 += _x1;
        y1 += _y1;

        x2 += _x2;
        y2 += _y2;

        Point mypt2 = centers.at<Point2f>(1,0);

      rho = mypt2.x;
      theta = (float)mypt2.y/100;
      a = cos(theta), b = sin(theta);
      x0 = a * rho, y0 = b * rho;

      // cout << "pt : " << mypt2.x << ' ' << mypt2.y << endl;

        _x1 = int(x0 + 1000*(-b));
        _y1 = int(y0 + 1000*(a));
        _x2 = int(x0 - 1000*(-b));
        _y2 = int(y0 - 1000*(a));

        x1 += _x1;
        y1 += _y1;

        x2 += _x2;
        y2 += _y2;

      break;
    };
  }
  if (count != 0){
    p1->x = x1/2; p1->y = y1/2;
    p2->x = x2/2; p2->y = y2/2;

    return false;
  }
  return true;
}

bool Look_Ahead::hough_right(Mat& img, Point* p1, Point* p2){
  vector<Vec2f> linesR;

  int count = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  int threshold = 40;

  for (int i = 10; i > 0; i--){
    HoughLines(img, linesR, 1, CV_PI / 180, threshold, 0, 0, CV_PI/2, CV_PI);
    int clusterCount = 2;
    Mat h_points = Mat(linesR.size(), 1, CV_32FC2);
    Mat labels, centers;
    if (linesR.size() > 1){
      for (size_t i = 0; i < linesR.size(); i++){
        count ++;
        float rho = linesR[i][0];
        float theta = linesR[i][1];
        double a = cos(theta), b = sin(theta);
        double x0 = a * rho, y0 = b * rho;
        // cout << "x0, y0 : " << rho << ' ' << theta << endl;
        h_points.at<Point2f>(i, 0) = Point2f(rho, (float)(theta*100));
      }
      kmeans(h_points, clusterCount, labels,
            TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 10, 1.0),
               3, KMEANS_RANDOM_CENTERS, centers);

      Point mypt1 = centers.at<Point2f>(0,0);

      float rho = mypt1.x;
      float theta = (float)mypt1.y/100;
      double a = cos(theta), b = sin(theta);
      double x0 = a * rho, y0 = b * rho;

      // cout << "pt : " << mypt1.x << ' ' << mypt1.y << endl;

        int _x1 = int(x0 + 1000*(-b));
        int _y1 = int(y0 + 1000*(a));
        int _x2 = int(x0 - 1000*(-b));
        int _y2 = int(y0 - 1000*(a));

        x1 += _x1;
        y1 += _y1;

        x2 += _x2;
        y2 += _y2;

        Point mypt2 = centers.at<Point2f>(1,0);

      rho = mypt2.x;
      theta = (float)mypt2.y/100;
      a = cos(theta), b = sin(theta);
      x0 = a * rho, y0 = b * rho;

      // cout << "pt : " << mypt2.x << ' ' << mypt2.y << endl;

        _x1 = int(x0 + 1000*(-b));
        _y1 = int(y0 + 1000*(a));
        _x2 = int(x0 - 1000*(-b));
        _y2 = int(y0 - 1000*(a));

        x1 += _x1;
        y1 += _y1;

        x2 += _x2;
        y2 += _y2;

      break;
    };
  }
  if (count != 0){
    p1->x = x1/2; p1->y = y1/2;
    p2->x = x2/2; p2->y = y2/2;

    return false;
  }
  return true;
}
void Look_Ahead::v_roi(Mat& img, Mat& img_ROI, const Point& p1, const Point& p2){


  float slope = get_slope(p1, p2);
  float alphaY = 80.f / sqrt(slope*slope + 1);
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

void Look_Ahead::region_of_interest_L(Mat& img, Mat& img_ROI){
  Point a = Point(0, 0);
  Point b = Point(0, img.rows);
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

}

void Look_Ahead::region_of_interest_R(Mat& img, Mat& img_ROI){
	Point a = Point(0, 0);
	Point b = Point(0, img.rows);
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


#endif
