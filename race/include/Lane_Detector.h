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
#include <ros/ros.h>
#include <race/drive_values.h>
#include <race/control_variables.h>

#define P_SLOPE 2.0f
#define P_POSITON -2.0f

class Lane_Detector {
private :
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
    Point p1, p2, p3, p4;
    Lane_Detector(){}
    void init();
    void operate();
    void set_control_variables(float p_slope, float p_position);
    float get_left_slope();
    float get_right_slope();
    float get_left_length();
    float get_right_length();
};

#endif