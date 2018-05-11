#ifndef LANE_DETECTOR_H
#define LANE_DETECTOR_H

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
#include <string.h>
#include <sys/time.h>
using namespace cv;
using namespace std;

const CvScalar COLOR_BLUE = CvScalar(255, 0, 0);
const CvScalar COLOR_RED = CvScalar(0, 0, 255);
const CvScalar COLOR_GREEN = CvScalar(0, 255, 0);

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

struct sLine
{
	double sx, sy;
	double ex, ey;

	sLine() : sx(0), sy(0), ex(0), ey(0) { }
	sLine(int sx_, int sy_, int ex_, int ey_)
		: sx(sx_), sy(sy_), ex(ex_), ey(ey_)
	{

	}

};

class Lane_Detector {
protected:
	float left_slope;
	float right_slope;
	float left_length;
	float right_length;

	VideoCapture capture_left;
	VideoCapture capture_right;
	VideoCapture capture_park;
	VideoWriter output_video;

	bool left_error;
	bool right_error;
	Mat input_left, input_right;

	int left_error_count;
	int right_error_count;

	Mat img_hsv, filterImg1, filterImg2, binaryImg1, binaryImg2, initROI1, initROI2,
		mask, cannyImg1, cannyImg2, houghImg1, houghImg2, park_img;

	int clusterCount;
	sLine cluster[6];
	int cluster_idx[6];
	bool parking_mode_onoff;
	void base_ROI(Mat& img, Mat& img_ROI);
	void v_roi(Mat& img, Mat& img_ROI, const Point& p1, const Point& p2);
	void region_of_interest_L(Mat& img, Mat& img_ROI);
	void region_of_interest_R(Mat& img, Mat& img_ROI);
	bool hough_left(Mat& img, Point* p1, Point* p2);
	bool hough_right(Mat& img, Point* p1, Point* p2);
	float get_slope(const Point& p1, const Point& p2);
	int position(const Point P1, const Point P2);
	void hough_to_cluster();

public:
	Point p1, p2, p3, p4;
	int parking_position;
	int parking_state;
	bool uturn_mode_onoff;
	Point parking_point1, parking_point2, stop_parking;
	int stop_y;
	Mat originImg_left;
	Mat originImg_right;
	Lane_Detector() {}
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
	void parking_init();
	void parking_release();
	void get_crosspoint();
	void stop_line();
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

void Lane_Detector::init() {
	string path = "/home/nvidia/ISCC_Videos/";
	struct tm* datetime;
	time_t t;
	t = time(NULL);
	datetime = localtime(&t);
	string s_t = path.append(to_string(datetime->tm_year + 1900)).append("-").append(to_string(datetime->tm_mon + 1)).append("-").append(to_string(datetime->tm_mday)).append("_").append(to_string(datetime->tm_hour)).append(":").append(to_string(datetime->tm_min)).append(":").append(to_string(datetime->tm_sec)).append(".avi");

	//capture_park = VideoCapture(3);
	capture_left = VideoCapture(2);
	capture_right = VideoCapture(1);
	capture_left.set(CV_CAP_PROP_FRAME_WIDTH, 320);
	capture_left.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
	capture_right.set(CV_CAP_PROP_FRAME_WIDTH, 320);
	capture_right.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
	output_video.open(s_t, VideoWriter::fourcc('X', 'V', 'I', 'D'), 20, Size(1280, 480), true);
	mask = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1, 1));

	left_error = false;
	right_error = false;
	left_length = 0;
	right_length = 0;
	parking_mode_onoff = false;
	left_error_count = 0;
	right_error_count = 0;
}

void Lane_Detector::operate() {
	capture_left >> input_left;
	capture_right >> input_right;

	capture_park >> park_img;

	resize(input_left, originImg_left, Size(640, 480), 0, 0, CV_INTER_LINEAR);
	resize(input_right, originImg_right, Size(640, 480), 0, 0, CV_INTER_LINEAR);
	if (originImg_left.empty()) {
		cerr << "Empty Left Image" << endl;
		return;
	}

	if (originImg_right.empty()) {
		cerr << "Empty right Image" << endl;
		return;
	}

	GaussianBlur(originImg_left, filterImg1, Size(5, 5), 0);
	GaussianBlur(originImg_right, filterImg2, Size(5, 5), 0);

	if (parking_mode_onoff)
	{
		inRange(filterImg1, RGB_WHITE_LOWER, RGB_WHITE_UPPER, binaryImg1);
	}
	else
	{
		cvtColor(filterImg1, img_hsv, COLOR_BGR2HSV);
		inRange(img_hsv, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER, binaryImg1);
	}

	Canny(binaryImg1, cannyImg1, 130, 270);

	inRange(originImg_right, RGB_WHITE_LOWER, RGB_WHITE_UPPER, binaryImg2);
	Canny(binaryImg2, cannyImg2, 130, 270);

	// Mat initROI1;
	// Mat initROI2;

	// region_of_interest_L(originImg_left, initROI1);
	// region_of_interest_R(originImg_right, initROI2);


	if (!left_error) {
		v_roi(cannyImg1, initROI1, p1, p2);
	}
	else if(left_error && left_error_count != 0){
		base_ROI(cannyImg1, initROI1);
	}
	else{
		region_of_interest_L(cannyImg1, initROI1);
		left_error_count++;
	}

	if (!right_error) {
		v_roi(cannyImg2, initROI2, p4, p3);
	}
	else if(right_error && right_error_count != 0){
		base_ROI(cannyImg2, initROI2);
	}
	else {
		region_of_interest_R(cannyImg2, initROI2);
		right_error_count++;
	}

	left_error = hough_left(initROI1, &p1, &p2);
	right_error = hough_right(initROI2, &p3, &p4);

	line(originImg_left, p1, p2, COLOR_RED, 4, CV_AA);
	line(originImg_right, p3, p4, COLOR_RED, 4, CV_AA);

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

	if (parking_mode_onoff) {
		hough_to_cluster();
		get_crosspoint();
	}

#ifdef DEBUG
	imshow("img_hsv", img_hsv);
	imshow("mask", binaryImg1);
	imshow("canny1", cannyImg1);
	imshow("initORI1", initROI1);
	imshow("initORI2", initROI2);
	imshow("result", c);
#endif
	// output_video << c;
	if (waitKey(10) == 0) {
		return;
	}
}

void Lane_Detector::base_ROI(Mat& img, Mat& img_ROI) {

	Point a = Point(0, 40);
	Point b = Point(0, img.rows);
	Point c = Point(img.cols, img.rows);
	Point d = Point(img.cols, 40);

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

void Lane_Detector::v_roi(Mat& img, Mat& img_ROI, const Point& p1, const Point& p2) {


	float slope = get_slope(p1, p2);
	float alphaY = 50.f / sqrt(slope*slope + 1);
	float alphaX = slope * alphaY;

	Point a(p1.x - alphaX, p1.y + alphaY);
	Point b(p1.x + alphaX, p1.y - alphaY);
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


void Lane_Detector::region_of_interest_L(Mat& img, Mat& img_ROI) {
	Point a = Point(0, 40);
	Point b = Point(0, img.rows);
	Point c = Point(img.cols, img.rows / 5);
	Point d = Point(img.cols, 40);

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

void Lane_Detector::region_of_interest_R(Mat& img, Mat& img_ROI) {
	Point a = Point(0, 40);
	Point b = Point(0, img.rows / 5);
	Point c = Point(img.cols, img.rows);
	Point d = Point(img.cols, 40);

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

float Lane_Detector::get_slope(const Point& p1, const Point& p2) {

	float slope;

	if (p2.y - p1.y != 0.0) {
		slope = ((float)p2.y - (float)p1.y) / ((float)p2.x - (float)p1.x);
	}
	return slope;
}

bool Lane_Detector::get_intersectpoint(const Point& AP1, const Point& AP2,
	const Point& BP1, const Point& BP2, Point* IP)
{
	double t;
	double s;
	double under = (BP2.y - BP1.y)*(AP2.x - AP1.x) - (BP2.x - BP1.x)*(AP2.y - AP1.y);
	if (under == 0) return false;

	double _t = (BP2.x - BP1.x)*(AP1.y - BP1.y) - (BP2.y - BP1.y)*(AP1.x - BP1.x);
	double _s = (AP2.x - AP1.x)*(AP1.y - BP1.y) - (AP2.y - AP1.y)*(AP1.x - BP1.x);

	t = _t / under;
	s = _s / under;

	if (t<0.0 || t>1.0 || s<0.0 || s>1.0) return false;
	if (_t == 0 && _s == 0) return false;

	IP->x = AP1.x + t * (double)(AP2.x - AP1.x);
	IP->y = AP1.y + t * (double)(AP2.y - AP1.y);

	return true;
}

bool Lane_Detector::hough_left(Mat& img, Point* p1, Point* p2) {

	vector<Vec2f> linesL;

	int count = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
	int threshold = 80;

	for (int i = 10; i > 0; i--) {
		HoughLines(img, linesL, 1, CV_PI / 180, threshold, 0, 0, 0, CV_PI / 2);
		int clusterCount = 2;
		Mat h_points = Mat(linesL.size(), 1, CV_32FC2);
		Mat labels, centers;
		if (linesL.size() > 1) {
			for (size_t i = 0; i < linesL.size(); i++) {
				count++;
				float rho = linesL[i][0];
				float theta = linesL[i][1];
				double a = cos(theta), b = sin(theta);
				double x0 = a * rho, y0 = b * rho;
				// cout << "x0, y0 : " << rho << ' ' << theta << endl;
				h_points.at<Point2f>(i, 0) = Point2f(rho, (float)(theta * 100));
			}
			kmeans(h_points, clusterCount, labels,
				TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 10, 1.0),
				3, KMEANS_RANDOM_CENTERS, centers);

			Point mypt1 = centers.at<Point2f>(0, 0);

			float rho = mypt1.x;
			float theta = (float)mypt1.y / 100;
			double a = cos(theta), b = sin(theta);
			double x0 = a * rho, y0 = b * rho;

			// cout << "pt : " << mypt1.x << ' ' << mypt1.y << endl;

			int _x1 = int(x0 + 1000 * (-b));
			int _y1 = int(y0 + 1000 * (a));
			int _x2 = int(x0 - 1000 * (-b));
			int _y2 = int(y0 - 1000 * (a));

			x1 += _x1;
			y1 += _y1;

			x2 += _x2;
			y2 += _y2;

			Point mypt2 = centers.at<Point2f>(1, 0);

			rho = mypt2.x;
			theta = (float)mypt2.y / 100;
			a = cos(theta), b = sin(theta);
			x0 = a * rho, y0 = b * rho;

			// cout << "pt : " << mypt2.x << ' ' << mypt2.y << endl;

			_x1 = int(x0 + 1000 * (-b));
			_y1 = int(y0 + 1000 * (a));
			_x2 = int(x0 - 1000 * (-b));
			_y2 = int(y0 - 1000 * (a));

			x1 += _x1;
			y1 += _y1;

			x2 += _x2;
			y2 += _y2;

			break;
		};
	}
	if (count != 0) {
		p1->x = x1 / 2; p1->y = y1 / 2;
		p2->x = x2 / 2; p2->y = y2 / 2;

		left_error_count = 0;
		return false;
	}
	return true;
}

bool Lane_Detector::hough_right(Mat& img, Point* p1, Point* p2) {
	vector<Vec2f> linesR;

	int count = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
	int threshold = 80;

	for (int i = 10; i > 0; i--) {
		HoughLines(img, linesR, 1, CV_PI / 180, threshold, 0, 0, CV_PI / 2, CV_PI);
		int clusterCount = 2;
		Mat h_points = Mat(linesR.size(), 1, CV_32FC2);
		Mat labels, centers;
		if (linesR.size() > 1) {
			for (size_t i = 0; i < linesR.size(); i++) {
				count++;
				float rho = linesR[i][0];
				float theta = linesR[i][1];
				double a = cos(theta), b = sin(theta);
				double x0 = a * rho, y0 = b * rho;
				// cout << "x0, y0 : " << rho << ' ' << theta << endl;
				h_points.at<Point2f>(i, 0) = Point2f(rho, (float)(theta * 100));
			}
			kmeans(h_points, clusterCount, labels,
				TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 10, 1.0),
				3, KMEANS_RANDOM_CENTERS, centers);

			Point mypt1 = centers.at<Point2f>(0, 0);

			float rho = mypt1.x;
			float theta = (float)mypt1.y / 100;
			double a = cos(theta), b = sin(theta);
			double x0 = a * rho, y0 = b * rho;

			// cout << "pt : " << mypt1.x << ' ' << mypt1.y << endl;

			int _x1 = int(x0 + 1000 * (-b));
			int _y1 = int(y0 + 1000 * (a));
			int _x2 = int(x0 - 1000 * (-b));
			int _y2 = int(y0 - 1000 * (a));

			x1 += _x1;
			y1 += _y1;

			x2 += _x2;
			y2 += _y2;

			Point mypt2 = centers.at<Point2f>(1, 0);

			rho = mypt2.x;
			theta = (float)mypt2.y / 100;
			a = cos(theta), b = sin(theta);
			x0 = a * rho, y0 = b * rho;

			// cout << "pt : " << mypt2.x << ' ' << mypt2.y << endl;

			_x1 = int(x0 + 1000 * (-b));
			_y1 = int(y0 + 1000 * (a));
			_x2 = int(x0 - 1000 * (-b));
			_y2 = int(y0 - 1000 * (a));

			x1 += _x1;
			y1 += _y1;

			x2 += _x2;
			y2 += _y2;

			break;
		};
	}
	if (count != 0) {
		p1->x = x1 / 2; p1->y = y1 / 2;
		p2->x = x2 / 2; p2->y = y2 / 2;

		right_error_count = 0;
		return false;
	}
	return true;
}

void Lane_Detector::hough_to_cluster()
{
	Mat park_roi, blur_park, range_park, canny_park;


	if (parking_state == 1) park_roi = input_right;
	else park_roi = park_img(Rect(0, park_img.rows / 2, park_img.cols, park_img.rows / 2));


	GaussianBlur(park_roi, blur_park, Size(5, 5), 0);
	inRange(blur_park, Scalar(180, 100, 100), Scalar(255, 255, 255), range_park);

	Canny(range_park, canny_park, 70, 200);

	clusterCount = 0;
	int h_threshold = 80;
	vector<Vec2f> lines_out_park;
	vector<Vec2f> different_rho;
	memset(cluster_idx, 0, sizeof(cluster_idx));
	for (int i = 0; i < 6; i++)
	{
		cluster[i].sx = 0;
		cluster[i].sy = 0;
		cluster[i].ex = 0;
		cluster[i].ey = 0;
	}

	HoughLines(canny_park, lines_out_park, 1, CV_PI / 180, h_threshold);

	if (lines_out_park.size() >= 1)
	{
		for (size_t i = 0; i < lines_out_park.size(); i++)
		{
			float rho = lines_out_park[i][0];
			float theta = lines_out_park[i][1];
			double a = cos(theta), b = sin(theta);
			double x0 = a * rho, y0 = b * rho;
			int x1_ = int(x0 + 1000 * (-b));
			int y1_ = int(y0 + 1000 * (a));
			int x2_ = int(x0 - 1000 * (-b));
			int y2_ = int(y0 - 1000 * (a));

			int tx = x2_ - x1_;
			int ty = y2_ - y1_;
			double deg = atan2((double)ty, (double)tx) * 180 / CV_PI;
			//cout << "deg, rho = " << deg <<" "<<rho<< endl;


			if (different_rho.empty())
			{

				different_rho.push_back(Vec2f(deg, rho));
				clusterCount++;
				cluster[0].sx += x1_;
				cluster[0].ex += x2_;
				cluster[0].sy += y1_;
				cluster[0].ey += y2_;
				cluster_idx[0]++;

			}
			else
			{
				for (int k = 0; k < different_rho.size(); k++)
				{
					if (abs(different_rho[k][0] - deg) <= 5 && (different_rho[k][1] - rho)<55 || (abs(different_rho[k][0] - deg)>5 && abs(different_rho[k][0] - deg) <= 10) && abs(different_rho[k][1] - rho)<50)// \B5ι\F8° \C1\B6\B0\C7 \BF\F8\B7\A1 35
					{
						cluster[k].sx += x1_;
						cluster[k].ex += x2_;
						cluster[k].sy += y1_;
						cluster[k].ey += y2_;
						cluster_idx[k]++;

						//cout << "rho>=0  sx,sy,ex,ey, idx = " << cluster[k].sx << " " << cluster[k].sy << " " << cluster[k].ex << " " << cluster[k].ey <<" "<<cluster_idx[k]<< endl;

						break;
					}
					if (k == different_rho.size() - 1)
					{
						different_rho.push_back(Vec2f(deg, rho));
						clusterCount++;
						cluster[k + 1].sx += x1_;
						cluster[k + 1].ex += x2_;
						cluster[k + 1].sy += y1_;
						cluster[k + 1].ey += y2_;
						cluster_idx[k + 1]++;
					}
				}
			}
		}

		//cout << "cluster = " << clusterCount << endl;
	}
}



void Lane_Detector::get_crosspoint()
{
	double a, b, c, d;
	double cross_x, cross_y, minus = 100000, plus = 100000, minus_ = -100000;
	int	minus_idx = -1, plus_idx = -1;

	for (int p = 0; p < clusterCount; p++)
	{

		cluster[p].sx = (double)cluster[p].sx / cluster_idx[p];
		cluster[p].sy = (double)cluster[p].sy / cluster_idx[p];
		cluster[p].ex = (double)cluster[p].ex / cluster_idx[p];
		cluster[p].ey = (double)cluster[p].ey / cluster_idx[p];

		//cout << "sx,sy,ex,ey  , idx= " << cluster[p].sx << " " << cluster[p].sy << " " << cluster[p].ex << " " << cluster[p].ey << " " << cluster_idx[p] << endl;

		//cout << "degggg  = " << (double)(cluster[p].ey - cluster[p].sy) / (cluster[p].ex - cluster[p].sx) << endl;

		if ((double)(cluster[p].ey - cluster[p].sy) / (cluster[p].ex - cluster[p].sx) < 0)
		{
			if (parking_state == 0 && cluster[p].sx < minus)
			{
				minus = cluster[p].sx;
				minus_idx = p;
			}
			else if (parking_state == 1 && cluster[p].sx > minus_)
			{
				minus_ = cluster[p].sx;
				minus_idx = p;
			}
		}
		else if ((double)(cluster[p].ey - cluster[p].sy) / (cluster[p].ex - cluster[p].sx) >= 0)
		{
			if (cluster[p].sy < plus)
			{
				plus = cluster[p].sy;
				plus_idx = p;
			}
		}


		line(park_img, Point(cluster[p].sx, cluster[p].sy), Point(cluster[p].ex, cluster[p].ey), Scalar(255, 0, 0), 3, LINE_AA);
	}


	if (clusterCount == 2)
	{
		a = (double)(cluster[0].ey - cluster[0].sy) / (cluster[0].ex - cluster[0].sx);
		b = (double)cluster[0].ey - (a* cluster[0].ex);
		c = (double)(cluster[1].ey - cluster[1].sy) / (cluster[1].ex - cluster[1].sx);
		d = (double)cluster[1].ey - (c* cluster[1].ex);

		cross_x = (double)(d - b) / (a - c);
		cross_y = (double)(a* cross_x) + b;

		//cout << "cross = " << cross_x << " " << cross_y << endl;

		circle(park_img, Point2f(cross_x, cross_y), 5, Scalar(0, 255, 0), 3, 8);
	}

	else if (clusterCount > 2 && minus_idx >= 0 && plus_idx >= 0)
	{
		a = (double)(cluster[minus_idx].ey - cluster[minus_idx].sy) / (cluster[minus_idx].ex - cluster[minus_idx].sx);
		b = (double)cluster[minus_idx].ey - (a* cluster[minus_idx].ex);
		c = (double)(cluster[plus_idx].ey - cluster[plus_idx].sy) / (cluster[plus_idx].ex - cluster[plus_idx].sx);
		d = (double)cluster[plus_idx].ey - (c* cluster[plus_idx].ex);

		cross_x = (double)(d - b) / (a - c);
		cross_y = (double)(a* cross_x) + b;

		//cout << "cross = " << cross_x << " " << cross_y << endl;

		circle(park_img, Point2f(cross_x, cross_y), 5, Scalar(0, 255, 0), 3, 8);
	}

	if (clusterCount >= 2)
	{
		if (parking_state == 1)
		{
			stop_parking.x = cross_x;
			stop_parking.y = cross_y;
		}
		else if (parking_point1.x == 0 && parking_point1.y == 0)
		{
			parking_position = 1;
			parking_point1.x = cross_x;
			parking_point1.y = cross_y;
		}
		else if (parking_position == 1 && abs(parking_point1.x - cross_x) < 10 && abs(parking_point1.y - cross_y)<10)
		{
			parking_point1.x = cross_x;
			parking_point1.y = cross_y;
		}
		else
		{
			if (parking_position == 1) parking_position = 2;
			parking_point2.x = cross_x;
			parking_point2.y = cross_y;

		}


		//if (parking_position == 1) cout << "position , x, y = " << parking_position << " " << parking_point1.x << " " << parking_point1.y << endl;
		//else if (parking_position == 2) cout << "position , x, y = " << parking_position << " " << parking_point2.x << " " << parking_point2.y << endl;
		//-> \C0Ӱ\E8 \B0\AA 63 ~68
	}
}

void Lane_Detector::parking_init() {
	capture_park.open(3);
	parking_mode_onoff = true;
	parking_point1.x = 0;
	parking_point1.y = 0;
	parking_point2.x = 0;
	parking_point2.y = 0;
	stop_parking.x = 0;
	stop_parking.y = 0;
	parking_position = 0;
	parking_state = 0;
	clusterCount = 0;
	memset(cluster_idx, 0, sizeof(cluster_idx));
}

void Lane_Detector::parking_release() {
	capture_park.release();
	parking_mode_onoff = false;
}

void Lane_Detector::stop_line()
{
	Mat canny_stop, range_stop, stop_img, roi_stop;
	int threshold = 80;
	vector<Vec2f> linesL;
	int Lstop_x1, Lstop_x2, Lstop_y1, Lstop_y2 = 0;

	stop_img = input_left;

	if (uturn_mode_onoff)
	{
		GaussianBlur(stop_img, roi_stop, Size(5, 5), 0);

		cvtColor(roi_stop, roi_stop, COLOR_BGR2HSV);
		inRange(img_hsv, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER, range_stop);
		Canny(range_stop, canny_stop, 70, 200);
		HoughLines(canny_stop, linesL, 1, CV_PI / 180, threshold, 0, 0, 0, CV_PI / 2);
	}
	else
	{
		GaussianBlur(stop_img, roi_stop, Size(5, 5), 0);
		inRange(roi_stop, Scalar(180, 100, 100), Scalar(255, 255, 255), range_stop);
		Canny(range_stop, canny_stop, 70, 200);
		HoughLines(canny_stop, linesL, 1, CV_PI / 180, threshold, 0, 0, CV_PI / 2, CV_PI);
	}


	for (int i = 0; i < linesL.size(); i++)
	{
		float rho = linesL[i][0];
		float theta = linesL[i][1];
		double a = cos(theta), b = sin(theta);
		double x0 = a * rho, y0 = b * rho;
		int x1_ = int(x0 + 1000 * (-b));
		int y1_ = int(y0 + 1000 * (a));
		int x2_ = int(x0 - 1000 * (-b));
		int y2_ = int(y0 - 1000 * (a));

		int tx = x2_ - x1_;
		int ty = y2_ - y1_;
		double deg = atan2((double)ty, (double)tx) * 180 / CV_PI;

		if (uturn_mode_onoff)
		{
			if (Lstop_y1 < y1_)
			{
				Lstop_x1 = x1_;
				Lstop_y1 = y1_;
				Lstop_x2 = x2_;
				Lstop_y2 = y2_;

			}
		}
		else
		{
			if (deg < 30)
			{
				if (Lstop_y2 < y2_)
				{
					Lstop_x1 = x1_;
					Lstop_y1 = y1_;
					Lstop_x2 = x2_;
					Lstop_y2 = y2_;

				}
			}
		}
	}

	//if (Lstop_y2 != 0)line(stop_img, Point(Lstop_x1, Lstop_y1), Point(Lstop_x2, Lstop_y2), Scalar(0, 0, 255), 3, LINE_AA);

	if (uturn_mode_onoff) stop_y = Lstop_y1;
	else stop_y = Lstop_y2;


}




#endif
