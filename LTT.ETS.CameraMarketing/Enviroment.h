#pragma once
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <iostream>
#include <vector>
#include "Parameter.h"

#define PATH_DATABASE "D:/Data/IpCam/database/ETSCameraClientCache.db3"

using namespace std;
using namespace cv;

class Enviroment
{
public:
	Enviroment();
	~Enviroment(void);
	void drawEnviroment(cv::Mat& frame);
	void resizeFrame(cv::Mat& frame);

	//>>>Bốn điểm của vùng cửa hàng
	static cv::Point TOP_LEFT;
	static cv::Point TOP_RIGHT;
	static cv::Point BOTTOM_LEFT;
	static cv::Point BOTTOM_RIGHT;
	static std::vector<cv::Point> POLY_POINT_OF_ROI;
	//<<<Bốn điểm của vùng cửa hàng

	//>>> Vùng ảnh có thể gây nhiều lỗi
	static cv::Point EVA_LEFT;
	static cv::Point EVA_RIGHT;
	//<<<Vùng ảnh có thể gây nhiều lỗi

	//>>>Vùng cửa hàng
	static cv::Rect ROI;
	//<<<Vùng cửa hàng

	bool static isIn(Point2d center);
};

