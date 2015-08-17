#include "stdafx.h"
#include "Enviroment.h"

Enviroment::Enviroment()
{
	//Giới hạn khung hình
	POLY_POINT_OF_ROI.clear();

	float SCALE_HEIGHT = 0.65;
	float SCALE_WIDTH = 0.2;

	//Bốn điểm danh giới
	BOTTOM_LEFT = cv::Point( static_cast<int>(FRAME_WIDTH* SCALE_WIDTH) - 25, static_cast<int> (FRAME_HEIGHT * SCALE_HEIGHT) + 10);

	BOTTOM_RIGHT = cv::Point(FRAME_WIDTH - static_cast<int>(static_cast<int>(FRAME_WIDTH* SCALE_WIDTH)), static_cast<int> (FRAME_HEIGHT * SCALE_HEIGHT) + 10);

	TOP_LEFT = cv::Point(static_cast<int>(FRAME_WIDTH* SCALE_WIDTH) - 25,static_cast<int>(FRAME_HEIGHT * 0.05));
	TOP_RIGHT = cv::Point(FRAME_WIDTH - static_cast<int>(static_cast<int>(FRAME_WIDTH* SCALE_WIDTH)), static_cast<int>(FRAME_HEIGHT * 0.05));

	POLY_POINT_OF_ROI.push_back(BOTTOM_LEFT);
	POLY_POINT_OF_ROI.push_back(BOTTOM_RIGHT);
	POLY_POINT_OF_ROI.push_back(TOP_RIGHT);
	POLY_POINT_OF_ROI.push_back(TOP_LEFT);

	//Vùng ảnh theo vết trong cửa hàng
	ROI = cv::Rect(TOP_LEFT.x, TOP_LEFT.y, TOP_RIGHT.x - TOP_LEFT.x, BOTTOM_LEFT.y - TOP_LEFT.y);

	//Vùng ảnh đánh giá thời gian
	EVA_LEFT = cv::Point(static_cast<int>(BOTTOM_LEFT.x), static_cast<int>(BOTTOM_LEFT.y - 30));
	EVA_RIGHT = cv::Point(static_cast<int>(BOTTOM_RIGHT.x), static_cast<int>(BOTTOM_RIGHT.y- 30));
}


Enviroment::~Enviroment(void)
{
}

void Enviroment::resizeFrame(cv::Mat& frame)
{
	cv::resize(frame, frame, cv::Size(FRAME_WIDTH, FRAME_HEIGHT));
}

void Enviroment::drawEnviroment(cv::Mat& frame)
{
	
	int size = POLY_POINT_OF_ROI.size();
	for(int i=1; i< size; i++){
		cv::line(frame, POLY_POINT_OF_ROI[i-1], POLY_POINT_OF_ROI[i], cv::Scalar(255, 0, 0), 2);
	}

	cv::line(frame, POLY_POINT_OF_ROI[0], POLY_POINT_OF_ROI[size-1], cv::Scalar(255, 0, 0), 2);
	//cv::line(frame, EVA_LEFT, EVA_RIGHT, cv::Scalar(255, 0, 0), 2);
}

bool Enviroment::isIn(Point2d center){
	double x = center.x;
	double y = center.y;

	if((x >= TOP_LEFT.x)&&(x <= TOP_RIGHT.x)&&(y >= TOP_LEFT.y)&&(y <= BOTTOM_LEFT.y))
	{
		return true;
	}

	return false;
}

