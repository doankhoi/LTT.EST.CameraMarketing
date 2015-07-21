#pragma once
#include <list>
#include <opencv2\opencv.hpp>
#include "Util.h"
#include "Parameter.h"

#define BIN_NUMBER 32

//Kích thước histogram
static int hSize[] = {BIN_NUMBER, BIN_NUMBER};

class AppTemplate
{
public:
	AppTemplate(const AppTemplate& tracker);
	AppTemplate(const cv::Mat* frame_set, const Rect iniWin, int ID);
	~AppTemplate(void);

	void calcBP(const cv::Mat* frame_set, cv::Mat& occ_map, Rect ROI);

	void calcScore(Rect b_inner, Rect b_outer); //Tính điểm cho template
	cv::Mat& getConfidenceMap(){ return confidence_map;}
	Point2f getShiftVector(){return shift_vector;}
	double getScore(){return score;}
	int getID(){return ID;}
private:
	const int ID;
	int channels[2]; //Hai channels tốt nhất
	cv::Mat hist; //Histagram cho 2 channels

	float _hRang[2][2];
	const float* hRang[2];

	cv::Mat confidence_map; //Bản đồ sự tin cậy
	Point2f shift_vector; //Đánh vận tốc của template
	double score; //Điểm đánh giá template
};

