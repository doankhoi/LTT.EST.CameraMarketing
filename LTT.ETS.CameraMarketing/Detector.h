#pragma once
#include <opencv2\opencv.hpp>
#include <opencv2\gpu\gpu.hpp>

#include "Util.h"
#include "Parameter.h"

#define HOG 1
#define XML 2

#include "package_bgs\jmo\MultiLayerBGS.h"

class Detector
{
public:
	Detector(int t):type(t){}
	~Detector(void);
	virtual void detect(const cv::Mat& frame) = 0;
	vector<Rect> getDetection(){ return detection;}
	vector<double> getResponse(){ return response;}
	void draw(cv::Mat& frame);
protected:
	vector<Rect> detection;
	vector<double> response;
	int type;
};


class HogDetector:public Detector
{
public:
	HogDetector();
	~HogDetector();
	virtual void detect(const Mat& frame);

private:
	MultiLayerBGS *mbgs;
	vector<float> detector;
	vector<float> repsonse;//classifier response
};
