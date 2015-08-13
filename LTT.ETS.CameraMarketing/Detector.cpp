#include "stdafx.h"
#include "Detector.h"
#define MIN_AREA_PERSON 125

Detector::~Detector(void)
{
}

void Detector::draw(Mat& frame)
{
	for (size_t i = 0;i < detection.size(); ++i)
	{
		rectangle(frame,detection[i],Scalar((3*i)%256,(57*i)%256,(301*i)%256));
	}
}

HogDetector::HogDetector():Detector(HOG)
{
	//Khoi tao tru nen
	mbgs = NULL;
	mbgs = new MultiLayerBGS();
	if(mbgs == NULL)
	{
		cout << "Loi khoi tao MultiLayer" << endl;
		return;
	}
}


void HogDetector::detect(const Mat& frame)
{
	assert(!frame.empty());
	//>>>su dung tru nen
	if(mbgs == NULL)
	{
		std::cout << "chua khoi tao multilayerbgs" << endl;
		return;
	}
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	cv::Mat fore, back;
	mbgs->process(frame, fore, back);

	//Segmentation
	//cv::Mat dist;
	//cv::distanceTransform(fore, dist, CV_DIST_L2, 3);
	//cv::normalize(dist, dist, 0, 1.0, cv::NORM_MINMAX);
	//cv::threshold(dist, dist, 0.5, 1.0, CV_THRESH_BINARY);
	//cv::Mat dist_8u;
	//double minVal, maxVal;
	//cv::minMaxLoc(dist, &minVal, &maxVal);
	//dist.convertTo(dist_8u, CV_8U, 255.0/(maxVal - minVal), -minVal*255.0/(maxVal-minVal));
	//cv::findContours(dist_8u, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	////>>>Đánh dấu các đối tượng
	//cv::Mat markers = cv::Mat::zeros(dist.size(), CV_8U);
	//for(int i = 0; i < contours.size(); i++)
	//{
	//	cv::drawContours(markers, contours, i , Scalar::all(i+20), -1);
	//}
	////<<<Đánh dấu các đối tượng

	////>>>Đánh dấu background
	//cv::circle(markers, Point(5, 5), 3, Scalar(255, 255, 255), -1);
	////<<<Đánh dấu background

	//>>>Segmentation foreground
	cv::Mat fg;
	cv::erode(fore, fg, cv::Mat(), Point(-1,-1), 3);
	//<<<Segmentation foreground

	//>>>Segmentation background
	cv::Mat bg;
	cv::dilate(fore, bg, cv::Mat(), Point(-1, -1), 3);
	cv::threshold(bg, bg, 1, 128, CV_THRESH_BINARY_INV);
	//<<<Segmentation background

	cv::Mat markers(fore.size(), CV_8U, cv::Scalar(0));
	markers = bg + fg;

	segmenter.setMarkers(markers);
	result = segmenter.process(frame);
	cv::imshow("result", result);
	cv::Canny(result, result_egde, 30, 30*3, 3);
	findContours(result_egde, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point());

	this->detection.clear();
	if( contours.size() > 0)
	{
		for( int i = 0; i < contours.size(); i++ )
		{
			Rect r=cv::boundingRect(contours[i]);
			double area = contourArea(contours[i]);
			//nếu đối tượng quá nhỏ bỏ qua
			if((area > MIN_AREA_PERSON) && (area > (r.width*r.height) / 4))
			{
				this->detection.push_back(r);
			}
		}
	}
	//<<<Su dung tru nen

	//Correct lai kich thuoc
	for (vector<Rect>::iterator it = detection.begin(); it < detection.end(); it++)
	{
		it->x=(int)(it->x/HOG_DETECT_FRAME_RATIO);
		it->y=(int)(it->y/HOG_DETECT_FRAME_RATIO);
		it->width=(int)(it->width/HOG_DETECT_FRAME_RATIO);
		it->height=(int)(it->height/HOG_DETECT_FRAME_RATIO);
	}
}

HogDetector::~HogDetector()
{
	if(mbgs != NULL)
	{
		delete mbgs;
	}
}