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

	cv::Mat fore, back;
	mbgs->process(frame, fore, back);

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Mat element = getStructuringElement(MORPH_RECT, Size(5,5), Point(-1 , -1));
	cv::dilate(fore, fore, element); 
	cv::erode(fore, fore, element);
	imshow("foreground", fore); 

	findContours(fore, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point());

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
	//cpu_hog.detectMultiScale(frame, detection, response, 0.0, Size(8,8),Size(0, 0), 1.05, 2);

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