// LTT.ETS.CameraMarketing.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <ctime>
#include <iostream>
#include <sstream>
#include <fstream>

#include "Tracker.h"
#include "Detector.h"
#include "MultiTrackAssociation.h"
#include "Parameter.h"
#include "DataReader.h"
#include "package_bgs\jmo\MultiLayerBGS.h"

using namespace cv;
using namespace std;

int _tmain(int argc, _TCHAR* argv[])
{
	namedWindow("multiTrack",CV_WINDOW_AUTOSIZE);
	SeqReader* reader = NULL;
	Mat frame;
	reader = new VideoReader(URL);

	if(reader == NULL)
	{
		cout << "Loi khoi tao reader" << endl;
		return EXIT_FAILURE;
	}

	reader->readImg(frame);

	if (frame.data==NULL)
	{
		cerr<<"fail to open pictures!"<<endl;
		return EXIT_FAILURE;
	}

	Detector* detector = NULL;
	detector = new HogDetector();
	if(detector == NULL)
	{
		cerr << "Loi tao detector" << endl;
		return EXIT_FAILURE;
	}

	Enviroment enviroment;
	TrakerManager mTrack(detector,frame,EXPERT_THRESH);

	while(true)
	{
		mTrack.doWork(frame);
		imshow("multiTrack", frame);
		reader->readImg(frame);

		char c = waitKey(1);
		enviroment.drawEnviroment(frame);

		if(cv::waitKey(20) > 0)
			break;
	}

	if(reader != NULL)
		delete reader;

	if(detector != NULL)
		delete detector;

	return EXIT_SUCCESS;
}