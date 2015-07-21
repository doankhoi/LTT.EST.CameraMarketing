#pragma once
#include <iostream>
#include <cstdio>
#include "Util.h"
#include "Parameter.h"
#include "Enviroment.h"
#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

#define VIDEO 0
#define IMAGE 1

typedef struct Result2D
{
	int id;
	float xc, yc;
	float w, h;
	double response;
	Result2D(int i, float x_, float y_, float w_, float h_, double res=1): id(i), xc(x_), yc(y_), w(w_), h(h_), response(res){}
	Result2D(){}
} Result2D;

class SeqReader
{
public:
	SeqReader(){};
	virtual void readImg(cv::Mat& frame)=0;
};


class VideoReader:public SeqReader
{
public:
	VideoReader(const string filename)
	{
		capture.open(filename);
		if(!capture.isOpened())
		{
			cout << "Loi khoi tao capture" << endl;
			exit(-1);
		}
	}

	virtual void readImg(Mat& frame)
	{	
		capture >> frame;
		enviroment.resizeFrame(frame);
	}

private:
	VideoCapture capture;
	Enviroment enviroment;
};

#define  ENCODING "UTF-8"