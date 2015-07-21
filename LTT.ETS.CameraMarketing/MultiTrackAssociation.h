#pragma once
#include <fstream>
#include <opencv2\opencv.hpp>
#include "Parameter.h"
#include "Util.h"
#include "Tracker.h"
#include "Detector.h"
#include "ConnectDB.h"
#include "Enviroment.h"

#include <Windows.h>
#include <direct.h>
#include <CommCtrl.h>
#include <shellapi.h>
#include <stdlib.h>
#include <cstdlib>
#include <stdio.h>

#define GOOD 0
#define NOTSURE 1
#define BAD 2

#define COUNT_NUM 1000.0
#define SLIDING_WIN_SIZE 7.2*TIME_WINDOW_SIZE 

#define DATABASE_PATH  "D:/IpCam/Database/ETSCameraClientCache.db3"

using namespace cv;
using namespace std;

class WaitingList
{
	typedef struct Waiting
	{
		int accu;
		Rect currentWin;
		Point center;
		int life_count;
		Waiting(Rect win)
			:accu(1),
			life_count(1),
			currentWin(win),
			center((int)(win.x+0.5*win.width),(int)(win.y+0.5*win.height))
		{
		}
	}Waiting;

	list<Waiting> w_list;
	int life_limit;

public:
	WaitingList(int life):life_limit(life){}
	void update();
	vector<Rect>outputQualified(double thresh);
	void feed(Rect bodysize_win,double response);
};


class Controller
{
public:
	WaitingList waitList;
	WaitingList waitList_suspicious;

	Controller(
		Size sz,int r, int c,double vh=0.01,
		double lr=1/COUNT_NUM,
		double thresh_expert=0.5);

	void takeVoteForHeight(Rect bodysize_win);	
	vector<int> filterDetection(vector<Rect> detction_bodysize);	
	void takeVoteForAvgHittingRate(list<EnsembleTracker*> _tracker_list);	

	void deleteObsoleteTracker(list<EnsembleTracker*>& _tracker_list);	
	
	void calcSuspiciousArea(list<EnsembleTracker*>& _tracker_list);	

	vector<Rect> getQualifiedCandidates()
	{
		/*
		Điều kiện sinh ứng cử viên
		*/
		double l=_hit_record._getAvgHittingRate(_alpha_hitting_rate,_beta_hitting_rate);
		return waitList.outputQualified((l-sqrt(l)-1.0));		
	}

private:
	typedef struct HittingRecord
	{
		Mat record;
		int idx;
		HittingRecord():idx(0)
		{
			record=Mat::zeros(2,(int)(SLIDING_WIN_SIZE),CV_64FC1);
		}

		void recordVote(bool vote)
		{
			idx=idx-record.cols*(idx/record.cols);
			record.at<double>(0,idx)=vote ? 1.0:0;
			record.at<double>(1,idx)=1;
			idx++;
		}

		double _getAvgHittingRate(double _alpha_hitting_rate, double _beta_hitting_rate)
		{
			Scalar s1=sum(record.row(0));
			Scalar s2=sum(record.row(1));
			return (s1[0]*TIME_WINDOW_SIZE+_alpha_hitting_rate)/(_beta_hitting_rate+s2[0]);
		}

	}HittingRecord;

	double _thresh_for_expert;
	
	Size _frame_size;
	int _grid_rows;
	int _grid_cols;
	double _prior_height_variance;
	vector<vector<double> > _bodyheight_map;
	vector<vector<double> > _bodyheight_map_count;
	double _bodyheight_learning_rate;

	HittingRecord _hit_record;
	double _alpha_hitting_rate;
	double _beta_hitting_rate;

	vector<Rect> _suspicious_rect_list;
};


class TrakerManager
{
public:
	TrakerManager(Detector* detctor,Mat& frame, double thresh_promotion);
	~TrakerManager();	
	void doWork(Mat& frame);

	void restart();
	string getexepath();
	void callPro();
	int getTime();

	void setKey(char c)
	{
		_my_char = c;
	}	

private:	
	void doHungarianAlg(const vector<Rect>& detections);
	static bool compareTraGroup(EnsembleTracker* c1,EnsembleTracker* c2)
	{
		return (c1->getTemplateNum()>c2->getTemplateNum()) ? true:false;
	}

	Controller _controller;
	Mat* _frame_set;
	//Danh sách theo vết
	list<EnsembleTracker*> _tracker_list;
	int _tracker_count;
	char _my_char;		
	
	Detector* _detector;
	int _frame_count;
	
	Mat _occupancy_map;	
	double _thresh_for_expert_;

	//>>>Kết nối cơ sở dữ liệu
	sqlite3* db;
	ConnectDB connectDb;
	string SHOP_CD;
	string dateCurrDb;
	string getCurrentDate();
	//<<<Kết nối cơ sở dữ liệu
};
	
