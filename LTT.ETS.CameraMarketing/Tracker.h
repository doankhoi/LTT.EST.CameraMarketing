#pragma once
#include <list>
#include <opencv2\opencv.hpp>
#include "AppTemplate.h"
#include "Parameter.h"
#include "Util.h"
#include "Enviroment.h"
#include <time.h>

using namespace cv;
using namespace std;

class EnsembleTracker
{

public:
	EnsembleTracker(unsigned int id, Size body_size, double phi1=0.5, double phi2=1.5, double phi_max=4.0);		
	~EnsembleTracker();	

	//Biến đếm
	void refcAdd1(){	++_refc;	}
	void refcDec1(){--_refc; _refc = MAX(0,_refc);}
	size_t getRefc(){return _refc;}

	// Quản lý bộ nhớ
	bool getIsDumped(){return _is_dumped;}
	void dump();
	static void emptyTrash();

	// Cập nhật các track lân cận
	void updateNeighbors(
		list<EnsembleTracker*> tr_list,
		double dis_thresh_r=2.5,
		double scale_r1=1.2, double scale_r2=0.8,
		double hist_thresh=0.5);

	void addAppTemplate(const Mat* frame_set,Rect iniWin);
	void track(const Mat* frame_set,Mat& occ_map);

	//Tính toán bản đồ các track với Kalman filter
	void calcConfidenceMap(const Mat* frame_set, Mat& occ_map);
	//Tính điểm cho template
	void calcScore();
	void deletePoorTemplate(double threshold); //Xóa các template dưới ngưỡng
	void deletePoorestTemplate();		
	void demote();
	void promote();
	void registerTrackResult();

	void updateMatchHist(Mat& frame);
	double compareHisto(Mat& frame, Rect win);

	//Tính toán vận tốc di chuyển
	double getVel()
	{
		return (abs(_kf.statePost.at<float>(2,0)) + abs(_kf.statePost.at<float>(3,0)));//*FRAME_RATE
	}

	void setAddNew(bool b){_added_new=b;}
	bool getAddNew(){return _added_new;}

	double getHitFreq()
	{
		Scalar s = sum(_recentHitRecord.row(0));
		return s[0]/MIN((double)_recentHitRecord.cols,(double)_result_history.size());		
	}

	double getHitMeanScore()
	{
		return mean(_recentHitRecord.row(1))[0];
	}

	void drawFilterWin(Mat& frame)
	{
		//Rect win=_filter_result_history.back();
		//rectangle(frame,win,COLOR(_ID),1);
	}

	void drawResult(Mat& frame,double scale)
	{
		scale-=1;
		Rect win = _result_history.back();//_result_history.back();
		Point center((int)(win.x + 0.5*win.width), (int)(win.y + 0.5*win.height));
		if(enviroment.isIn(center))
			cv::circle(frame, center, 3 ,Scalar( 123, 255, 0), -1);
	}

	void drawAssRadius(Mat& frame)
	{
		Rect win=_result_temp;
		circle(frame,Point((int)(win.x+0.5*win.width),(int)(win.y+0.5*win.height)),(int)MAX(_match_radius,0), COLOR(_ID),1);
	}

	double getAssRadius(){return _match_radius;}	
	int getTemplateNum(){return _template_list.size();}
	vector<Rect>& getResultHistory(){return _result_history;}
	int getID(){return _ID;}

	double getDisToLast(Rect win)
	{
		return sqrt(
			(win.x+0.5*win.width-_result_last_no_sus.x-0.5*_result_last_no_sus.width)*(win.x+0.5*win.width-_result_last_no_sus.x-0.5*_result_last_no_sus.width)+
			(win.y+0.5*win.height-_result_last_no_sus.y-0.5*_result_last_no_sus.height)*(win.y+0.5*win.height-_result_last_no_sus.y-0.5*_result_last_no_sus.height)
			);		
	}

	bool getIsNovice(){return _is_novice;}
	void setIsNovice(bool novice){ this->_is_novice = novice;}
	int getSuspensionCount(){return _novice_status_count;}
	double getHistMatchScore(){return hist_match_score;}
	Rect getResult(){return _result_temp;}
	void setResult(Rect _rect){ this->_result_temp = _rect;}
	Rect getBodysizeResult(){return _result_bodysize_temp;}	
	Rect getResultLastNoSus(){return _result_last_no_sus;}
	void setResultLastNoSus(Rect _rect){ this->_result_last_no_sus = _rect;}
	void setResultBodySizeTemp(Rect _temp){ this->_result_bodysize_temp = _temp;}
	void updateKfCov(double body_width)
	{
		Mat m_temp =*(Mat_<float>(4,4)<<0.025,0,0,0,0,0.025,0,0,0,0,0.25,0,0,0,0,0.25);
		_kf.processNoiseCov = m_temp*((float)body_width/FRAME_RATE)*((float)body_width/FRAME_RATE);
		setIdentity(_kf.measurementNoiseCov, Scalar::all(1.0*(float)body_width*(float)body_width));
	}

	time_t getTimeIn(){ return _time_in; }
	void setTimeIn(time_t _t){ this->_time_in = _t;}
	bool getCalcTime() { return _is_calc_time; }
	void setCalcTime(bool _calc){ this->_is_calc_time = _calc;}
	Rect getRectNearLast();
	bool getIsAssign(){ return _is_assign;}
	void setIsAssign(bool is_assign){ this->_is_assign = is_assign;}
	bool getMarkAssign(){ return this->_mark_assign;}
	void setMarkAssign(bool _mark) { this->_mark_assign = _mark;}
	void init_kf(Rect win)
	{
		_kf.statePost =*(Mat_<float>(4,1)<<win.x+0.5*win.width,win.y+0.5*win.height,0,0);
	}

private:
	void correct_kf(KalmanFilter& kf, Rect win)
	{
		kf.correct(*(Mat_<float>(2,1)<<win.x+0.5*win.width,win.y+0.5*win.height));
	}

	double compareHisto(Mat& h)
	{
		return compareHist(hist, h, CV_COMP_INTERSECT);
	}

	typedef struct TraResult
	{
		Rect window;
		double likelihood;
		TraResult(Rect win,double like){window = win;likelihood = like;}
	}TraResult;

	size_t _refc;
	bool _is_dumped;
	static list<EnsembleTracker*> _TRASH_LIST;

	// Các tham số mặc định phục vụ tính ngưỡng khoảng cách
	double _phi1_,_phi2_,_phi_max_;

	unsigned int _ID;
	bool _is_novice;
	int _novice_status_count;//Đếm số khung hình là novice
	double _match_radius; //Bán kính vùng xét đối tượng

	list<AppTemplate*> _template_list; //Danh sách các template của track
	AppTemplate* _retained_template; //Template nếu đối tượng đứng yên, đánh dấu đứng yên
	int _template_count;
	vector<Rect> _result_history;
	vector<Rect> _filter_result_history;
	KalmanFilter _kf;

	int histSize[3];
	float _hRang[3][2];
	const float* hRange[3];
	int channels[3];
	MatND hist;
	double hist_match_score;	

	Size2f _window_size; //Kích thước vùng đối tượng
	Mat _confidence_map;
	Rect _cm_win;
	Rect _result_temp;//Lưu kết quả meanshift
	Rect _result_last_no_sus;//Lưu kết quả gần nhất không phải là novice dùng để cập nhật lại
	Rect _result_bodysize_temp;

	list<EnsembleTracker*> _neighbors;
	bool _added_new;
	Mat _recentHitRecord; 
	int _record_idx;

	//>>>Biến tính thời gian
	time_t _time_in;
	bool _is_calc_time;
	//<<<Biến tính thời gian

	bool _is_assign;
	bool _mark_assign;
	Enviroment enviroment;
};

