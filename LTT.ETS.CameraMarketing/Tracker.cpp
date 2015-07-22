#include "stdafx.h"
#include "Tracker.h"

#define SCALE_UPDATE_RATE 0.4
#define HIST_MATCH_UPDATE 0.01

list<EnsembleTracker*> EnsembleTracker::_TRASH_LIST;

void EnsembleTracker::dump()
{
	for (list<EnsembleTracker*>::iterator it = _neighbors.begin();it != _neighbors.end(); it++)
	{
		(*it)->refcDec1();
	}

	_TRASH_LIST.push_back(this); //Đẩy đối tượng vào trash_list
	_is_dumped = true;
}


void EnsembleTracker::emptyTrash()
{
	//Xóa các track không còn tham chiếu
	for (list<EnsembleTracker*>::iterator it=_TRASH_LIST.begin();it != _TRASH_LIST.end();)
	{
		if ((*it)->_refc == 0)
		{
			delete (*it);
			_TRASH_LIST.erase(it++);
			continue;
		}

		it++;
	}
}

inline bool compareTemplate(AppTemplate* t1, AppTemplate* t2)
{
	return (t1->getScore() > t2->getScore()) ? true:false;
}

EnsembleTracker::EnsembleTracker(int id,Size body_size,double phi1,double phi2,double phi_max)
	:_refc(0),_is_dumped(false),
	_phi1_(phi1),
	_phi2_(phi2),
	_phi_max_(phi_max),
	_novice_status_count(0),
	_template_count(0),
	_ID(id),
	_kf(4,2,0),
	_is_novice(false),
	_match_radius(0),
	hist_match_score(0),
	_added_new(true),
	_record_idx(0)
{
	_retained_template=0;
	//initialize kalman filter
	_kf.transitionMatrix =* (Mat_<float>(4,4)<<1,0,1,0,0,1,0,1,0,0,1,0,0,0,0,1);
	setIdentity(_kf.measurementMatrix);
	updateKfCov(body_size.width);
	setIdentity(_kf.errorCovPost,Scalar::all(3.0*(double)body_size.width*body_size.width));

	// RGB
	histSize[0]=8;histSize[1]=8;histSize[2]=8;
	channels[0]=0;channels[1]=1;channels[2]=2;
	for (int i=0;i<3;++i)
	{
		_hRang[i][0]=0;
		_hRang[i][1]=255;	
		hRange[i]=_hRang[i];
	}

	_recentHitRecord=Mat::zeros(2, 4*FRAME_RATE, CV_64FC1);
	
	//>>>Cài đặt thời gian
	this->_is_calc_time = false;
	time_t c;
	time(&c);
	this->_time_in = c;
	//<<<Cài đặt thời gian
}

EnsembleTracker::~EnsembleTracker()
{
	list<AppTemplate*>::iterator it;
	for (it=_template_list.begin(); it!=_template_list.end();it++)
	{
		delete *it;
	}
	delete _retained_template;
}


void EnsembleTracker::updateNeighbors(
	list<EnsembleTracker*> tr_list,
	double dis_thresh_r,
	double scale_r1, double scale_r2,
	double hist_thresh)
{
	//Xóa các track gần kề
	for (list<EnsembleTracker*>::iterator it=_neighbors.begin();it!=_neighbors.end();)
	{
		//Xóa các neighbors là dump
		if ((*it)->getIsDumped())
		{
			(*it)->refcDec1();
			_neighbors.erase(it++);
			continue;
		}

		Rect r=(*it)->getBodysizeResult();
		Point2f c1(r.x+0.5f*r.width,r.y+0.5f*r.height);
		Point2f c2(_result_bodysize_temp.x+0.5f*_result_bodysize_temp.width,_result_bodysize_temp.y+0.5f*_result_bodysize_temp.height);
		double dis=sqrt(pow(c1.x-c2.x,2.0f)+pow(c1.y-c2.y,2.0f));
		double scale_ratio=(double)r.width/(double)_result_bodysize_temp.width;

		// Xóa các track ben cạnh nếu chúng là novice hoặc nó nằm ngoài bán kính cần xét hoặc ngoài ngưỡng
		if (
			dis > dis_thresh_r*_match_radius || 
			scale_ratio > scale_r1 ||scale_ratio < scale_r2 || 
			(*it)->getIsNovice())
		{
			(*it)->refcDec1();
			_neighbors.erase(it++);
			continue;
		}
		it++;
	}

	// Bổ sung các track lân cận
	for (list<EnsembleTracker*>::iterator it=tr_list.begin();it!=tr_list.end();it++)
	{
		//Nếu nó là novice hoặc là chính bản thân nó
		if ((*it)->getIsNovice() || (*it)->getID()==_ID)
		{
			continue;
		}

		bool alreadyFound=false;
		for (list<EnsembleTracker*>::iterator k=_neighbors.begin();k!=_neighbors.end();k++)
		{
			if ((*k)->getID()==(*it)->getID())
			{
				alreadyFound=true;
				continue;
			}
		}

	/*	if (alreadyFound)
		{
			continue;
		}*/

		Rect r =(*it)->getBodysizeResult();
		Point c((int)(r.x+0.5*r.width),(int)(r.y+0.5*r.height));
		double dis=sqrt((_result_bodysize_temp.x+0.5*_result_bodysize_temp.width-c.x)*(_result_bodysize_temp.x+0.5*_result_bodysize_temp.width-c.x)+(_result_bodysize_temp.y+0.5*_result_bodysize_temp.height-c.y)*(_result_bodysize_temp.y+0.5*_result_bodysize_temp.height-c.y));

		double scale_ratio=(double)r.width/(double)_result_bodysize_temp.width;
		double h_match = compareHisto((*it)->hist); //Tính độ tương đồng histogram

		if (
			dis < dis_thresh_r*_match_radius && 
			scale_ratio < scale_r1 && scale_ratio > scale_r2 && 
			h_match > hist_thresh)
		{
			(*it)->refcAdd1();
			_neighbors.push_back((*it));
		}
	}	
}

void EnsembleTracker::addAppTemplate(const Mat* frame_set,Rect iniWin)
{
	setAddNew(true);
	_recentHitRecord.at<double>(0,_record_idx)=1.0;
	_recentHitRecord.at<double>(1,_record_idx)=1.0;

	//Thêm vào _template_list
	AppTemplate* tra_template=new AppTemplate(frame_set,iniWin,_template_count);
	_template_list.push_back(tra_template);

	//cập nhật window size 60% _widown_size 40% là iniWin
	Size2f detection_size((float)iniWin.width,(float)iniWin.height);
	_window_size.width= (_template_list.size()==1) ? (float)iniWin.width : (float)(_window_size.width*(1-SCALE_UPDATE_RATE)+detection_size.width*SCALE_UPDATE_RATE);
	_window_size.height= (_template_list.size()==1) ? (float)iniWin.height : (float)(_window_size.height*(1-SCALE_UPDATE_RATE)+detection_size.height*SCALE_UPDATE_RATE);

	// Cập nhật track
	if (
		_result_history.size()==0|| // Nếu là track mới
		getIsNovice()) // Nếu là novice
	{
		_result_temp=iniWin;
		_result_last_no_sus=iniWin;
		_result_bodysize_temp = scaleWin(iniWin,1/TRACKING_TO_BODYSIZE_RATIO);
		_retained_template = new AppTemplate(*tra_template);
	}
	_template_count++;
}

void EnsembleTracker::calcConfidenceMap(const Mat* frame_set,Mat& occ_map)
{
	//Dự đoán với kalman filter
	_kf.predict();
	Point center((int)_kf.statePre.at<float>(0,0),(int)_kf.statePre.at<float>(1,0));
	double w = _window_size.width/TRACKING_TO_BODYSIZE_RATIO;
	double h = _window_size.height/TRACKING_TO_BODYSIZE_RATIO; 
	h += 2*w;
	w += 2*w;

	Rect roi_win((int)(center.x-0.5*w), (int)(center.y-0.5*h),(int)w,(int)h);
	_cm_win = roi_win;
	_confidence_map = Mat::zeros((int)h,(int)w,CV_32FC1);

	
	Mat final_occ_map;
	occ_map.copyTo(final_occ_map);
	//duyệt các neighbors
	for (list<EnsembleTracker*>::iterator it=_neighbors.begin();it!=_neighbors.end();it++)
	{
		// Tính mask neighbors' nếu chúng không phải novice
		if ((*it)==NULL || (*it)->getIsNovice() || (*it)->getTemplateNum() < ( int)_template_list.size())
			continue;

		Rect r = scaleWin((*it)->getBodysizeResult(),1.0);
		//Đánh dấu đối tượng đã được dùng
		ellipse(final_occ_map,Point((int)(r.x+0.5*r.width),(int)(r.y+0.5*r.height)),Size((int)(0.5*r.width),(int)(0.5*r.height)),0,0,360,Scalar(1),-1);
	}

	//Trường hợp không phải là novice hoặc còn template
	if (!getIsNovice() || _template_list.size()>0)
	{
		list<AppTemplate*>::iterator it;
		float c=0;
		for (it = _template_list.begin();it != _template_list.end(); it++)
		{
			AppTemplate* tr = *it;
			Point shift_vector = tr->getShiftVector()*_window_size.width; //tính toán shiftvector
			tr->calcBP(frame_set,final_occ_map,roi_win+shift_vector);
			_confidence_map += tr->getConfidenceMap();
			c+=1;
		} 
		_confidence_map /= MAX(c,0.0001);
	}
	else//Khi track đứng lại
	{
		Point shift_vector = _retained_template->getShiftVector()*_window_size.width;
		_retained_template->calcBP(frame_set,final_occ_map,roi_win+shift_vector);
		_confidence_map += _retained_template->getConfidenceMap();
	}	
}


void EnsembleTracker::track(const Mat* frame_set,Mat& occ_map)
{
	// cập nhật ma trận phương sai kalman filter
	updateKfCov(getBodysizeResult().width);

	_record_idx = (_record_idx+1)-_recentHitRecord.cols*((_record_idx+1)/_recentHitRecord.cols);
	_recentHitRecord.at<double>(0,_record_idx)=0.0;
	_recentHitRecord.at<double>(1,_record_idx)=0.0;

	//Cập nhật cờ nếu là track được match
	setAddNew(false);

	_kf.predict();

	//Tăng biến đếm nếu là novicce
	if (getIsNovice())
		_novice_status_count++;

	double alpha;

	//Cập nhật lại bán kính của track
	alpha = MIN(_phi1_*sqrt(_kf.errorCovPre.at<float>(0,0))/(double)_result_bodysize_temp.width+_phi2_,_phi_max_);
	_match_radius=alpha*_result_bodysize_temp.width;

	Rect iniWin(
		(int)(0.5*(_confidence_map.cols-_window_size.width)),
		(int)(0.5*(_confidence_map.rows-_window_size.height)),
		(int)_window_size.width,
		(int)_window_size.height);

	//meanShift(_confidence_map,iniWin,TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));

	_result_temp = iniWin + Point(_cm_win.x,_cm_win.y);
	_result_bodysize_temp = scaleWin(_result_temp,1/TRACKING_TO_BODYSIZE_RATIO);

	if (getIsNovice()) //nếu là track chưa chắc chắn
	{
		_kf.errorCovPre.copyTo(_kf.errorCovPost);
		_kf.statePost.at<float>(0,0)=(float)(_result_temp.x+0.5*_result_temp.width);
		_kf.statePost.at<float>(1,0)=(float)(_result_temp.y+0.5*_result_temp.height);
	}
	else
	{		
		// cập nhật lại vị trí
		_result_last_no_sus = _result_temp;
		correct_kf(_kf,_result_temp);
	}	
}

void EnsembleTracker::calcScore()
{
	Rect roi_result=_result_temp-Point(_cm_win.x,_cm_win.y);
	Rect roi_bodysize = scaleWin(roi_result,1/TRACKING_TO_BODYSIZE_RATIO);

	if (getIsNovice())
		return;

	list<AppTemplate*>::iterator it;
	for (it=_template_list.begin();it!=_template_list.end();it++)
	{
		if (!getIsNovice())
		{
			(*it)->calcScore(roi_result,roi_bodysize);
		}		
	}
	_template_list.sort(compareTemplate);//high to low
}

void EnsembleTracker::deletePoorTemplate(double threshold)
{
	if (getIsNovice())
		return;

	list<AppTemplate*>::iterator it;
	for (it=_template_list.begin();it!=_template_list.end();)
	{
		AppTemplate* tr = *it;
		if (tr->getScore() <= threshold)
		{
			if (_template_list.size()==1)
			{
				delete _retained_template;
				_retained_template = tr;
				_template_list.erase(it++);
				continue;
			}
			delete tr;
			_template_list.erase(it++);
		}
		else
			it++;
	}
}


void EnsembleTracker::deletePoorestTemplate()
{
	delete _template_list.back();
	_template_list.pop_back();
}

void EnsembleTracker::demote()
{
	_is_novice=true;
	_novice_status_count=0;
}

void EnsembleTracker::promote()
{
	_is_novice=false;
	_novice_status_count=0;

	//Tạo lại kalmal filter
	Rect win=getResult();
	_kf.statePost.at<float>(0,0)=(float)(win.x+0.5*win.width);
	_kf.statePost.at<float>(1,0)=(float)(win.y+0.5*win.height);
}

//Cập nhật hist
void EnsembleTracker::updateMatchHist(Mat& frame)
{
	Rect roi_result=getResult();
	Rect roi_result_bodysize = scaleWin(roi_result,1/TRACKING_TO_BODYSIZE_RATIO);
	Rect win = roi_result_bodysize & Rect(0,0,frame.cols,frame.rows);
	Mat roi(frame,win);
	Mat temp;
	Mat mask_win = Mat::zeros(roi.size(),CV_8UC1);
	ellipse(mask_win,Point((int)(0.5*mask_win.cols),(int)(0.5*mask_win.rows)),Size((int)(0.35*mask_win.cols),(int)(0.35*mask_win.rows)),0,0,360,Scalar(1),-1);
	calcHist(&roi,1,channels,mask_win,temp,3,histSize,hRange);
	normalize(temp,temp,1,0,NORM_L1);

	if (_result_history.size()==1)//Nếu là tracker mới
	{
		hist_match_score=1;
		hist=temp;
		return;
	}

	hist_match_score = compareHist(hist,temp,CV_COMP_INTERSECT);
	hist += HIST_MATCH_UPDATE*temp;
	normalize(hist,hist,1,0,NORM_L1);
}

double EnsembleTracker::compareHisto(Mat& frame, Rect win)
{
	Rect roi_win = win & Rect(0,0,frame.cols,frame.rows);
	Mat roi(frame,roi_win);
	Mat temp;
	Mat mask_win=Mat::zeros(roi.size(),CV_8UC1);
	ellipse(mask_win,Point((int)(0.5*mask_win.cols),(int)(0.5*mask_win.rows)),Size((int)(0.35*mask_win.cols),(int)(0.35*mask_win.rows)),0,0,360,Scalar(1),-1);
	calcHist(&roi,1,channels,Mat(),temp,3,histSize,hRange);
	normalize(temp,temp,1,0,NORM_L1);
	return compareHist(hist,temp,CV_COMP_INTERSECT);
}

void EnsembleTracker::registerTrackResult()
{
	if (!getIsNovice())
	{
		_result_history.push_back(_result_temp);
		_result_last_no_sus = _result_temp;
		if (_result_history.size()==1)//tracker mới
		{
			init_kf(_result_temp);
		}	
	}
	else
	{
		_result_history.push_back(_result_temp);
	}		
}	

Rect EnsembleTracker::getRectNearLast()
{
	if(_result_history.size() <= 1)
		return _result_temp;

	vector<Rect>::iterator itbegin = _result_history.begin();
	vector<Rect>::iterator itend = _result_history.end();

	while(itbegin != itend)
	{
		if(((itbegin + 1) == itend ))
		{
			return *(itbegin -1);
		}
		itbegin++;
	}

	return _result_history.back();
}