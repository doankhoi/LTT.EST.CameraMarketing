#include "stdafx.h"
#include "MultiTrackAssociation.h"
#include <cstdio>
#include <iostream>

#include "Parameter.h"
#include "HungarianAlg.h"
#include "Util.h"

using namespace std;

#define HIST_MATCH_THRESH_CONT 0.3 // 0.4 Tùy chỉnh tỷ lệ mapping histogram

void WaitingList::update()
{
	for (list<Waiting>::iterator it=w_list.begin();it!=w_list.end();)
	{
		if ((*it).life_count > life_limit)
		{
			w_list.erase(it++);
			continue;
		}
		else
		{
			(*it).life_count++;
		}
		it++;
	}
}


vector<Rect> WaitingList::outputQualified(double thresh)
{
	vector<Rect> ret;
	for (list<Waiting>::iterator it=w_list.begin();it!=w_list.end();)
	{
		if ((*it).accu>thresh)
		{
			ret.push_back((*it).currentWin);
			w_list.erase(it++);
			continue;
		}
		it++;
	}
	return ret;
}


void WaitingList::feed(Rect gt_win,double response)
{
	Point center((int)(gt_win.x+0.5*gt_win.width),(int)(gt_win.y+0.5*gt_win.height));
	for (list<Waiting>::iterator it=w_list.begin();it!=w_list.end();it++)
	{
		double x1=center.x;
		double y1=center.y;
		double x2=(*it).center.x;
		double y2=(*it).center.y;
		double dis = sqrt(pow(x1-x2,2.0)+pow(y1-y2,2.0))*FRAME_RATE;
		double scale_ratio=(*it).currentWin.width/(double)gt_win.width;

		if (dis < (*it).currentWin.width*2.3 /*&& scale_ratio < 1.1 && scale_ratio > 0.90*/)
		{
			(*it).currentWin=gt_win;
			(*it).center=Point((int)(gt_win.x+0.5*gt_win.width),(int)(gt_win.y+0.5*gt_win.height));
			(*it).accu++;
			return;
		}
	}

	w_list.push_back(Waiting(gt_win));
}

Controller::Controller(Size sz,int r, int c,double vh,double lr,double thresh_expert)
	:_hit_record(),
	_grid_rows(r),_grid_cols(c),
	_prior_height_variance(vh),
	_frame_size(sz),
	_bodyheight_learning_rate(lr),
	_alpha_hitting_rate(4*TIME_WINDOW_SIZE),_beta_hitting_rate(5),
	waitList((int)TIME_WINDOW_SIZE),
	waitList_suspicious((int)(2*TIME_WINDOW_SIZE)),
	_thresh_for_expert(thresh_expert)
{
	for (int i=0;i<r;i++)
	{
		vector<double> temp1;
		vector<double> temp2;
		for (int j=0;j<c;j++)
		{
			temp1.push_back(-1);//mark
			temp2.push_back(0);
		}
		_bodyheight_map.push_back(temp1);
		_bodyheight_map_count.push_back(temp2);	
	}
}


void Controller::takeVoteForHeight(Rect bodysize_win)
{
	double foot_x=bodysize_win.x+0.5*bodysize_win.width;
	double foot_y=bodysize_win.y+bodysize_win.height;

	int r=(int)MIN(foot_y*_grid_rows/_frame_size.height,_grid_rows-1);
	int c=(int)MIN(foot_x*_grid_cols/_frame_size.width,_grid_cols-1);

	for (int i=-1;i<2;i++)
	{
		for (int j=-1;j<2;j++)
		{
			int r_=MAX(MIN(r+i,_grid_rows-1),0);
			int c_=MAX(MIN(c+j,_grid_cols-1),0);

			if (_bodyheight_map[r_][c_]==-1) 
			{
				_bodyheight_map[r_][c_]=bodysize_win.height;
				_bodyheight_map_count[r_][c_]++;
			}
			else if (_bodyheight_map_count[r][c]>COUNT_NUM)
			{
				_bodyheight_map[r_][c_] = (_bodyheight_learning_rate)*bodysize_win.height+(1-_bodyheight_learning_rate)*_bodyheight_map[r_][c_];
			}
			else // Giữ lại
			{
				_bodyheight_map_count[r_][c_]++;
				_bodyheight_map[r_][c_] = (1/_bodyheight_map_count[r_][c_])*bodysize_win.height + (1-1/_bodyheight_map_count[r_][c_])*_bodyheight_map[r_][c_];
			}
		}
	}
}


vector<int> Controller::filterDetection(vector<Rect> detction_bodysize)
{
	vector<int> ret;
	for (size_t i=0;i < detction_bodysize.size();i++)
	{
		//So sánh với các danh sách không chắc chắn
		bool flag=false;
		for (size_t j=0;j < _suspicious_rect_list.size();j++)
		{
			//Tính tỉ lệ phần trăm overide
			//if (getRectDist(_suspicious_rect_list[j],detction_bodysize[i],OVERLAP)<0.5)
			//{
			//	ret.push_back(BAD); //Đánh dấu phát hiện tồi
			//	flag = true;
			//	break;
			//}

			//Tính tỉ lệ phần trăm trùng cao
			if(getRectDist(_suspicious_rect_list[j], detction_bodysize[i], OVERLAP) > 0.5)
			{
				ret.push_back(GOOD);
				flag = true;
				break;
			}
		}

		if (flag)
			continue;

		//Phân loại các phát hiện
		double foot_x = detction_bodysize[i].x + 0.5*detction_bodysize[i].width;
		double foot_y = detction_bodysize[i].y + 0.5*detction_bodysize[i].height;
		int r = (int)MIN(foot_y*_grid_rows/_frame_size.height,_grid_rows-1);
		int c = (int)MIN(foot_x*_grid_cols/_frame_size.width,_grid_cols-1);

		if (_bodyheight_map[r][c]==-1)
			ret.push_back(NOTSURE);
		else if (abs(_bodyheight_map[r][c]-detction_bodysize[i].height)<(3+10/_bodyheight_map_count[r][c])*sqrt(_prior_height_variance)*_bodyheight_map[r][c])

			ret.push_back(GOOD);
		else
			ret.push_back(BAD);	
	}
	return ret;
}


void Controller::takeVoteForAvgHittingRate(list<EnsembleTracker*> _tracker_list)
{
	double vote_count=0;
	vector<double> hitting;
	for (list<EnsembleTracker*>::iterator it=_tracker_list.begin();it!=_tracker_list.end();it++)
	{
		Rect win = (*it)->getBodysizeResult();
		// Chỉ đánh giá các đối tượng di chuyển
		if ((*it)->getVel() > win.width*0.7)	
			_hit_record.recordVote((*it)->getAddNew());
	}
}


void Controller::deleteObsoleteTracker(list<EnsembleTracker*>& _tracker_list)
{
	waitList_suspicious.update();
	double l=_hit_record._getAvgHittingRate(_alpha_hitting_rate,_beta_hitting_rate);

	for (list<EnsembleTracker*>::iterator it = _tracker_list.begin();it!=_tracker_list.end();)
	{	
		if((*it)->getHitFreq()*TIME_WINDOW_SIZE <= MAX(l-2*sqrt(l),0))
		{
			(*it)->refcDec1();
			(*it)->dump(); //đẩy vào trash list
			_tracker_list.erase(it++);
			continue;
		}		
		else if (!(*it)->getIsNovice() && (*it)->getTemplateNum() < _thresh_for_expert)
		{
			(*it)->demote();
		}
		it++;
	}		
}	

//Tính toán các vùng đối tượng không chắc chắn
void Controller::calcSuspiciousArea(list<EnsembleTracker*>& _tracker_list)
{
	double l=_hit_record._getAvgHittingRate(_alpha_hitting_rate,_beta_hitting_rate);
	waitList_suspicious.update();

	for (list<EnsembleTracker*>::iterator it = _tracker_list.begin();it!=_tracker_list.end();)
	{	
		if ((*it)->getAddNew() && // phát hiện mới
			(*it)->getHitFreq()*TIME_WINDOW_SIZE < l-sqrt(l) && // tỉ lệ thấp
			(*it)->getVel() < (*it)->getBodysizeResult().width*0.14)// gần như không di chuyển
		{
			waitList_suspicious.feed((*it)->getBodysizeResult(),1);
		}

		it++;
	}

	vector<Rect> sus_rects = waitList_suspicious.outputQualified(0.3*TIME_WINDOW_SIZE); //0.4
	for (size_t i=0;i < sus_rects.size();i++)
	{
		_suspicious_rect_list.push_back(sus_rects[i]);
	}
}

TrakerManager::TrakerManager(Detector* detector,Mat& frame,double thresh_promotion)
	:_detector(detector),
	_my_char(0),
	//_frame_count(0),
	_tracker_count(0),
	_controller(frame.size(),8,8,0.01,1/COUNT_NUM,thresh_promotion)
{

	//>>> Kết nối database
	db = NULL;
	db = (this->connectDb).connectDb(DATABASE_PATH);
	if(db == NULL)
	{
		cout << "Not open database" << endl;
		system("pause");
		exit(-1);
	}

	SHOP_CD = (this->connectDb).getShopInfo(db);
	//<<< Kết nối database 
}


TrakerManager::~TrakerManager()
{
	for (list<EnsembleTracker*>::iterator i=_tracker_list.begin();i!=_tracker_list.end();i++)
		delete *i;
}


void TrakerManager::doHungarianAlg(const vector<Rect>& detections)
{
	_controller.waitList.update();

	list<EnsembleTracker*> expert_class;
	list<EnsembleTracker*> novice_class;

	vector<Rect> detection_left;
	//Phân loại các track
	for (list<EnsembleTracker*>::iterator it=_tracker_list.begin();it!=_tracker_list.end();it++)
	{
		if ((*it)->getIsNovice())
			novice_class.push_back((*it));
		else
			expert_class.push_back((*it));
	}

	//Gán lại các đối tương expert
	int hp_size=expert_class.size();
	int dt_size=detections.size();

	if (dt_size*hp_size>0)
	{
		Matrix<double> matrix(dt_size, hp_size+dt_size);
		vector<bool> indicator;

		for (int i=0;i<dt_size;i++)
		{
			Rect detect_win_GTsize = scaleWin(detections[i],BODYSIZE_TO_DETECTION_RATIO);
			Rect shrinkWin=scaleWin(detections[i],TRACKING_TO_DETECTION_RATIO);
			list<EnsembleTracker*>::iterator j_tl = expert_class.begin();

			for (int j=0; j<hp_size+dt_size;j++)
			{
				if (j<hp_size)
				{
					Rect currentWin=(*j_tl)->getResult();
					double currentWin_cx = currentWin.x+0.5*currentWin.width+0.5;
					double currentWin_cy = currentWin.y+0.5*currentWin.height+0.5;
					double detectWin_cx = detections[i].x+0.5*detections[i].width+0.5;
					double detectWin_cy=detections[i].y+0.5*detections[i].height+0.5;
					double d=sqrt(pow(currentWin_cx-detectWin_cx,2.0)+pow(currentWin_cy-detectWin_cy,2.0));

					double ratio=(double)(*j_tl)->getBodysizeResult().width/detect_win_GTsize.width;

					if (d<(*j_tl)->getAssRadius() /*&& ratio<1.2 && ratio>0.8*/)
					{
						double dis_to_last=(*j_tl)->getDisToLast(shrinkWin);

						if (dis_to_last/(((double)(*j_tl)->getSuspensionCount()+1)/(FRAME_RATE*5/7)+0.5)<((*j_tl)->getBodysizeResult().width*1.0))
						{								
							matrix(i,j)=d;//*h;
						}
						else
							matrix(i,j)=INFINITY;
					}
					else
						matrix(i,j)=INFINITY;
					j_tl++;
				}
				else
					matrix(i,j)=100000;// dummy
			}
		}

		HungarianAlg m;
		m.solve(matrix);

		for (int i=0;i<dt_size;i++)
		{
			bool flag=false;
			list<EnsembleTracker*>::iterator j_tl=expert_class.begin();
			Rect shrinkWin=scaleWin(detections[i],TRACKING_TO_DETECTION_RATIO);

			for (int j=0;j<hp_size;j++)
			{
				if (matrix(i,j)==0)//matched
				{
					(*j_tl)->addAppTemplate(_frame_set,shrinkWin);//Sẽ thay đổi result_temp nếu demoted
					flag=true;

					if ((*j_tl)->getIsNovice())
						(*j_tl)->promote();					

					while((*j_tl)->getTemplateNum() > MAX_TEMPLATE_SIZE)
						(*j_tl)->deletePoorestTemplate();

					break;
				}
				j_tl++;
			}
			if (!flag )
				detection_left.push_back(detections[i]);
		}
	}
	else
		detection_left=detections;


	//Gán các phát hiện với phân lớp novice
	dt_size=detection_left.size();
	hp_size=novice_class.size();

	if (dt_size*hp_size>0)
	{
		Matrix<double> matrix(dt_size, hp_size+dt_size);
		for (int i=0;i<dt_size;i++)
		{
			Rect detect_win_GTsize=scaleWin(detection_left[i],BODYSIZE_TO_DETECTION_RATIO);
			Rect shrinkWin=scaleWin(detection_left[i],TRACKING_TO_DETECTION_RATIO);
			list<EnsembleTracker*>::iterator j_tl=novice_class.begin();
			for (int j=0; j<hp_size+dt_size;j++)
			{
				if (j<hp_size)
				{
					Rect currentWin=(*j_tl)->getResult();
					double currentWin_cx=currentWin.x+0.5*currentWin.width+0.5;
					double currentWin_cy=currentWin.y+0.5*currentWin.height+0.5;
					double detectWin_cx=detection_left[i].x+0.5*detection_left[i].width+0.5;
					double detectWin_cy=detection_left[i].y+0.5*detection_left[i].height+0.5;
					double d=sqrt(pow(currentWin_cx-detectWin_cx,2.0)+pow(currentWin_cy-detectWin_cy,2.0));
					double ratio=(double)(*j_tl)->getBodysizeResult().width/detect_win_GTsize.width;
					if (d<(*j_tl)->getAssRadius()/* && ratio<1.2 && ratio>0.8*/)
					{			
						double dis_to_last=(*j_tl)->getDisToLast(shrinkWin);

						if (dis_to_last/(((double)(*j_tl)->getSuspensionCount()+1)/(FRAME_RATE*5/7)+0.5)<((*j_tl)->getBodysizeResult().width)*2)					
							matrix(i,j)=d;
						else
							matrix(i,j)=INFINITY;
					}
					else
						matrix(i,j)=INFINITY;
					j_tl++;
				}
				else
					matrix(i,j)=100000; // dummy
			}
		}

		HungarianAlg m;
		m.solve(matrix);
		for (int i=0;i<dt_size;i++)
		{
			bool flag=false;
			list<EnsembleTracker*>::iterator j_tl=novice_class.begin();
			Rect shrinkWin=scaleWin(detection_left[i],TRACKING_TO_DETECTION_RATIO);
			for (int j=0;j<hp_size;j++)
			{
				if (matrix(i,j)==0)//matched
				{
					(*j_tl)->addAppTemplate(_frame_set,shrinkWin);
					flag=true;
					if ((*j_tl)->getIsNovice())
						(*j_tl)->promote();					

					while((*j_tl)->getTemplateNum()>MAX_TEMPLATE_SIZE)
						(*j_tl)->deletePoorestTemplate();

					break;
				}
				j_tl++;
			}
			if (!flag )
				_controller.waitList.feed(scaleWin(detection_left[i],BODYSIZE_TO_DETECTION_RATIO),1.0);
		}
	}
	//Khởi tạo vị trí
	else if (dt_size>0)
	{
		for (int i=0;i<dt_size;i++)
			_controller.waitList.feed(scaleWin(detection_left[i],BODYSIZE_TO_DETECTION_RATIO),1.0);
	}
}


void TrakerManager::doWork(Mat& frame)
{
	Mat frame_resize;
	resize(frame,frame_resize,Size((int)(frame.cols*HOG_DETECT_FRAME_RATIO),(int)(frame.rows*HOG_DETECT_FRAME_RATIO)));
	_detector->detect(frame_resize);

	Mat bgr,hsv,lab;
 	frame.copyTo(bgr);
	cvtColor(frame,hsv,CV_RGB2HSV);
	cvtColor(frame,lab,CV_RGB2Lab);
	Mat frame_set[]={bgr,hsv,lab};
	_frame_set = frame_set;

	// Khởi tạo bản đồ phân phối
	_occupancy_map = Mat(frame.rows,frame.cols,CV_8UC1,Scalar(0));
	
	//Lấy các vùng phát hiện đối tượng
	vector<Rect> detections = _detector->getDetection();
	vector<int> det_filter; //Phân loại các phát hiện

	//Lọc các phát hiện
	if (detections.size()>0)
	{
		//Scale kích thước các phát hiện
		vector<Rect> detection_bodysize;
		for (size_t i = 0;i < detections.size();i++)
		{
			detection_bodysize.push_back(scaleWin(detections[i],BODYSIZE_TO_DETECTION_RATIO));
		}
		//Phân loại các phát hiện
		det_filter = _controller.filterDetection(detection_bodysize);
	}

	vector<Rect> good_detections; //Bao gồm cả GOOD và NOTSURE
	for (size_t k=0;k < detections.size();k++)
	{
		if (det_filter[k] != BAD)
			good_detections.push_back(detections[k]);
	}

	//Xóa trash
	EnsembleTracker::emptyTrash();	

	//Cập nhật tỷ lệ matching rate
	_controller.takeVoteForAvgHittingRate(_tracker_list);
	//_controller.getQualifiedCandidates();
	_controller.deleteObsoleteTracker(_tracker_list);
	_controller.calcSuspiciousArea(_tracker_list);

	//Vẽ các phát hiện
	for (size_t it = 0;it < detections.size();it++)
	{
		if (det_filter[it] != BAD)
			rectangle(frame,detections[it],Scalar(0,0,255),2);
		else
			rectangle(frame,detections[it],Scalar(0,0,255),1); //Phát hiện tồi	
	}

	//Tracking đối với mỗi track và quản lý các template của nó
	for (list<EnsembleTracker*>::iterator i = _tracker_list.begin(); i != _tracker_list.end();)
	{	
		(*i)->calcConfidenceMap(_frame_set, _occupancy_map);
		(*i)->track(_frame_set, _occupancy_map);
		(*i)->calcScore();
		(*i)->deletePoorTemplate(0.0);

		// Cập nhật các neighbors
		(*i)->updateNeighbors(_tracker_list);

		// tăng tỷ lệ các track expert nếu nó di chuyển
		if (!(*i)->getIsNovice() && ((*i)->getVel() > (*i)->getBodysizeResult().width*0.3)) // 0.42
			_controller.takeVoteForHeight((*i)->getBodysizeResult());

		//Cập nhật bản đồ track nếu nó có số lượng template hoac la expert
		if (!(*i)->getIsNovice() && (*i)->getTemplateNum() > 0 )
			rectangle(_occupancy_map,(*i)->getResult(),Scalar(1),-1);

		//Xóa các tracker ra ngoài 
		Rect avgWin=(*i)->getResult();

		if (avgWin.x <= 0 || 
			avgWin.x+avgWin.width >= _frame_set[0].cols-1 || 
			avgWin.y <= 0 || 
			avgWin.y+avgWin.height>=_frame_set[0].rows-1)
		{
			(*i)->refcDec1();
			(*i)->dump();
			_tracker_list.erase(i++);
			continue;
		}		

		i++;			
	}

	// Gán các phát hiện tốt
	doHungarianAlg(good_detections);	

	//Tạo mới các track phát hiện là mới
	vector<Rect> qualified = _controller.getQualifiedCandidates();
	for (size_t i=0;i<qualified.size();i++)
	{
		if (_tracker_list.size() < MAX_TRACKER_NUM)
		{
			EnsembleTracker* tracker = new EnsembleTracker(_tracker_count,Size(qualified[i].width,qualified[i].height));
			tracker->refcAdd1();
			Rect iniWin = scaleWin(qualified[i],TRACKING_TO_BODYSIZE_RATIO);
			tracker->addAppTemplate(_frame_set,iniWin);
			_tracker_list.push_back(tracker);
			_tracker_count++;	
		}			
	}

	//Hiện kết quả
	for (list<EnsembleTracker*>::iterator i=_tracker_list.begin();i!=_tracker_list.end();i++)
	{
		(*i)->registerTrackResult();

		if (!(*i)->getIsNovice())
		{
			(*i)->updateMatchHist(bgr);
		}

		//Nếu có danh sách các kết quả
		if ((*i)->getResultHistory().size() > 0) // >= 0
		{				
			if (!(*i)->getIsNovice() || ((*i)->getIsNovice() && (*i)->compareHisto(bgr,(*i)->getBodysizeResult()) > HIST_MATCH_THRESH_CONT))
			{
				(*i)->drawResult(frame,1/TRACKING_TO_BODYSIZE_RATIO);

				Rect win = (*i)->getResultHistory().back();
				Point tx(win.x, win.y-1);
				char buff[10];
				sprintf(buff,"%d",(*i)->getID());
				string s = buff;
				putText(frame,s,tx,FONT_HERSHEY_PLAIN , 1.5,COLOR((*i)->getID()),2);
			}				
		}

	}

	// Sắp xếp trên số lượng templa
	_tracker_list.sort(TrakerManager::compareTraGroup);
	//_frame_count++;
}

string TrakerManager::getCurrentDate()
{
	time_t t;
	time(&t);
	struct tm timeinfo;
	localtime_s(&timeinfo, &t);
	char time[10];
	const char c[] = "%Y%m%d";
	strftime(time, sizeof(time) - 1, c, &timeinfo);
	string time_curr(time);
	return time_curr;
}


string TrakerManager::getexepath()
{
	CHAR pBuf[MAX_PATH];
	int bytes = GetModuleFileNameA(NULL, pBuf, MAX_PATH);
	string path = string(pBuf);

	string newpath;  
	for (int i = 0; i < path.length() ;i++)
	{
		if(path.at(i) == '\\')
		{
			newpath.append("\\");
			newpath.append("\\");
		}
		else
			newpath.append(path.substr(i,1));
	}
	return newpath;
}

void TrakerManager::callPro()
{
	string path = getexepath();
	char* urlA = new char[path.length() + 1];
	strcpy(urlA, path.c_str());

	wchar_t urlW[ MAX_PATH ];
	std::copy( urlA, urlA + lstrlenA( urlA ) + 1, urlW );
	if ((int)ShellExecuteW( NULL, L"open", urlW, NULL, NULL, SW_SHOWNORMAL ) < 32)
	{
		cout << "Error call program" <<  endl;
		exit(EXIT_FAILURE);
	}

	destroyAllWindows();
	exit(EXIT_SUCCESS);
}

int TrakerManager::getTime()
{
	time_t t;
	time(&t);
	struct tm timeinfo;
	localtime_s(&timeinfo, &t);
	char time[4];
	const char c[] = "%H";
	strftime(time, sizeof(time) - 1, c, &timeinfo);
	string time_curr(time);
	return stoi(time_curr);
}