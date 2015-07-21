#include "stdafx.h"
#include "AppTemplate.h"

typedef struct ChannelScore
{
	int idx;
	double score;
	ChannelScore(int i, double s):idx(i), score(s){}
}ChannelScore;

inline bool compareChannel(ChannelScore c1, ChannelScore c2)
{
	return (c1.score < c2.score) ? true:false;
}

//Tính phương sai giữa hai histogram
double getVR(cv::Mat hist1, cv::Mat hist2)
{
	cv::Mat idx = cv::Mat::zeros(hist1.rows, hist1.cols, CV_32FC1);
	for(int i=0; i < idx.rows; ++i)
	{
		float* r_ptr = idx.ptr<float>(i);
		r_ptr[0] = (float)i;
	}

	double mean_idx = hist1.dot(idx);
	cv::Mat temp = idx - mean_idx;
	temp = temp.mul(temp);
	double variance1 = hist1.dot(temp);

	mean_idx = hist2.dot(idx);
	temp = idx - mean_idx;
	temp = temp.mul(temp);
	double variance2 = hist2.dot(temp);

	cv::Mat hist_mean = (hist1 + hist2)*0.5;
	mean_idx = hist_mean.dot(idx);
	temp = idx - mean_idx;
	temp = temp.mul(temp);
	double variance_mean = hist_mean.dot(temp);

	return variance_mean/(variance1+variance2);
}

AppTemplate::AppTemplate(const cv::Mat* frame_set, const Rect iniWin, int ID):ID(ID)
{

	Rect body_win = scaleWin(iniWin, 1/TRACKING_TO_BODYSIZE_RATIO);
	Rect roi_win(body_win.x - body_win.width, body_win.y - body_win.width, 3*body_win.width, 2*body_win.width+body_win.height);

	//Lấy phần chung với frame_set
	body_win = body_win & Rect( 0, 0, frame_set[0].cols, frame_set[0].rows);
	roi_win = roi_win & Rect( 0, 0, frame_set[0].cols, frame_set[0].rows);
	//Lấy tập các vùng trên các channels
	cv::Mat roi_set[] = {cv::Mat(frame_set[0], roi_win), cv::Mat(frame_set[1], roi_win), cv::Mat(frame_set[2], roi_win)};

	//Dịch chuyển khung hình
	Rect iniWin_roi = iniWin - Point(roi_win.x, roi_win.y);

	//Tính điểm cho mỗi channel
	list<ChannelScore> channel_score;

	//Khởi tạo mask
	cv::Mat mask_roi(roi_set[0].rows, roi_set[0].cols, CV_8UC1, Scalar(0));
	rectangle(mask_roi, iniWin_roi, Scalar(255), -1);

	cv::Mat inv_mask_roi(roi_set[0].rows, roi_set[0].cols, CV_8UC1, Scalar(255));
	rectangle(inv_mask_roi, body_win - Point(roi_win.x, roi_win.y), Scalar(0), -1);

	cv::Mat temp_hist;
	cv::Mat temp_bp;
	int hist_size[] = {BIN_NUMBER};
	for(int i=0; i < 9; i++)
	{
		float range1[] = {0, 255};
		if( i==3) 
		{
			range1[1] = 179;
		}

		const float* hist_range[] = {range1};
		cv::calcHist(roi_set, 3, &i, inv_mask_roi, temp_hist, 1, hist_size, hist_range);
		cv::normalize(temp_hist, temp_hist, 255, 0.0, NORM_L1);
		cv::calcBackProject(roi_set, 3, &i, temp_hist, temp_bp, hist_range);

		int c[] = {0};
		int hs[] = {BIN_NUMBER};
		float hr[] = {0,255};
		const float* hrr[] = {hr};
		cv::Mat hist_fore;
		cv::Mat hist_back;
		cv::calcHist(&temp_bp, 1, c, mask_roi, hist_fore, 1, hs, hrr);
		cv::calcHist(&temp_bp, 1, c, inv_mask_roi, hist_back, 1, hs, hrr);

		cv::normalize(hist_fore, hist_fore, 1.0, 0.0, NORM_L1);
		cv::normalize(hist_back, hist_back, 1.0, 0.0, NORM_L1);

		double score = getVR(hist_back, hist_fore);
		score = score==score ? score:0;
		channel_score.push_back(ChannelScore(i, score));
	}

	//Lấy hai channel có điểm cao nhất
	channel_score.sort(compareChannel);
	channels[0] = channel_score.back().idx;
	channel_score.pop_back();
	channels[1] = channel_score.back().idx;

	//Tính histogram vói 2 channel cao điểm nhất
	for(int i=0 ; i < 2; i++)
	{
		_hRang[i][0] = 0;
		if(channels[i] == 3)
			_hRang[i][1] = 179;
		else
			_hRang[i][1] = 255;

		hRang[i] = _hRang[i];
	}

	cv::calcHist(roi_set, 3, channels, inv_mask_roi, temp_hist, 2, hSize, hRang);
	cv::normalize(temp_hist, temp_hist, 255, 0, NORM_L1);

	cv::Mat final_mask;
	cv::calcBackProject(roi_set, 3, channels, temp_hist, final_mask, hRang);
	cv::threshold(final_mask, final_mask, 5, 255, CV_THRESH_BINARY_INV);

	final_mask = cv::min(final_mask, mask_roi);

	//Lấy các đặc trưng tốt nhất 
	cv::Mat hist_fore, hist_back;
	channel_score.clear();
	double sum_score = 0;

	for(int i=0; i < 9; i++)
	{
		float range1[] = {0, 255};
		if(i==3)
		{
			range1[1] = 179;
		}

		const float* hist_range[] = {range1};
		cv::Mat temp_hist_neg;

		cv::calcHist(roi_set, 3, &i, final_mask, temp_hist, 1, hist_size, hist_range);
		cv::normalize(temp_hist, temp_hist, 255, 0, NORM_L1);
		cv::calcHist(roi_set, 3, &i, inv_mask_roi, temp_hist_neg, 1, hist_size, hist_range);
		cv::normalize(temp_hist_neg, temp_hist_neg, 255, 0, NORM_L1);
		cv::log(temp_hist, temp_hist);
		cv::log(temp_hist_neg, temp_hist_neg);
		temp_hist = temp_hist - temp_hist_neg;
		cv::threshold(temp_hist, temp_hist, 0, 255, CV_THRESH_TOZERO);
		cv::normalize(temp_hist, temp_hist, 255, 0.0, NORM_L1);

		cv::calcBackProject(roi_set, 3, &i, temp_hist, temp_bp, hist_range);
		int c[] = {0};
		int hs[] = {BIN_NUMBER};
		float hr[] = {0,255};
		const float* hrr[] = {hr};
		cv::calcHist(&temp_bp, 1, c, final_mask, hist_fore, 1, hs, hrr);
		cv::calcHist(&temp_bp, 1, c,inv_mask_roi, hist_back, 1, hs, hrr);
		cv::normalize(hist_fore, hist_fore, 1.0, 0.0, NORM_L1);
		cv::normalize(hist_back, hist_back, 1.0, 0.0, NORM_L1);
		double score = getVR(hist_back, hist_fore);
		score = score==score ? score:0;
		channel_score.push_back(ChannelScore(i, score));
		sum_score += exp(score);
	}

	channel_score.sort(compareChannel);
	channels[0] = channel_score.back().idx;
	channel_score.pop_back();
	channels[1] = channel_score.back().idx;

	for(int i=0; i < 2; i++)
	{
		_hRang[i][0] = 0;
		if(channels[i] == 3)
			_hRang[i][1] = 179;
		else
			_hRang[i][1] = 255;

		hRang[i] = _hRang[i];
	}

	cv::calcHist(roi_set, 3, channels, final_mask, hist, 2, hSize, hRang);
	cv::normalize(hist, hist, 255, 0, NORM_L1);

	//Tính shift_vector
	cv::Mat backPro;
	cv::calcBackProject(roi_set, 3, channels, hist, backPro, hRang);
	//Tính độ dịch chuyển
	iniWin_roi = iniWin - Point(roi_win.x, roi_win.y);
	Point2f origin_point_roi((float)(iniWin_roi.x + 0.5*iniWin_roi.width), (float)(iniWin_roi.y+0.5*iniWin_roi.height));

	cv::meanShift(backPro, iniWin_roi, TermCriteria(CV_TERMCRIT_EPS|CV_TERMCRIT_ITER, 10, 1));
	Point2f shift_point_roi((float)(iniWin_roi.x+0.5*iniWin_roi.width),(float)(iniWin_roi.y+0.5*iniWin_roi.height));

	shift_vector = (shift_point_roi - origin_point_roi)*(1/(float)iniWin.width);
}

AppTemplate::AppTemplate(const AppTemplate& tracker):ID(tracker.ID)
{
	tracker.hist.copyTo(hist);
	tracker.confidence_map.copyTo(confidence_map);
	shift_vector = tracker.shift_vector;

	for(int i=0; i < 2; i++)
	{
		channels[i] = tracker.channels[i];
		for(int j=0; j < 2; j++)
		{
			_hRang[i][j] = tracker._hRang[i][j];
		}
	}

	hRang[0] = _hRang[0];
	hRang[1] = _hRang[1];
	this->score = tracker.score;
}


AppTemplate::~AppTemplate(void)
{
}

void AppTemplate::calcBP(const Mat* frame_set, Mat& occ_map, Rect ROI)
{
	confidence_map = cv::Mat::zeros(ROI.height, ROI.width, CV_8UC1);
	Rect frame_win(0, 0, frame_set[0].cols, frame_set[0].rows);
	Rect roi = frame_win & ROI;

	cv::Mat roi_set[] = {cv::Mat(frame_set[0], roi), cv::Mat(frame_set[1], roi), cv::Mat(frame_set[2], roi)};

	cv::Mat roi_backproj(confidence_map, roi - Point(ROI.x, ROI.y));
	cv::Mat roi_mask(occ_map, roi);
	roi_backproj.setTo(Scalar(0.0), roi_mask);
	confidence_map.convertTo(confidence_map, CV_32FC1);
}

void AppTemplate::calcScore(Rect b_inner, Rect b_outer)
{
	cv::Mat cm;
	confidence_map.copyTo(cm);

	//Lấy vùng ảnh của đối tượng
	Rect rw = b_inner & Rect(0,0, cm.cols, cm.rows);

	//Tính giá trị trung bình
	Scalar fg = mean(cm(rw));

	cv::Mat mask = cv::Mat::zeros(confidence_map.size(), CV_8UC1);
	cv::rectangle(mask, b_outer, Scalar(1), -1);
	cm.setTo(Scalar(0), mask);

	cv::Mat matching_map;
	if(confidence_map.rows == 0)
	{
		score = 0;
		return;
	}

	cv::matchTemplate(cm, cv::Mat(b_inner.height, b_inner.width, CV_32FC1, Scalar(255)), matching_map, CV_TM_SQDIFF);
	Point minLoc;
	cv::minMaxLoc(matching_map, 0, 0, &minLoc);
	Scalar bg = mean(cm(Rect(minLoc.x, minLoc.y, b_inner.width, b_inner.height)));

	score = (fg[0] - bg[0]);
}