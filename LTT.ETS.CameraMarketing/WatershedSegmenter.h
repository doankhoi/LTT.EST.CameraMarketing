#pragma once
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\highgui\highgui.hpp>

class WatershedSegmenter
{
private:
	cv::Mat markers;
public:
	WatershedSegmenter(void);
	~WatershedSegmenter(void);
	void setMarkers(const cv::Mat& _markers)
	{
		_markers.convertTo(markers, CV_32S);
		cv::imshow("after", markers);
	}

	cv::Mat process(const cv::Mat& image)
	{
		cv::watershed(image, markers);
		markers.convertTo(markers, CV_8U);
		return markers;
	}
};

