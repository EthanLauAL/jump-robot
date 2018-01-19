#ifndef FIND_HH
#define FIND_HH

#include "opencv2/opencv.hpp"

const cv::Rect notFound(-1, -1, -1, -1);

cv::Rect find(const cv::Mat& image, const cv::Point p,
		const cv::Size normalSize, const cv::Size biggestSize);

#endif