#ifndef DETECT_HH
#define DETECT_HH

#include <utility>
#include "opencv2/opencv.hpp"

std::pair<cv::Rect, bool> detectCharacter(const cv::Mat& image);
cv::Mat getForeGroundMask(const cv::Mat& image);
std::pair<cv::Point2f,bool> findTop(const cv::Mat& mask, int points);

#endif