#ifndef DETECT_HH
#define DETECT_HH

#include <utility>
#include "opencv2/opencv.hpp"

std::pair<cv::Rect, bool> detectCharacter(const cv::Mat& image);
cv::Mat getBorder(const cv::Mat& image);
std::pair<cv::Point2f,bool> findTop(const cv::Mat& mask, int points);

cv::Mat sumDerivative(const cv::Mat& image, bool x, bool y);
float symmetry(const cv::Mat& derivative, float min = -1, float max = -1);

#endif