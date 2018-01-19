#ifndef SAVE_HH
#define SAVE_HH

#include "opencv2/opencv.hpp"

void save(const cv::Mat& image, cv::Rect pos, cv::Rect bg, const std::string& name);

#endif