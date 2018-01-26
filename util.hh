#ifndef UTIL_HH
#define UTIL_HH

#include <vector>

#include "opencv2/opencv.hpp"

inline cv::Point tlPoint(const cv::Rect& rect) {
	return rect.tl();
}

inline cv::Point blPoint(const cv::Rect& rect) {
	return cv::Point(rect.x, rect.y + rect.height);
}

inline cv::Point brPoint(const cv::Rect& rect) {
	return rect.br();
}

inline cv::Point trPoint(const cv::Rect& rect) {
	return cv::Point(rect.x + rect.width, rect.y);
}

cv::Mat resizeImageRatio(const cv::Mat& m, float ratio);

struct Stat {
	Stat();

	int count;
	double sum;
	double mean;
	double median;
	double variance;
	double standardDeviation;
};

class DenoicingSequence {
public:
	explicit DenoicingSequence(int size);
	DenoicingSequence(const DenoicingSequence&) = delete;
	DenoicingSequence& operator=(const DenoicingSequence&) = delete;

	void Push(const double val);
	Stat Get(double errlimit) const;
	void Reset();
	//cv::Mat Debug() const;
private:
	int size;
	std::vector<double> values;
};

#endif