#include "detect.hh"

#include <mutex>
#include <unordered_set>

cv::CascadeClassifier& charDetecter() {
	static cv::CascadeClassifier d("cascade.xml");
	return d;
}

std::mutex charDetecterMutex;

std::pair<cv::Rect, bool> detectCharacter(const cv::Mat& image) {
	std::lock_guard<std::mutex> lock(charDetecterMutex);

	cv::Mat gray(image.size(), CV_8UC1);
	cv::cvtColor(image, gray, CV_BGR2GRAY);
	std::vector<cv::Rect> objs;
	charDetecter().detectMultiScale(gray, objs, 1.1,
    	3, 0, cv::Size(40,40), cv::Size(80,80));

	if (objs.size() == 0) {
		return std::make_pair(cv::Rect(0,0,0,0), false);
	} else {
		return std::make_pair(objs[0], true);
	}
}

/*
template<int Channels>
cv::Vec<uchar, Channels> ntpColor(const cv::Mat& image, double pos) {
	typedef cv::Vec<uchar, Channels> Vec;

	assert(image.channels() == Channels && image.depth() == CV_8U);

	std::vector<std::vector<uchar>> c;
	for (int i = 0 ; i < Channels ; i++) {
		c.push_back(std::vector<uchar>(image.total()));
	}

	auto iter = image.begin<Vec>();
	auto end = image.end<Vec>();
	int idx = 0;
	for ( ; iter < end; iter++, idx++) {
		for (int i = 0 ; i < Channels ; i++) {
			c[i][idx] = (*iter)[i];
		}
	}

	Vec result;
	for (int i = 0 ; i < Channels ; i++) {
		auto m = c[i].begin() + c[i].size() * pos;
		std::nth_element(c[i].begin(), m, c[i].end());
		result[i] = *m;
	}
	return result;
}
*/

cv::Mat getBorder(const cv::Mat& image) {
	assert(image.type() == CV_8UC3);

	cv::Mat tmp = image;
	cv::Mat tmp2;

	tmp2 = cv::Mat(tmp.size(), tmp.type());
	cv::blur(tmp, tmp2, cv::Size(3,3));
	tmp = std::move(tmp2);

	tmp2 = cv::Mat(tmp.size(), CV_8UC3);
	cv::cvtColor(tmp, tmp2, CV_BGR2YCrCb);
	tmp = std::move(tmp2);

	std::vector<cv::Mat> channels;
	cv::split(image, channels);
	for (auto& c : channels) {
		cv::Mat out(c.size(), CV_8UC1);
		cv::Canny(c, out, 20, 60);
		c = out;
	}
	return cv::max(cv::max(
		channels[0], channels[1]), channels[2]);
}

std::pair<cv::Point2f,bool> findTop(const cv::Mat& mask, int points) {
	assert(mask.type() == CV_8UC1);

	std::unordered_set<int> columns;
	cv::Point2f sum(0, 0);
	int count = 0;
	for (int i = 0 ; i < mask.rows ; i++) {
		for (int j = 0 ; j < mask.cols ; j++) {
			if (mask.at<uchar>(i,j) > 0) {
				if (columns.insert(j).second) {
					sum += cv::Point2f(j, i);
					count++;
				}
			}
		}
		if (count >= points) {
			break;
		}
	}

	if (count == 0) {
		return std::make_pair(sum, false);
	}

	sum.x /= count;
	sum.y /= count;
	return std::make_pair(sum, true);
}

cv::Mat sumDerivative(const cv::Mat& image, bool x, bool y) {
	assert(image.type() == CV_8UC3);

	cv::Mat tmp(image.size(), CV_8UC3);
	cv::cvtColor(image, tmp, CV_BGR2YCrCb);

	std::vector<cv::Mat> channels;
	cv::split(tmp, channels);

	cv::Mat sum(tmp.size(), CV_16SC1, cv::Scalar(0));
	cv::Mat out(tmp.size(), CV_16SC1);

	for (auto& c : channels) {
		if (x) {
			cv::Scharr(c, out, CV_16S, 1, 0);
			cv::add(sum, cv::abs(out), sum);
		}
		if (y) {
			cv::Scharr(c, out, CV_16S, 0, 1);
			cv::add(sum, cv::abs(out), sum);
		}
	}

	cv::Mat result(tmp.size(), CV_8UC1);
	cv::normalize(sum, result, 0, 255, cv::NORM_MINMAX, CV_8UC1);

	return result;
}

float symmetry(const cv::Mat& der, float _min, float _max) {
	assert(der.type() == CV_8UC1);

	cv::Size size = der.size();

	int min = std::round(_min * 2);
	if (min < 0) min = 1;

	int max = std::round(_max * 2);
	if (max < 0) max = size.width * 2 - 2;

	int maxs = min + max;
	int64_t maxSum = 0;
	for (int s = min ; s <= max; s++) {
		int64_t sum = 0;
		for (int x0 = (s-1)/2 ; x0 >= 0  ; x0--) {
			int x1 = s - x0;
			if (x1 >= size.width) break;
			for (int y = 0 ; y < size.height ; y++) {
				sum += int64_t(der.at<ushort>(x0,y)) *
					int64_t(der.at<ushort>(x1,y));
			}
		}
		if (sum > maxSum) {
			maxSum = sum;
			maxs = s;
		}
	}
	return float(maxs) / 2;
}
