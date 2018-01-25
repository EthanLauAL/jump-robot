#include "detect.hh"

#include <mutex>

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

cv::Mat getForeGroundMask(const cv::Mat& image) {
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

	cv::Point2f sum(0, 0);
	int count = 0;
	for (int i = 0 ; i < mask.rows ; i++) {
		for (int j = 0 ; j < mask.cols ; j++) {
			if (mask.at<uchar>(i,j) > 0) {
				sum += cv::Point2f(j, i);
				count++;
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
