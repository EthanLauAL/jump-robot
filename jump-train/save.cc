#include "save.hh"

#include <fstream>

const cv::Size SampleSize(24,24);

void saveRect(const cv::Mat& image, cv::Size size,
		const std::string& filename) {
	cv::Mat resized(size, CV_8UC3);
	cv::Mat gray(size, CV_8UC1);
	cv::resize(image, resized, resized.size(), cv::INTER_AREA);
	cv::cvtColor(resized, gray, CV_BGR2GRAY);
	imwrite(filename, gray);
}

void save(const cv::Mat& image, cv::Rect pos, cv::Rect bg, const std::string& name) {
	saveRect(image(pos), SampleSize,
		std::string("pos/") + name + ".bmp");

	float scale = float(pos.width) / float(SampleSize.width);

	cv::Rect bg1(bg.x, bg.y, pos.x - bg.x, bg.height);
	cv::Size bg1size(round(bg1.width * scale), round(bg1.height * scale));
	if (bg1size.width > SampleSize.width) {
		saveRect(image(bg1), bg1size,
		std::string("neg/") + name + "_1.bmp");
	}

	cv::Rect bg2(pos.x + pos.width, bg.y,
		bg.width - pos.width - bg1.width, bg.height);
	cv::Size bg2size(round(bg2.width * scale), round(bg2.height * scale));
	if (bg2size.width > SampleSize.width) {
		saveRect(image(bg2), bg2size,
		std::string("neg/") + name + "_2.bmp");
	}
}
