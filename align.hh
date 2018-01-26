#ifndef INCLUDE_ALIGN_HH
#define INCLUDE_ALIGN_HH

#include <thread>

#include "opencv2/opencv.hpp"

std::vector<cv::Point2f> findBorder(const cv::Mat& frame, const cv::Rect startingArea, cv::Mat& debug);

cv::Mat alignTransform(const cv::Mat& image,
	cv::Size dstSize,
	const std::vector<cv::Point2f>& dstPoly);

class grabCutEvaler {
public:
	grabCutEvaler(const cv::Rect& startingArea);
	cv::Mat eval(const cv::Mat& image);
private:
	const cv::Rect startingArea;
	cv::Mat mask, bgModel, fgModel;
};

class AsyncFindBorder {
public:
	explicit AsyncFindBorder(cv::Rect rect);
	void Push(const cv::Mat& frame);
	std::vector<cv::Point2f> GetPoly() const;
	cv::Mat GetDebugImage() const;
	void Wait() const;
private:
	cv::Rect startingArea;
	mutable std::mutex mtx;
	std::unique_ptr<std::thread> thd;
	bool running;

	//input
	cv::Mat processingFrame;

	//output
	std::vector<cv::Point2f> poly;
	mutable std::vector<cv::Point2f> smoothPoly;
	cv::Mat debugImage;

	//eval
	std::unique_ptr<grabCutEvaler> evaler;
	void eval();
};

#endif