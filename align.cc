#include "align.hh"

#include <algorithm>
#include <thread>

#include "opencv2/opencv.hpp"

class maxFloat {
private:
	bool first;
	float max;
	int idx;
public:
	maxFloat();
	void compare(float v, int i);
	int operator()() const;
};

maxFloat::maxFloat() : first(true) {}

void maxFloat::compare(float v, int i) {
	if (first || v > max) {
		max = v;
		idx = i;
	}
	first = false;
}

int maxFloat::operator()() const {
	return idx;
}

float euclidDistance(cv::Point2f p1, cv::Point2f p2) {
	float dx = p1.x - p2.x;
	float dy = p1.y - p2.y;
	return std::sqrt(dx * dx + dy * dy);
}

grabCutEvaler::grabCutEvaler(const cv::Rect& startingArea) :
 	startingArea(startingArea) { }

cv::Mat grabCutEvaler::eval(const cv::Mat& image) {
	if (mask.total() == 0) {
		mask = cv::Mat(image.size(), CV_8UC1, cv::GC_PR_BGD);
		cv::rectangle(mask, startingArea, cv::GC_PR_FGD, CV_FILLED);
		cv::grabCut(image, mask, cv::Rect(),
			bgModel, fgModel, 1, cv::GC_INIT_WITH_MASK);
	} else {
		cv::grabCut(image, mask, cv::Rect(),
			bgModel, fgModel, 1, cv::GC_EVAL);
	}

	cv::Mat maskBinay = mask.clone();
	for(int i = 0; i < maskBinay.rows; i++) {
		for(int j = 0; j < maskBinay.cols; j++) {
			auto& p = maskBinay.at<unsigned char>(i, j);
			switch(p) {
			case cv::GC_FGD:
			case cv::GC_PR_FGD:
				p = 255;
				break;
			case cv::GC_BGD:
			case cv::GC_PR_BGD:
				p = 0;
				break;
			default:
				assert(false);
			}
		}
	}
	return maskBinay;
}

int biggestContourIndex(std::vector<std::vector<cv::Point>> contours) {
	int maxSize = -1;
	int maxSizeIdx = -1;
	for (int i = 0 ; i < contours.size() ; i++) {
		if (int(contours[i].size()) > maxSize) {
			maxSize = contours[i].size();
			maxSizeIdx = i;
		}
	}
	return maxSizeIdx;
}

cv::Vec3f lineVector2Equation(cv::Vec4f line) {
	double a = -line[1];
	double b = line[0];
	double c = line[2] * (line[1] + line[3]) -
		(line[0] + line[2]) * line[3];
	return cv::Vec3f(a, b, c);
}

//两条直线的交点
cv::Point2f crossPoint(cv::Vec4f line0, cv::Vec4f line1) {
	auto e0 = lineVector2Equation(line0);
	auto e1 = lineVector2Equation(line1);
	double d = e0[0] * e1[1] - e1[0] * e0[1];
	double x = (e0[1] * e1[2] - e1[1] * e0[2]) / d;
	double y = (e1[0] * e0[2] - e0[0] * e1[2]) / d;
	return cv::Point2f(x, y);
}

std::vector<cv::Point2f> findBorder(grabCutEvaler& evaler,
	const cv::Mat& frame, cv::Mat& debug) {
	frame.copyTo(debug);

	auto areaMask = evaler.eval(frame);

	//寻找轮廓
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(areaMask, contours,
		CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	//选定最大的轮廓
	int contourIdx = biggestContourIndex(contours);
	if (contourIdx == -1) {
		return std::vector<cv::Point2f>();
	}
	cv::drawContours(debug, contours,contourIdx, cv::Scalar(0,0,255),2);
	auto contour = contours[contourIdx];

	//找轮廓接近多边形的顶点
	std::vector<cv::Point> poly;
	approxPolyDP(contour, poly,
		(frame.rows + frame.cols) / 12, true);
	if (poly.size() != 4) {
		return std::vector<cv::Point2f>();
	}

	//把轮廓边缘按定点切分成4个边
	std::vector<std::vector<cv::Point>> vertexes(4);
	int idx = 0;
	for (auto p : contour) {
		for (auto p2 : poly) {
			if (p == p2) {
				idx = (idx + 1) % 4;
				break;
			}
		}
		vertexes[idx].push_back(p);
	}
	if (idx != 0) {
		return std::vector<cv::Point2f>();
	}

	//每个边用直线拟合，
	std::vector<cv::Vec4f> lines(4);
	for (int i=0 ; i<4 ; i++)
	{
		std::vector<cv::Point2f> vertexPoints;

		int size = vertexes[i].size();
		for (int j = size * 1 / 4 ; j < size * 3 / 4 ; j++) {
			auto& p = vertexes[i][j];
			vertexPoints.push_back(cv::Point2f(p.x, p.y));
		}

		cv::fitLine(vertexPoints, lines[i],
			CV_DIST_L2, 0, 0, 0.01);
	}

	//每两个边的交点
	std::vector<cv::Point2f> polyFloat(4);
	maxFloat lt, lb, rb, rt;
	for (int i=0 ; i<4 ; i++) {
		auto p = crossPoint(lines[i], lines[(i+1) % 4]);
		polyFloat[i] = cv::Point2f(p.x, p.y);

		lt.compare(- p.x + p.y, i);
		lb.compare(p.x + p.y, i);
		rb.compare(p.x - p.y, i);
		rt.compare(- p.x - p.y, i);
	}
	//重新排序
	polyFloat = std::vector<cv::Point2f>({
		polyFloat[lt()],
		polyFloat[lb()],
		polyFloat[rb()],
		polyFloat[rt()]
	});

	//检查各边长度，确定找到的区域较大可能是目标区域。
	for (int i=0 ; i<4 ; i++) {
		float dist = euclidDistance(polyFloat[i],
			polyFloat[(i+1)%4]);
		if (dist < (frame.rows + frame.cols) / 12) {
			return std::vector<cv::Point2f>();
		}
	}

	//辅助线：轮廓4角
	std::vector<cv::Point> polyInt(4);
	for (int i=0 ; i<4 ; i++) {
		auto& p = polyFloat[i];
		polyInt[i] = cv::Point(p.x, p.y);
	}
	cv::polylines(debug, polyInt, true, cv::Scalar(0,255,0), 2);

	return polyFloat;
}

cv::Mat alignTransform(const cv::Mat& image,
	cv::Size dstSize,
	const std::vector<cv::Point2f>& dstPoly) {
	std::vector<cv::Point2f> alignedPoints(4);
	alignedPoints[0] = cv::Point2f(0, 0);
	alignedPoints[1] = cv::Point2f(0, dstSize.height);
	alignedPoints[2] = cv::Point2f(dstSize.width, dstSize.height);
	alignedPoints[3] = cv::Point2f(dstSize.width, 0);
	auto transform = cv::getPerspectiveTransform(
		alignedPoints, dstPoly);

	cv::Mat dst;
	cv::warpPerspective(image, dst, transform.inv(), dstSize);
	return dst;
}

AsyncFindBorder::AsyncFindBorder(cv::Rect rect) :
	startingArea(rect), running(false), thd(nullptr), evaler(nullptr)
{ }

void AsyncFindBorder::Push(const cv::Mat& frame)
{
	std::lock_guard<std::mutex> lock(mtx);
	if (running) { return; }

	running = true;
	processingFrame = frame.clone();

	if (thd != nullptr) {
		thd->join();
	}
	if (evaler == nullptr) {
		evaler.reset(new grabCutEvaler(startingArea));
	}
	thd.reset(new std::thread([&]{
		eval();
	}));
}

void AsyncFindBorder::eval() {
	cv::Mat tmpDebug;
	auto tmpPoly = findBorder(
		*evaler, processingFrame, tmpDebug);

	std::lock_guard<std::mutex> lock(mtx);
	running = false;
	poly = std::move(tmpPoly);
	if (tmpPoly.size() == 0) {
		evaler.reset(nullptr);
	}
	debugImage = tmpDebug;
}

std::vector<cv::Point2f> AsyncFindBorder::GetPoly() const
{
	std::lock_guard<std::mutex> lock(mtx);
	if (smoothPoly.size() == 0) {
		smoothPoly = poly;
	} else if (poly.size() > 0) {
		for (int i = 0; i < smoothPoly.size() ; i++) {
			smoothPoly[i] = smoothPoly[i] * 0.8 + poly[i] * 0.2;
		}
	}
	return smoothPoly;
}

cv::Mat AsyncFindBorder::GetDebugImage() const
{
	std::lock_guard<std::mutex> lock(mtx);
	return debugImage.clone();
}

void AsyncFindBorder::Wait() const
{
	if (thd == nullptr) { return; }
	thd->join();
}
