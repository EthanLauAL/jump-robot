#include "find.hh"

cv::Rect find(const cv::Mat& image, const cv::Point p,
		const cv::Size normalSize, const cv::Size biggestSize) {
	cv::Mat bgModel, fgModel;
	cv::Mat mask(image.size(), CV_8UC1, cv::GC_BGD);
	cv::rectangle(mask,
		cv::Rect(
			p.x - biggestSize.width / 2,
			p.y - biggestSize.height / 2,
			biggestSize.width,
			biggestSize.height),
		cv::GC_PR_BGD, CV_FILLED);
	cv::rectangle(mask,
		cv::Rect(
			p.x - normalSize.width / 2,
			p.y - normalSize.height / 2,
			normalSize.width,
			normalSize.height),
		cv::GC_PR_FGD, CV_FILLED);
	cv::grabCut(image, mask, cv::Rect(),
		bgModel, fgModel, 5, cv::GC_INIT_WITH_MASK);

	bool found = false;
	int minX, minY, maxX, maxY;

	for(int i = 0; i < mask.rows; i++) {
		for(int j = 0; j < mask.cols; j++) {
			switch(mask.at<unsigned char>(i, j)) {
			case cv::GC_FGD:
			case cv::GC_PR_FGD:
				if (!found) {
					minX = j;
					maxX = j;
					minY = i;
					maxY = i;
					found = true;
				} else {
					if (j < minX) { minX = j; }
					if (j > maxX) { maxX = j; }
					if (i < minY) { minY = i; }
					if (i > maxY) { maxY = i; }
				}
				//mask.at<unsigned char>(i, j) = 255;
			}
		}
	}
	//cv::imshow("test", mask);

	if (found) {
		int r = ((maxY - minY) - (maxX - minX)) / 2;
		if (r > 0) {
			maxX += r;
			minX -= r;
		} else {
			maxY += -r;
			minY -= -r;
		}
		return cv::Rect(cv::Point(minX, minY),
			cv::Point(maxX, maxY));
	} else {
		return notFound;
	}
}
