#include <iostream>
#include <cstdlib>
#include <vector>

#include "opencv2/opencv.hpp"

#include "align.hh"

const cv::Size FrameSize(640, 480);
const cv::Size AreaSize(600, 300);
const cv::Size AlignedSize(300, 600);

int main(int argc, char* argv[])
{
	cv::namedWindow("Jump", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("JumpDebug", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("JumpFrame", CV_WINDOW_AUTOSIZE);
	cv::VideoCapture cam(0);
	cam.set(CV_CAP_PROP_FRAME_WIDTH, FrameSize.width);
	cam.set(CV_CAP_PROP_FRAME_HEIGHT, FrameSize.height);

	cv::Rect startingArea(
		(FrameSize.width - AreaSize.width) / 2,
		(FrameSize.height - AreaSize.height) / 2,
		AreaSize.width,
		AreaSize.height);

	AsyncFindBorder finding(startingArea);
	std::unique_ptr<cv::VideoWriter> videoWriter(nullptr);

	while(true)
	{
		auto k = cv::waitKey(1);
		if (k == 27) {
			if (videoWriter == nullptr) {
				 break;
			} else {
				videoWriter.reset(nullptr);
			}
		}
		if (k == 'r') {
			if (videoWriter == nullptr) {
				videoWriter.reset(new cv::VideoWriter(
					"output.mp4",
					CV_FOURCC('H','E','V','C'),
					25,
					AlignedSize));
			}
		}

		cv::Mat frame;
		cam.read(frame);

		finding.Push(frame);
		auto poly = finding.GetPoly();
		auto debugImage = finding.GetDebugImage();
		//cv::Mat debugImage;
		//auto poly = findBorder(frame, startingArea, debugImage);

		cv::Mat aligned;
		if (poly.size() > 0) {
			aligned = alignTransform(frame, AlignedSize, poly);
		}
		cv::rectangle(frame, startingArea, cv::Scalar(255,0,0), 2);

		cv::imshow("JumpFrame", frame);
		if (debugImage.total() > 0) {
			cv::imshow("JumpDebug", debugImage);
		}
		if (aligned.total() > 0) {
			if (videoWriter != nullptr) {
				videoWriter->write(aligned);
				cv::putText(aligned, "recording",
					cv::Point(10,10), cv::FONT_HERSHEY_SIMPLEX,
					0.5, cv::Scalar(0,0,255), 2);
			}
			cv::imshow("Jump", aligned);
		}
	}

	finding.Wait();
	return 0;
}
