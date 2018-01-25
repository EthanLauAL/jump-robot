#include <iostream>
#include <cstdlib>
#include <vector>

#include "opencv2/opencv.hpp"

#include "align.hh"
#include "detect.hh"
#include "jump.hh"
#include "util.hh"

const cv::Size FrameSize(640, 480);
const cv::Size AreaSize(600, 300);
const cv::Size AlignedSize(300, 600);
const cv::Point DetectTopLeft(50, 150);
const cv::Point DetectTopRight(250, 150);

const double T = 0.59; //透视纵横比
const double P = 1.5; //跳跃按压时长常数

int main(int argc, char* argv[])
{
	if (argc < 2) {
		std::cerr<<"missing argument <arduino serial port>"<<std::endl;
		return -1;
	}

	cv::namedWindow("Jump", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("JumpDebug", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("JumpFrame", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("JumpShot", CV_WINDOW_AUTOSIZE);
	cv::VideoCapture cam(0);
	cam.set(CV_CAP_PROP_FRAME_WIDTH, FrameSize.width);
	cam.set(CV_CAP_PROP_FRAME_HEIGHT, FrameSize.height);

	cv::Rect startingArea(
		(FrameSize.width - AreaSize.width) / 2,
		(FrameSize.height - AreaSize.height) / 2,
		AreaSize.width,
		AreaSize.height);

	DenoicingSequence seq(11);
	AsyncFindBorder finding(startingArea);
	Robot robot(argv[1]);
	bool jump = false;
	bool lock = false;
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
				std::cout<<"record"<<std::endl;
			}
		}
		if (k == 'y') {
			jump = true;
			std::cout<<"start"<<std::endl;
		}
		if (k == 'n') {
			jump = false;
			std::cout<<"stop"<<std::endl;
		}
		if (k == 'l') {
			lock = !lock;
			std::cout<<"lock:"<<lock<<std::endl;
		}

		cv::Mat frame;
		cam.read(frame);

		if (!lock)
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

			cv::Mat fore = getForeGroundMask(aligned);
			//cv::imshow("JumpMask", fore);

			auto c = detectCharacter(aligned);
			if (c.second) {
				cv::Rect charRect = c.first;
				cv::rectangle(aligned, charRect, cv::Scalar(0,255,0));
				cv::Point charPoint = cv::Point(
					charRect.x + charRect.width / 2,
					charRect.y + charRect.height);
				double dist = 0;
				int py = -1;

				cv::Rect rect1(DetectTopLeft, blPoint(charRect));
				if (tlPoint(rect1) == DetectTopLeft) {
					cv::rectangle(aligned, rect1, cv::Scalar(0,255,0));
					auto p1f = findTop(fore(rect1), 10);
					if (p1f.second) {
						auto p1 = cv::Point(p1f.first.x, p1f.first.y) + rect1.tl();
						cv::circle(aligned, p1, 20, cv::Scalar(0,255,0));

						if (py == -1 || p1.y < py) {
							//int dx = p1.x - (charRect.x + charRect.width / 2);
							//int dy = p1.y - (charRect.y + charRect.height);
							//dist = perspectiveistance(dx, dy);
							py = p1.y;

							dist = (charPoint.y / T
								- p1.x * 2
								+ charPoint.x
								- AlignedSize.height / T / 2
								+ AlignedSize.width / 2) / std::sqrt(2);
						}
					}
				}

				cv::Rect rect2(DetectTopRight, brPoint(charRect));
				if (trPoint(rect2) == DetectTopRight) {
					cv::rectangle(aligned, rect2, cv::Scalar(0,255,0));
					auto p2f = findTop(fore(rect2), 10);
					if (p2f.second) {
						auto p2 = cv::Point(p2f.first.x, p2f.first.y) + rect2.tl();
						cv::circle(aligned, p2, 20, cv::Scalar(0,255,0));

						if (py == -1 || p2.y < py) {
							//int dx = p2.x - (charRect.x + charRect.width / 2);
							//int dy = p2.y - (charRect.y + charRect.height);
							//dist = perspectiveistance(dx, dy);
							py = p2.y;

							dist = (charPoint.y / T
								+ p2.x * 2
								- charPoint.x
								- AlignedSize.height / T / 2
								- AlignedSize.width / 2) / std::sqrt(2);
						}
					}
				}

				if (py != -1) {
					seq.Push(dist);
				}
				auto stat = seq.Get(AlignedSize.width * 0.02);
				if (stat.count >= 7 && jump && robot.Avaliable()) {
					float dur = stat.mean / AlignedSize.width * P;
					std::cout<<"Jump:"<<dur<<std::endl;
					cv::imshow("JumpShot", aligned);
					robot.Jump(dur);
					seq.Reset();
				}
			}

			cv::imshow("Jump", aligned);
		}
	}

	finding.Wait();
	return 0;
}
