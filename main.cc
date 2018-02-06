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
const cv::Point DetectTopLeft(30, 150);
const cv::Point DetectTopRight(270, 150);

const int TestFrames = 29;
const int TestConfidence = 15;

const double T = 0.59; //透视纵横比
const double OffsetScale = 2.0;

int OffsetTL = 0; //对齐偏移-左上
int OffsetTR = 0; //对齐偏移-
double P = 1.40; //跳跃按压时长乘数

int _main(int argc, char* argv[])
{
	std::string dev;
	if (argc < 2) {
		std::cerr<<"missing argument <arduino serial port>"<<std::endl;
	} else {
		dev = argv[1];
	}

	cv::namedWindow("JumpDebug");
	cv::moveWindow("JumpDebug", 0, 0);

	cv::namedWindow("JumpFrame");
	cv::moveWindow("JumpFrame", 0, 300);

	cv::namedWindow("Jump");
	cv::moveWindow("Jump", 340, 0);

	//cv::namedWindow("JumpShot", CV_WINDOW_AUTOSIZE);

	cv::VideoCapture cam(0);
	cam.set(CV_CAP_PROP_FRAME_WIDTH, FrameSize.width);
	cam.set(CV_CAP_PROP_FRAME_HEIGHT, FrameSize.height);

	cv::Rect startingArea(
		(FrameSize.width - AreaSize.width) / 2,
		(FrameSize.height - AreaSize.height) / 2,
		AreaSize.width,
		AreaSize.height);

	DenoicingSequence seq(TestFrames);
	AsyncFindBorder finding(startingArea);
	Robot robot(dev);
	bool jump = false;
	bool lock = false;
	std::unique_ptr<cv::VideoWriter> videoWriter(nullptr);

	while(true)
	{
		auto k = cv::waitKey(1);
		switch (k) {
			case 27:
				if (videoWriter == nullptr) {
					 goto final;
				} else {
					videoWriter.reset(nullptr);
				}
				break;
			case ' ':
				jump = !jump;
				std::cout<<"jump:"<<jump<<std::endl;
				break;
			case '-':
				P -= 0.05;
				std::cout<<"P:"<<P<<std::endl;
				break;
			case '=':
				P += 0.05;
				std::cout<<"P:"<<P<<std::endl;
				break;
			case 'l':
				lock = !lock;
				std::cout<<"lock:"<<lock<<std::endl;
				break;
			case 'q':
				OffsetTL++;
				break;
			case 's':
				OffsetTL--;
				break;
			case 'w':
				OffsetTR++;
				break;
			case 'a':
				OffsetTR--;
				break;
			case 'r':
				if (videoWriter == nullptr) {
					videoWriter.reset(new cv::VideoWriter(
						"output.mp4",
						CV_FOURCC('H','E','V','C'),
						25,
						AlignedSize));
					std::cout<<"record"<<std::endl;
				}
				break;
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

		cv::imshow("JumpFrame", resizeImageRatio(frame, 0.5));
		if (debugImage.total() > 0) {
			cv::imshow("JumpDebug", resizeImageRatio(debugImage, 0.5));
		}
		if (aligned.total() == 0) { continue; }

		if (videoWriter != nullptr) {
			videoWriter->write(aligned);
			cv::putText(aligned, "recording",
				cv::Point(10,10), cv::FONT_HERSHEY_SIMPLEX,
				0.5, cv::Scalar(0,0,255), 2);
		}

		//cv::Mat derivative = sumDerivative(aligned, 1, 0);
		//cv::imshow("Derivative", derivative);

		cv::Mat border = getBorder(aligned);
		//cv::imshow("JumpMask", fore);

		cv::Point basePoint(AlignedSize.width / 2,
			AlignedSize.height / 2);
		basePoint.y -= (OffsetTR + OffsetTL) * OffsetScale * T;
		basePoint.x += (OffsetTR - OffsetTL) * OffsetScale;

		auto c = detectCharacter(aligned);
		if (c.second) {
			cv::Rect charRect = c.first;
			int d = charRect.width / 5;
			charRect.x += d;
			charRect.width -= 2 * d;

			cv::rectangle(aligned, charRect, cv::Scalar(0,0,255), 3);
			cv::Point charPoint = cv::Point(
				charRect.x + charRect.width / 2,
				charRect.y + charRect.height);
			double dist = 0;
			cv::Point target;
			int py = -1;

			cv::Rect rect1(DetectTopLeft, blPoint(charRect));
			if (tlPoint(rect1) == DetectTopLeft) {
				//cv::rectangle(aligned, rect1, cv::Scalar(0,255,0));
				auto p1f = findTop(border(rect1), 20);
				if (p1f.second) {
					auto p1 = cv::Point(p1f.first.x, p1f.first.y) + rect1.tl();
					int p1y = p1.y;
					p1.y = basePoint.y - (basePoint.x - p1.x) * T;
					//p1.y = charPoint.y - (charPoint.x - p1.x) * T;

					if (py == -1 || p1y < py) {
						py = p1y;
						target = p1;
						dist = ((charPoint.y - p1.y) / T
							+ (charPoint.x - p1.x))/ std::sqrt(2);
					}
				}
			}

			cv::Rect rect2(DetectTopRight, brPoint(charRect));
			if (trPoint(rect2) == DetectTopRight) {
				//cv::rectangle(aligned, rect2, cv::Scalar(0,255,0));
				auto p2f = findTop(border(rect2), 20);
				if (p2f.second) {
					auto p2 = cv::Point(p2f.first.x, p2f.first.y) + rect2.tl();
					int p2y = p2.y;
					p2.y = basePoint.y - (p2.x - basePoint.x) * T;
					//p2.y = charPoint.y - (p2.x - charPoint.x) * T;

					if (py == -1 || p2y < py) {
						py = p2y;
						target = p2;
						dist = ((charPoint.y - p2.y) / T
							+ (p2.x - charPoint.x))/ std::sqrt(2);
					}
				}
			}

			if (py != -1) {
				seq.Push(dist);
				cv::circle(aligned, target, 5, cv::Scalar(0,0,255), 3);
			}
			auto stat = seq.Get(AlignedSize.width * 0.01);
			if (jump && stat.count >= TestConfidence && robot.Avaliable()) {
				//seq.Debug();
				float dur = stat.mean / AlignedSize.width * P;
				std::cout<<"Jump:"<<dur<<" dist:"<<stat.mean
					<<" err:"<<stat.standardDeviation/stat.mean<<std::endl;
				//cv::imshow("JumpShot", aligned);
				robot.Jump(dur);
				seq.Reset();
			}
		}

		if (!jump) {
			cv::Point tl,tr;

			tl.x = DetectTopLeft.x;
			tl.y = basePoint.y - (basePoint.x - tl.x) * T;
			cv::line(aligned, basePoint, tl, cv::Scalar(0,255,0));

			tr.x = DetectTopRight.x;
			tr.y = basePoint.y - (tr.x - basePoint.x) * T;
			cv::line(aligned, basePoint, tr, cv::Scalar(0,255,0));

			cv::rectangle(aligned,
				cv::Rect(DetectTopLeft,
					cv::Point(DetectTopRight.x, basePoint.y)),
				cv::Scalar(0,255,0) );
		}

		cv::imshow("Jump", aligned);
	}
	final:

	finding.Wait();
	return 0;
}

int main(int argc, char* argv[]) {
	try {
		return _main(argc, argv);
	} catch(const std::exception& e) {
		std::cerr<<e.what()<<std::endl;
		return 1;
	}
}
