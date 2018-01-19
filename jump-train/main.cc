#include <mutex>
#include <queue>
#include <string>

#include "opencv2/opencv.hpp"

#include "find.hh"
#include "save.hh"

std::mutex mtx;
std::queue<cv::Point> click;

void onMouse(int event, int x, int y, int flags, void* ustc)
{
	if (event == 1) {
		std::lock_guard<std::mutex> lock(mtx);
		click.push(cv::Point(x, y));
	}
}

void waitKey(int& _key, cv::Point& _click) {
	while(true) {
		int key = cv::waitKey(1);
		if (key >= 0) {
			_key = key;
			_click = cv::Point(-1, -1);
			return;
		}

		std::lock_guard<std::mutex> lock(mtx);
		if (click.size() > 0) {
			_key = -1;
			_click = click.front();
			click.pop();
			return;
		}
	}
}

int main(int argc, char* argv[]) {
	if (argc < 2) {
		std::cerr<<"jump-train <video filename>"<<std::endl;
		return 1;
	}

	cv::namedWindow("trainner", CV_WINDOW_AUTOSIZE);
	cv::setMouseCallback("trainner", onMouse);

	cv::VideoCapture reader(argv[1]);
	while(true) {
		nextFrame:

		cv::Mat frame;
		int frameIndex = reader.get(CV_CAP_PROP_POS_FRAMES);
		//std::cout<<frameIndex<<std::endl;
		reader.read(frame);

		cv::Rect selected = notFound;

		while(true) {
			auto frame2 = frame.clone();
			if (selected != notFound) {
				cv::rectangle(frame2, selected,
					cv::Scalar(0, 255, 0), 2);
			}
			cv::imshow("trainner", frame2);

			int key;
			cv::Point click;
			waitKey(key, click);

			switch (key) {
			case -1: //mouse click
				selected = find(frame, click,
					cv::Size(20,50), cv::Size(30,70));
				break;
			case 27:
				if (selected != notFound) selected = notFound;
				else return 0;
				break;
			case ' ':
				if (selected != notFound) {
					save(frame, selected,
						cv::Rect(0, frame.rows/3, frame.cols, frame.rows/3),
						std::to_string(frameIndex));
				}
				goto nextFrame;
			case '.':
				goto nextFrame;
			case ',':
				reader.set(CV_CAP_PROP_POS_FRAMES, frameIndex - 1);
				goto nextFrame;
			case '>':
				reader.set(CV_CAP_PROP_POS_FRAMES, frameIndex + 10);
				goto nextFrame;
			case '<':
				if (frameIndex >= 10) {
					frameIndex -= 10;
				} else {
					frameIndex = 0;
				}
				reader.set(CV_CAP_PROP_POS_FRAMES, frameIndex);
				goto nextFrame;
			}
		}
	}
	return 0;
}
