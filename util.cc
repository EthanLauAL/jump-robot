#include "util.hh"

cv::Mat resizeImageRatio(const cv::Mat& m, float ratio) {
	cv::Mat n;
	cv::resize(m, n, cv::Size(0,0), ratio, ratio);
	return n;
}

DenoicingSequence::DenoicingSequence(int size) : size(size) {}

double median(const std::vector<double> values) {
	std::vector<double> tmp(values);
	auto it = tmp.begin();
	auto m = it + tmp.size() / 2;

	std::nth_element(it, m, tmp.end());
	if (tmp.size() % 2 == 1) {
		return *m;
	} else {
		auto m2 = m - 1;
		std::nth_element(it, m2, m);
		return (*m + *m2) / 2;
	}
}

Stat::Stat() : count(0),sum(0),mean(0),median(0),
	variance(0),standardDeviation(0) {}

void DenoicingSequence::Push(const double val) {
	values.push_back(val);
	while (values.size() > size) {
		values.erase(values.begin());
	}
}

Stat DenoicingSequence::Get(double errlimit) const {
	Stat stat;
	stat.median = median(values);

	std::vector<double> selected;
	double sumSelected = 0;

	for (double v : values) {
		if (std::abs(v - stat.median) < errlimit) {
			selected.push_back(v);
			sumSelected += v;
			stat.sum += v;
			stat.count++;
		}
	}
	if (stat.count > 0) {
		stat.mean = stat.sum / stat.count;
		for (double v : selected) {
			double diff = v - stat.mean;
			stat.variance += diff * diff;
		}
		stat.standardDeviation = std::sqrt(stat.variance);
	}
	return stat;
}

/*
cv::Mat DenoicingSequence::Debug() const {
	Stat s = Get(0);

}
*/

void DenoicingSequence::Reset() {
	values.clear();
}

/*
void DenoicingSequence::Debug() const {
	for (double v : values) {
		std::cout<<v<<" ";
	}
	std::cout<<std::endl;
}
*/
