#include "jump.hh"

#include <unistd.h>   // UNIX standard function definitions
#include <fcntl.h>    // File control definitions
#include <termios.h>  // POSIX terminal control definitions
#include <sys/ioctl.h>

#include <iostream>
#include <string>
#include <chrono>

class robot {
public:
	explicit robot(const std::string& device);
	~robot() noexcept;

	bool Avaliable() const;
	bool Jump(float t);
private:
	int fd;
	std::chrono::system_clock::time_point aval;
};

robot::robot(const std::string& device) :
		fd(open(device.c_str(), O_RDWR | O_NONBLOCK )),
		aval(std::chrono::system_clock::now()) {
	if (fd == -1) {
		std::cerr<<"Unable to open port:"
			<<device<<std::endl;
		return;
	}

	termios toptions;
	if (tcgetattr(fd, &toptions) < 0) {
		std::cerr<<"Couldn't get term attributes"
			<<std::endl;
		return;
	}
	speed_t brate = B9600;
	cfsetispeed(&toptions, brate);
	cfsetospeed(&toptions, brate);

	toptions.c_cflag &= ~PARENB;
	toptions.c_cflag &= ~CSTOPB;
	toptions.c_cflag &= ~CSIZE;
	toptions.c_cflag |= CS8;
	toptions.c_cflag &= ~CRTSCTS;

	//toptions.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset

	toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
	toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

	toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
	toptions.c_oflag &= ~OPOST; // make raw

	// see: http://unixwiz.net/techtips/termios-vmin-vtime.html
	toptions.c_cc[VMIN]  = 0;
	toptions.c_cc[VTIME] = 0;
	//toptions.c_cc[VTIME] = 20;

	tcsetattr(fd, TCSANOW, &toptions);
	if( tcsetattr(fd, TCSAFLUSH, &toptions) < 0) {
		std::cerr<<"Couldn't set term attributes"
			<<std::endl;
		return;
	}
}

robot::~robot() noexcept {
	if (fd != -1) {
		close(fd);
	}
}

bool robot::Avaliable() const {
	if (fd == -1) return false;
	return std::chrono::system_clock::now() > aval;
}

bool robot::Jump(float t) {
	if (!Avaliable()) return false;

	int dur = int(t * 1000);

	std::string data = std::to_string(dur) + "\n";
	int n = write(fd, data.c_str(), data.size());
	if (n != data.size()) return false;

	aval = std::chrono::system_clock::now() +
		std::chrono::milliseconds(dur + 800);
	return true;
}

Robot::Robot(const std::string& device)
 	: ptr(new robot(device)){ }

Robot::~Robot() noexcept {
	delete ((robot*)ptr);
}

bool Robot::Avaliable() const {
	return ((robot*)ptr)->Avaliable();
}

bool Robot::Jump(float t) {
	return ((robot*)ptr)->Jump(t);
}
