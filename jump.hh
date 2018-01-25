#ifndef JUMP_HH
#define JUMP_HH

#include <string>

class Robot {
public:
	explicit Robot(const std::string& device);
	Robot(const Robot&) = delete;
	Robot& operator=(const Robot&) = delete;
	~Robot() noexcept;

	bool Avaliable() const;
	bool Jump(float t);

private:
	void* ptr;
};

#endif