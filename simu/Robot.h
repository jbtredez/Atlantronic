#ifndef ROBOT_H
#define ROBOT_H

#include "ArmCm3.h"

class Robot
{
public:
	Robot();
	~Robot();

private:
	ArmCm3 cpu;
};


#endif
