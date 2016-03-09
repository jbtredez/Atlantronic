#ifndef MAIN_ROBOT_H
#define MAIN_ROBOT_H

#include "middleware/trajectory/Trajectory.h"
#include "kernel/driver/DynamixelManager.h"
#include "kernel/PwmMotor.h"

enum hokuyo_id
{
	HOKUYO1 = 0,
	HOKUYO2,
	HOKUYO_MAX,
};

#ifndef LINUX
extern Location location;
extern Motion motion;
extern Trajectory trajectory;
extern DynamixelManager ax12;
extern Dynamixel leftWing;
extern Dynamixel rightWing;
extern Dynamixel leftCarpet;
extern Dynamixel rightCarpet;
extern Dynamixel lowFinger;
extern Dynamixel highFinger;
extern Dynamixel rightFinger;
extern Dynamixel leftFinger;
#endif

#endif
