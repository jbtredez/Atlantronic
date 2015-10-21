#ifndef MAIN_ROBOT_H
#define MAIN_ROBOT_H

#include "middleware/trajectory/Trajectory.h"

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
#endif

#endif
