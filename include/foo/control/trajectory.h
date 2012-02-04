#ifndef TRAJECTORY_H
#define TRAJECTORY_H

//! @file trajectory.h
//! @brief Generation de trajectoire
//! @author Atlantronic

#include "control/control.h"
#include "location/location.h"

enum trajectory_cmd_type
{
	TRAJECTORY_FREE,
	TRAJECTORY_STRAIGHT,
	TRAJECTORY_STRAIGHT_TO_WALL,
	TRAJECTORY_ROTATE,
	TRAJECTORY_ROTATE_TO,
	TRAJECTORY_GOTO
};

struct trajectory_cmd_arg
{
	uint32_t type;
	int32_t x;
	int32_t y;
	int32_t alpha;
	int32_t dist;
	uint32_t way;
};

void trajectory_straight(int32_t dist);

void trajectory_straight_to_wall(int32_t dist);

void trajectory_rotate(int32_t angle);

void trajectory_rotate_to(int32_t angle);

void trajectory_goto_near(int32_t x, int32_t y, int32_t dist, enum trajectory_way way);

void trajectory_free();

#endif
