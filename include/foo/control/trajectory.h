#ifndef TRAJECTORY_H
#define TRAJECTORY_H

//! @file trajectory.h
//! @brief Generation de trajectoire
//! @author Atlantronic

#include "control/control.h"
#include "location/location.h"

enum trajectory_usb_cmd_type
{
	TRAJECTORY_STRAIGHT,
	TRAJECTORY_STRAIGHT_TO_WALL,
	TRAJECTORY_ROTATE,
	TRAJECTORY_ROTATE_TO,
	TRAJECTORY_GOTO
};

struct trajectory_cmd_arg
{
	uint32_t type;
	float x;
	float y;
	float alpha;
	float dist;
	uint32_t way;
};

void trajectory_straight(float dist);

void trajectory_straight_to_wall(float dist);

void trajectory_rotate(float angle);

void trajectory_rotate_to(float angle);

void trajectory_goto_near(float x, float y, float dist, enum trajectory_way way);

#endif
