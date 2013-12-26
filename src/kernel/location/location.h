#ifndef LOCATION_H
#define LOCATION_H

//! @file location.h
//! @brief Location
//! @author Atlantronic

#include <stdint.h>
#include "kernel/math/vect_plan.h"

struct location_cmd_arg
{
	float x;
	float y;
	float theta;
};

//void location_update();

//struct kinematics location_get_kinematics();

VectPlan location_get_position();

void location_set_position(VectPlan pos);

#endif
