#ifndef LOCATION_H
#define LOCATION_H

//! @file location.h
//! @brief Location
//! @author Atlantronic

#include <stdint.h>
#include "kernel/math/vect_plan.h"
#include "kernel/control/kinematics.h"

void location_update(Kinematics* kinematics_mes, float dt);

VectPlan location_get_position();

VectPlan location_get_speed();

void location_set_position(VectPlan pos);

#endif
