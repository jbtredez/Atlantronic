#ifndef LOCATION_H
#define LOCATION_H

//! @file location.h
//! @brief Location
//! @author Atlantronic

#include <stdint.h>
#include "kernel/math/vect_plan.h"
#include "kernel/control/kinematics.h"

#ifndef WEAK_LOCATION
#define WEAK_LOCATION __attribute__((weak, alias("nop_function") ))
#endif

void location_update(double voie_inv, Kinematics* kinematics_mes, float dt);

VectPlan location_get_position() WEAK_LOCATION;

VectPlan location_get_speed();

void location_set_position(VectPlan pos);

#endif
