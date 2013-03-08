#ifndef LOCATION_H
#define LOCATION_H

//! @file location.h
//! @brief Location
//! @author Atlantronic

#include <stdint.h>
#include "kernel/math/kinematics.h"
#include "kernel/vect_pos.h"

struct fx_vect_pos location_get_position();

struct kinematics location_get_kinematics();

#endif