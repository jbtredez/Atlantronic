#ifndef LOCATION_H
#define LOCATION_H

//! @file location.h
//! @brief Location
//! @author Atlantronic

#include <stdint.h>
#include "kernel/vect_pos.h"
#include "location/odometry.h"
#include "location/beacon.h"

void location_update();

struct kinematics location_get_kinematics();

struct fx_vect_pos location_get_position();

void location_set_position(int32_t x, int32_t y, int32_t alpha);

#endif