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

struct vect_pos location_get_position();

void location_set_position(float x, float y, float alpha);

float location_get_speed_curv_abs();

float location_get_speed_rot();

#endif
