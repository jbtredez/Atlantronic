#ifndef ODOMETRY_H
#define ODOMETRY_H

//! @file odometry.h
//! @brief Odometry
//! @author Atlantronic

#include <stdint.h>
#include "kernel/math/kinematics.h"

void odometry_update();

void odometry_set_position(const int32_t x, const int32_t y, const int32_t alpha);

struct kinematics odometry_get_kinematics();

#endif
