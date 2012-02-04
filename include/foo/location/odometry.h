#ifndef ODOMETRY_H
#define ODOMETRY_H

//! @file odometry.h
//! @brief Odometry
//! @author Atlantronic

#include <stdint.h>
#include "kernel/vect_pos.h"

void odometry_update();

void odometry_set_position(const struct fx_vect_pos pos);

struct fx_vect_pos odometry_get_position();

int32_t odometry_get_speed_curv_abs();

int32_t odometry_get_speed_rot();

#endif
