#ifndef ODOMETRY_H
#define ODOMETRY_H

//! @file odometry.h
//! @brief Odometry
//! @author Jean-Baptiste Trédez

#include <stdint.h>
#include "vect_pos.h"

void odometry_update();

struct vect_pos odometry_get_position();

float odometry_get_speed_curv_abs();

float odometry_get_speed_rot();

#endif
