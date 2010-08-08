#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

//! @file robot_state.h
//! @brief State of the robot
//! @author Jean-Baptiste Trédez

#include <stdint.h>
#include "vect_pos.h"

void robot_state_update_odometry();

struct vect_pos robot_state_get_position();

#endif
