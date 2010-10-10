#ifndef BEACON_H
#define BEACON_H

//! @file beacon.h
//! @brief Beacon
//! @author Jean-Baptiste Trédez

#include <stdint.h>
#include "vect_pos.h"

void beacon_update();

struct vect_pos beacon_get_position();

#endif
