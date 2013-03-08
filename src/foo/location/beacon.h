#ifndef BEACON_H
#define BEACON_H

//! @file beacon.h
//! @brief Beacon
//! @author Atlantronic

#include <stdint.h>
#include "kernel/vect_pos.h"

void beacon_update();

struct fx_vect_pos beacon_get_position();

#endif
