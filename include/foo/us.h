#ifndef US_H
#define US_H

//! @file us.h
//! @brief us.h
//! @author Atlantronic

#include "kernel/us_def.h"

uint8_t us_check_collision();

uint32_t us_get_state(enum us_id id);

#endif
