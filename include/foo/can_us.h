#ifndef US_H
#define US_H

//! @file us.h
//! @brief CAN - US
//! @author Atlantronic

#include <stdint.h>
#include "kernel/can/can_us.h"

uint32_t us_get_state(enum us_id id);

#endif
