#ifndef US_H
#define US_H

//! @file us.h
//! @brief us.h
//! @author Atlantronic

#include "kernel/us_def.h"

uint8_t us_check_collision();

void us_start_scan(uint8_t us_mask);

int us_get_scan_result();

#endif
