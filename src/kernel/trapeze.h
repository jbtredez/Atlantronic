#ifndef TRAPEZE_H
#define TRAPEZE_H

//! @file trapeze.h
//! @brief Trapezoidal speed filter
//! @author Atlantronic

#include <stdint.h>

int32_t trapeze_speed_filter(int32_t v, int32_t ds, int32_t amax, int32_t dmax, int32_t vmax);

#endif
