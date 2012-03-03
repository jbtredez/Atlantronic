#ifndef POLYLINE_H
#define POLYLINE_H

//! @file polyline.h
//! @brief Polyline
//! @author Atlantronic

#include <stdint.h>
#include "kernel/math/vect2.h"

struct polyline
{
	struct fx_vect2* pt;
	int16_t size;
};

#endif