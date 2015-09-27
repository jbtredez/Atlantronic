#ifndef POLYLINE_H
#define POLYLINE_H

//! @file polyline.h
//! @brief Polyline
//! @author Atlantronic

#include <stdint.h>
#include "kernel/math/Vect2.h"

struct polyline
{
	Vect2* pt;
	int16_t size;
};

#endif
