#ifndef TRAPEZE_H
#define TRAPEZE_H

//! @file trapeze.h
//! @brief Trapezoidal speed
//! @author Atlantronic

#include <stdint.h>

struct trapeze
{
	int32_t a_max;
	int32_t d_max;
	int32_t v_max;

	int32_t s;
	int32_t v;
};

void trapeze_apply(struct trapeze* t, int32_t s);

void trapeze_reset(struct trapeze* t, int32_t s, int32_t v);

#endif
