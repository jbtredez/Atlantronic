#ifndef TRAPEZE_H
#define TRAPEZE_H

//! @file trapeze.h
//! @brief Trapezoidal speed
//! @author Atlantronic

struct trapeze
{
	float a_max;
	float v_max;

	float distance;
	float v;
};

void trapeze_apply(struct trapeze* t, float distance);

void trapeze_set(struct trapeze* t, float a_max, float v_max);

void trapeze_reset(struct trapeze* t);

#endif
