//! @file trapeze.c
//! @brief Trapezoidal speed
//! @author Atlantronic

#include "kernel/trapeze.h"
#include <math.h>
#include <stdlib.h>

void trapeze_apply(struct trapeze* t, int32_t s)
{
	// abscisse curviligne qui reste à faire
	int32_t ds = s - t->s;
	int32_t v_max = t->v_max;

	// saturation de v_max pour la rampe de décélération
	// formule theorique : v_max_stop = sqrt( 2 * fabs(ds) * d_max);
	// formule modifiée pour limiter le pic de décélération à la fin
	int32_t half_d_max = t->d_max >> 1;
	int32_t v_max_stop = sqrtf( (int64_t)half_d_max * (int64_t)half_d_max + ( ((int64_t)abs(ds) * (int64_t)t->d_max) << 1)) - half_d_max;

	if(v_max_stop < v_max)
	{
		v_max = v_max_stop;
	}

	// saturation de v_max à cause de la saturation en accélération / décélération
	int32_t v_abs = abs(t->v);
	if( v_abs < v_max )
	{
		int32_t vm = v_abs + t->a_max;
		if( v_max > vm )
		{
			v_max = vm;
		}
	}
	else
	{
		int32_t vm = v_abs - t->d_max;
		if( vm  > v_max )
		{
			v_max = vm;
		}
	}

	// saturation à cause de la position (pour ne pas dépasser)
	if(ds > v_max)
	{
		t->s += v_max;
		t->v = v_max;
	}
	else if( ds < - v_max)
	{
		t->s -= v_max;
		t->v = -v_max;
	}
	else
	{
		t->s = s;
		t->v = ds;
	}
}

void trapeze_reset(struct trapeze* t, int32_t s, int32_t v)
{
	t->s = s;
	t->v = v;
}
