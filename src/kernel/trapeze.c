//! @file trapeze.c
//! @brief Trapezoidal speed
//! @author Atlantronic

#include "kernel/trapeze.h"
#include <math.h>

void trapeze_apply(struct trapeze* t, float s)
{
	// abscisse curviligne qui reste à faire
	float ds = s - t->s;
	float a_max = t->a_max;
	float v_max = t->v_max;
	float v = t->v;

	// saturation de v_max pour la rampe de décélération
//	float v_max_stop = sqrtf( 2 * fabs(ds) * a_max);
	float v_max_stop = sqrtf(a_max*a_max/4 + 2*fabs(ds)*a_max) - a_max/2;

	if(v_max_stop < v_max)
	{
		v_max = v_max_stop;
	}

	// saturation de v_max à cause de la saturation en accélération
	float v_abs = fabsf(v);
	if( v_abs < v_max )
	{
		if( v_max - v_abs > a_max )
		{
			v_max = v_abs + a_max;
		}
	}
	else
	{
		if( v_abs - v_max > a_max )
		{
			v_max = v_abs - a_max;
		}
	}

	float s_filtre;
	// saturation à cause de la position (pour ne pas dépasser)
	if(ds > v_max)
	{
		s_filtre = t->s + v_max;
		v = v_max;
	}
	else if( ds < - v_max)
	{
		s_filtre = t->s - v_max;
		v = -v_max;
	}
	else
	{
		s_filtre = s;
		v = ds;
	}

	t->v = v;
	t->s = s_filtre;
}

void trapeze_reset(struct trapeze* t, float s, float v)
{
	t->s = s;
	t->v = v;
}
