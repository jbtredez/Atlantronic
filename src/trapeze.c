//! @file trapeze.c
//! @brief Trapezoidal speed
//! @author Jean-Baptiste Trédez

#include "trapeze.h"
#include <math.h>

void trapeze_apply(struct trapeze* t, float distance)
{
	// distance qui reste à faire
	float d = distance - t->distance;
	float a_max = t->a_max;
	float v_max = t->v_max;
	float v = t->v;

	// saturation de v_max pour la rampe de décélération
//	float v_max_stop = sqrt( 2 * fabs(d) * a_max);
	float v_max_stop = sqrt(a_max*a_max/4 + 2*fabs(d)*a_max) - a_max/2;

	if(v_max_stop < v_max)
	{
		v_max = v_max_stop;
	}

	// saturation de v_max à cause de la saturation en accélération
	if( v < v_max )
	{
		if( v_max - v > a_max )
		{
			v_max = v + a_max;
		}
	}
	else
	{
		if( v - v_max > a_max )
		{
			v_max = v - a_max;
		}
	}

	float distance_filtre;
	// saturation à cause de la position (pour ne pas dépasser)
	if(d > v_max)
	{
		distance_filtre = t->distance + v_max;
		v = v_max;
	}
	else if( d < - v_max)
	{
		distance_filtre = t->distance - v_max;
		v = v_max;
	}
	else
	{
		distance_filtre = distance;
		v = d;
	}

	t->v = v;
	t->distance = distance_filtre;
}

void trapeze_set(struct trapeze* t, float a_max, float v_max)
{
	t->a_max = a_max;
	t->v_max = v_max;
}

void trapeze_reset(struct trapeze* t)
{
	t->distance = 0;
	t->v = 0;
}
