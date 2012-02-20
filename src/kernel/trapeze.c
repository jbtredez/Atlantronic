//! @file trapeze.c
//! @brief Trapezoidal speed filter
//! @author Atlantronic

#include "kernel/trapeze.h"
#include <math.h>
#include <stdlib.h>

//! @param v vitesse actuelle
//! @param ds abscisse curviligne qui reste à faire
int32_t trapeze_speed_filter(int32_t v, int32_t ds, int32_t amax, int32_t dmax, int32_t vmax)
{
	// saturation de vmax pour la rampe de décélération
	// formule theorique : vmax_stop = sqrt( 2 * fabs(ds) * d_max);
	// formule modifiée pour limiter le pic de décélération à la fin
	int32_t half_dmax = dmax >> 1;
	int32_t vmax_stop = sqrtf( (int64_t)half_dmax * (int64_t)half_dmax + ( ((int64_t)abs(ds) * (int64_t)dmax) << 1)) - half_dmax;

	if(vmax_stop < vmax)
	{
		vmax = vmax_stop;
	}

	// saturation de vmax à cause de la saturation en accélération / décélération
	int32_t vabs = abs(v);
	if( vabs < vmax )
	{
		int32_t vm = vabs + amax;
		if( vmax > vm )
		{
			vmax = vm;
		}
	}
	else
	{
		int32_t vm = vabs - dmax;
		if( vm  > vmax )
		{
			vmax = vm;
		}
	}

	// saturation à cause de la position (pour ne pas dépasser)
	if(ds > vmax)
	{
		v = vmax;
	}
	else if( ds < - vmax)
	{
		v = -vmax;
	}
	else
	{
		v = ds;
	}

	return v;
}