//! @file test_trigo.c
//! @brief Test trigo functions
//! @author Atlantronic

#include "kernel/math/trigo.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int error_count;

int main()
{
	int32_t val;
	error_count = 0;
	int32_t max_err = 0;

	// test des fx_sin
	for(val = 0; val < (1 << 26) ; val ++)
	{
		int32_t fxs = fx_sin(val + (20 << 26));
		int32_t fxsd = sin(val / ((double)(1 << 26)) * 2 * M_PI) * (1 << 30);
		int32_t err = abs(fxsd - fxs);
		if(err > max_err)
		{
			max_err = err;
		}
	}

	printf("fx_sin - erreur max : %d (%g) : ", max_err, max_err / ((float)(1 << 30)));
	if(max_err < 3e-7 * (1 << 30))
	{
		printf("OK\n");
	}
	else
	{
		error_count++;
		printf("KO <-----\n");
	}

	// test des fx_cos
	max_err = 0;
	for(val = 0; val < (1 << 26) ; val ++)
	{
		int32_t fxs = fx_cos(val + (20 << 26));
		int32_t fxsd = cos(val / ((double)(1 << 26)) * 2 * M_PI) * (1 << 30);
		int32_t err = abs(fxsd - fxs);
		if(err > max_err)
		{
			max_err = err;
		}
	}

	printf("fx_cos - erreur max : %d (%g) : ", max_err, max_err / ((float)(1 << 30)));
	if(max_err < 3e-7 * (1 << 30))
	{
		printf("OK\n");
	}
	else
	{
		error_count++;
		printf("KO <-----\n");
	}

	// test des fx_atan2
	max_err = 0;
	for(val = 0; val < (1 << 28) ; val ++)
	{
		int32_t alpha = atan2(2000, val/ ((double)(1 << 16))) * (1 << 26)/(2*M_PI);
		int32_t fxalpha = fx_atan2(2000 << 16, val);
		int32_t err = abs(alpha - fxalpha);

		if(err > max_err)
		{
			max_err = err;
		}
	}

	printf("fx_atan2 - erreur max : %d (%g) : ", max_err, max_err * (2*M_PI)/ ((float)(1 << 26)));
	if(max_err < 5e-6 * (1 << 26)/(2*M_PI))
	{
		printf("OK\n");
	}
	else
	{
		error_count++;
		printf("KO <-----\n");
	}

	return error_count;
}
