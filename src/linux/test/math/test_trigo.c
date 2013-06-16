//! @file test_trigo.c
//! @brief Test trigo functions
//! @author Atlantronic

#include "kernel/math/trigo.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int error_count;

void test_fx_sin()
{
	int32_t val;
	int32_t max_err = 0;
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
}

void test_fx_cos()
{
	int32_t val;
	int32_t max_err = 0;
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
}

void test_fx_atan2()
{
	int32_t val;
	int32_t max_err = 0;
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
}

void test_fx_acos()
{
	int32_t val;
	int32_t max_err = 0;
	for(val = -(1<<16); val < (1 << 16) ; val ++)
	{
		int32_t alpha = acos(val/ ((double)(1 << 16))) * (1 << 26)/(2*M_PI);
		int32_t fxalpha = fx_acos(val);
		int32_t err = abs(alpha - fxalpha);

		if(err > max_err)
		{
			max_err = err;
		}
	}

	printf("fx_acos - erreur max : %d (%g) : ", max_err, max_err * (2*M_PI)/ ((float)(1 << 26)));
	if(max_err < 9.5e-5 * (1 << 26)/(2*M_PI))
	{
		printf("OK\n");
	}
	else
	{
		error_count++;
		printf("KO <-----\n");
	}
}

void test_fx_sqrt()
{
	int32_t val;
	int32_t max_err = 0;
	for(val = 0; val < (1 << 26) ; val ++)
	{
		int32_t fx = fx_sqrt(val);
		int32_t fx_real = sqrt((double)val / 65536.0f)*65536;
		int32_t err = abs(fx - fx_real);
		if(err > max_err)
		{
			max_err = err;
		}
	}

	printf("fx_sqrt - erreur max : %d (%g) : ", max_err, max_err / ((float)(1 << 30)));
	if(max_err < 1e-9 * (1 << 30))
	{
		printf("OK\n");
	}
	else
	{
		error_count++;
		printf("KO <-----\n");
	}
}

int main(int argc, char** argv)
{
	error_count = 0;

	int test_id = 0;
	if( argc > 1)
	{
		test_id = atoi(argv[1]);
	}

	if( !test_id || test_id == 1 )
	{
		test_fx_sin();
	}

	if( !test_id || test_id == 2 )
	{
		test_fx_cos();
	}

	if( !test_id || test_id == 3 )
	{
		test_fx_atan2();
	}

	if( !test_id || test_id == 4 )
	{
		test_fx_acos();
	}

	if( !test_id || test_id == 5 )
	{
		test_fx_sqrt();
	}

	return error_count;
}
