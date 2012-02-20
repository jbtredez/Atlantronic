//! @file test_trapeze.c
//! @brief Test functions of trapeze.c
//! @author Atlantronic

#include "kernel/trapeze.h"
#include <stdio.h>

#define HZ   200

int main()
{
	int32_t vmax = 65536*2.0f/HZ;
	int32_t amax = 65536*1.0f/(HZ*HZ);
	int32_t dmax = 65536*2.0f/(HZ*HZ);

	int i = 0;
	int32_t d = 0;
	int32_t v = 0;
	int32_t v_old = 0;
	int32_t a = 0;

	FILE* f = fopen("log/test_trapeze.txt", "w");
	if(f == NULL)
	{
		perror("fopen");
		return -1;
	}

	for(i = 0; i < HZ*10; i++)
	{
		v = trapeze_speed_filter(v, -10*65536 - d, amax, dmax, vmax);
		d += v;
		a = v - v_old;
		fprintf(f, "%f\t%f\t%f\t%f\n", ((float)i)/HZ, a * HZ * HZ / 65536.0f , v * HZ / 65536.0f, d / 65536.0f);
		v_old = v;
	}

	fclose(f);

	FILE* p = popen("gnuplot --persist", "w");
	if(p == NULL)
	{
		perror("popen");
		return -2;
	}

	fprintf(p, "set term x11\n");
	fprintf(p, "set mouse\n");
	fprintf(p, "set xlabel \"t\"\n");
	fprintf(p, "plot \"log/test_trapeze.txt\" using 1:2 t \"acceleration\"  with lines, \"log/test_trapeze.txt\" using 1:3 t \"vitesse\" with lines, \"log/test_trapeze.txt\" using 1:4 t \"position\" with lines\n");
	fflush(p);

	// on ne ferme pas le programme tout de suite pour permettre de zoomer par exemple
	getchar();

	fclose(p);

	return 0;
}
