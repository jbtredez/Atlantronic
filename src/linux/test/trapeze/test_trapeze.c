//! @file test_trapeze.c
//! @brief Test functions of trapeze.c
//! @author Jean-Baptiste Trédez

#include "kernel/trapeze.h"
#include <stdio.h>

#define HZ   200

int main()
{
	struct trapeze trapeze;

	trapeze_set(&trapeze, 1.0f/(HZ*HZ), 2.0f/HZ);

	int i = 0;
	float d = 0;
	float d2 = 0;
	float v = 0;
	float v2 = 0;
	float a = 0;

	FILE* f = fopen("log/test_trapeze.txt", "w");
	if(f == NULL)
	{
		perror("fopen");
		return -1;
	}

	for(i = 0; i < HZ*10; i++)
	{
		trapeze_apply(&trapeze, -10);
		d2 = trapeze.distance;
		v2 = (d2 - d) * HZ;
		a = (v2 - v) * HZ;
		fprintf(f, "%f\t%f\t%f\t%f\n", ((float)i)/HZ, a, v, d);
		d = d2;
		v = v2;
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
