//! @file test_trapeze.c
//! @brief Test functions of trapeze.c
//! @author Atlantronic

#include "kernel/math/regression.h"
#include <stdio.h>
#include <stdlib.h>

#define SIZE   200

int main()
{
	int i;
	float ecart = 20.0f;
	float a = 5;
	float b = -1;
	float a_reg;
	float b_reg;
	float x[SIZE];
	float y[SIZE];
	float w[SIZE];
	float epsilon;

	srand(2.3657);

	for(i = 0; i < SIZE; i++)
	{
		w[i] = 1;
		x[i] = 0.1*i;
		epsilon = ecart * rand() / (RAND_MAX + 1.0) - 0.5f * ecart;
		y[i] = a * x[i] + b + epsilon;
	}

	int err = regression_linear(x, y, w, SIZE, &a_reg, &b_reg);

	if(err)
	{
		printf("error - regression_linear : %d\n", err);
		return 0;
	}

	printf("real : y = %g x + %g\n", a, b);
	printf("reg : y = %g x + %g\n", a_reg, b_reg);

	FILE* p = popen("gnuplot --persist", "w");
	if(p == NULL)
	{
		perror("popen");
		return -2;
	}

	fprintf(p, "set term x11\n");
	fprintf(p, "set mouse\n");
	fprintf(p, "set xlabel \"t\"\n");
	fprintf(p, "plot \"-\" title \"pt\", \"-\" title \"real\" with lines, \"-\" title \"reg\" with lines\n");
	for(i = 0; i < SIZE; i++)
	{
		fprintf(p, "%f %f\n", x[i], y[i]);
	}
	fprintf(p, "e\n");
	for(i = 0; i < SIZE; i++)
	{
		fprintf(p, "%f %f\n", x[i], a*x[i]+b);
	}
	fprintf(p, "e\n");
	for(i = 0; i < SIZE; i++)
	{
		fprintf(p, "%f %f\n", x[i], a_reg*x[i]+b_reg);
	}
	fprintf(p, "e\n");
	fflush(p);

	// on ne ferme pas le programme tout de suite pour permettre de zoomer par exemple
	getchar();

	fclose(p);

	return 0;
}
