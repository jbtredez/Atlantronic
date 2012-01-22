//! @file test_trapeze.c
//! @brief Test functions of trapeze.c
//! @author Atlantronic

#include "kernel/math/regression.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define NUM_PT     4

int main()
{
	FILE* p = popen("gnuplot --persist", "w");
	if(p == NULL)
	{
		perror("popen");
		return -2;
	}

	int i,j;

	float pt_x[NUM_PT] = {0   ,  100,  150, 300};
	float pt_y[NUM_PT] = {-5  ,   80,   80, -50};
	float pas[NUM_PT-1]  = {   1,  1,  1   };
	float ecart = 20.0f;

	int size[NUM_PT-1];
	float d[NUM_PT-1];

	int total_size = 0;
	for(i = 0; i < NUM_PT-1; i++)
	{
		float dx = pt_x[i+1] - pt_x[i];
		float dy = pt_y[i+1] - pt_y[i];
		d[i] = sqrt(dx*dx + dy*dy);

		size[i] = ceil(d[i]/pas[i]);
		total_size += size[i];
	}

	srand(2.3657);

	float* x_real = malloc(sizeof(float) * total_size);
	float* y_real = malloc(sizeof(float) * total_size);
	float* x = malloc(sizeof(float) * total_size);
	float* y = malloc(sizeof(float) * total_size);
	float* w = malloc(sizeof(float) * total_size);
	struct fx_vect_pos* points = malloc(sizeof(struct fx_vect_pos) * total_size);
	float epsilon;

	int id = 0;
	for(i = 0; i < NUM_PT-1 ; i++)
	{
		for(j = 0; j < size[i]; j++, id++)
		{
			epsilon = ecart * rand() / (RAND_MAX + 1.0) - 0.5f * ecart;
			x_real[id] = pt_x[i] + j*pas[i] * (pt_x[i+1] - pt_x[i])/d[i];
			y_real[id] = pt_y[i] + j*pas[i] * (pt_y[i+1] - pt_y[i])/d[i];
			x[id] = x_real[id];
			y[id] = y_real[id] + epsilon;
			points[id].x = x_real[id] * 65536;
			points[id].y = (y_real[id] + epsilon) * 65536;
			w[id] = 1;
		}
	}

	char* type = malloc(sizeof(char) * total_size);

	regression_poly(points, total_size, ecart, type);

	int num_seg = 0;
	for(i = 1; i<total_size; i++)
	{
		if(type[i] == 1)
		{
			num_seg++;
		}
	}
	printf("Il y a %d segments\n", num_seg);

	float* a_reg = malloc(sizeof(float) * num_seg);
	float* b_reg = malloc(sizeof(float) * num_seg);

	int a = 0;
	int seg = 0;
	for(i = 1; i<total_size; i++)
	{
		if(type[i] == 1)
		{
			int err = regression_linear(x+a, y+a, w, i-a, a_reg + seg, b_reg + seg);
			if(err)
			{
				printf("error - regression_linear : %d\n", err);
				return 0;
			}
			a = i;
			printf("y = %g x + %g\n", a_reg[seg], b_reg[seg]);
			seg++;
		}
	}

	fprintf(p, "set term x11\n");
	fprintf(p, "set mouse\n");
	fprintf(p, "set xlabel \"x\"\n");
	fprintf(p, "set ylabel \"y\"\n");
	fprintf(p, "plot \"-\" title \"pt\", \"-\" title \"real\" with lines, \"-\" title \"reg\" with lines, \"-\" title \"reg2\" with lines\n");
	for(i = 0; i < total_size; i++)
	{
		fprintf(p, "%f %f\n", x[i], y[i]);
	}
	fprintf(p, "e\n");
	for(i = 0; i < total_size; i++)
	{
		fprintf(p, "%f %f\n", x_real[i], y_real[i]);
	}
	fprintf(p, "e\n");

	for(i = 0; i < total_size; i++)
	{
		if(type[i])
		{
			fprintf(p, "%f %f\n", x[i], y[i]);
		}
	}
	fprintf(p, "e\n");

	seg = 0;
	for(i = 0; i < total_size; i++)
	{
		if(type[i] && i > 0)
		{
			seg++;
		}
		fprintf(p, "%f %f\n", x[i], a_reg[seg] * x[i] + b_reg[seg]);
	}
	fprintf(p, "e\n");

	fflush(p);

	// on ne ferme pas le programme tout de suite pour permettre de zoomer par exemple
	getchar();

	free( x_real );
	free( y_real );
	free( x );
	free( y );
	free( w );
	fclose(p);

	return 0;
}
