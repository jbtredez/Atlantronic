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

	float* x_real = new float[total_size];
	float* y_real = new float[total_size];
	float* x = new float[total_size];
	float* y = new float[total_size];
	float* w = new float[total_size];
	vect2* points = new vect2[total_size];
	vect2* reg = new vect2[total_size];
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
			points[id].x = x_real[id];
			points[id].y = (y_real[id] + epsilon);
			w[id] = 1;
		}
	}

	int reg_size = regression_poly(points, total_size, (int)ecart, reg, total_size);

	printf("Il y a %d segments\n", reg_size - 1);

	if( reg_size < 2)
	{
		return 0;
	}

	float* a_reg = new float[reg_size-1];
	float* b_reg = new float[reg_size-1];

	int seg = 0;
	int a = 0;
	for(i = 0; i<total_size; i++)
	{
		if( points[i].x == reg[seg+1].x && points[i].y == reg[seg+1].y)
		{
			int err = regression_linear(x+a, y+a, w, i-a, a_reg + seg, b_reg + seg);
			if(err)
			{
				printf("error - regression_linear : %d\n", err);
				return 0;
			}

			printf("y = %g x + %g\n", a_reg[seg], b_reg[seg]);
			a = i;
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

	for(i = 0; i < reg_size; i++)
	{
		fprintf(p, "%f %f\n", reg[i].x, reg[i].y);
	}
	fprintf(p, "e\n");

	for(i = 0; i < reg_size - 1; i++)
	{
		fprintf(p, "%f %f\n", reg[i].x, a_reg[i] * reg[i].x + b_reg[i]);
		fprintf(p, "%f %f\n", reg[i+1].x, a_reg[i] * reg[i+1].x + b_reg[i]);
	}
	fprintf(p, "e\n");

	fflush(p);

	// on ne ferme pas le programme tout de suite pour permettre de zoomer par exemple
	getchar();

	delete [] x_real;
	delete [] y_real;
	delete [] x;
	delete [] y;
	delete [] w;
	delete [] points;
	delete [] reg;
	delete [] a_reg;
	delete [] b_reg;

	fclose(p);

	return 0;
}
