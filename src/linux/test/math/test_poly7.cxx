#include "kernel/math/poly7.h"
#include <stdio.h>
#include <math.h>

int main()
{
	VectPlan start(700, -700, 3.14159);
	VectPlan end(-900, 0, 1.5708);

	float a[8];
	float b[8];
	VectPlan delta = end - start;
	float n = delta.norm();
	//float u[6] = { n, n, 0, 0, 0, 0};
	float u[6] = { n, 0, 0, 0, 0, 0};
	poly7f_full(start, end, a, b, u);

	FILE* p = popen("gnuplot", "w");
	if(p == NULL)
	{
		perror("popen");
		return -2;
	}

	fprintf(p, "set term x11\n");
	fprintf(p, "set mouse\n");
	fprintf(p, "set xlabel \"t\"\n");
	fprintf(p, "plot \"-\"\n");

	int i;
	for(i = 0; i < 100; i++)
	{
		float t = i / 100.0f;
		float t2 = t*t;
		float t3 = t2 * t;
		float t4 = t3 * t;
		float t5 = t4 * t;
		float t6 = t5 * t;
		float t7 = t6 * t;
		float x = a[0] + a[1] * t + a[2] * t2 + a[3] * t3 + a[4] * t4 + a[5] * t5 + a[6] * t6 + a[7] * t7;
		float y = b[0] + b[1] * t + b[2] * t2 + b[3] * t3 + b[4] * t4 + b[5] * t5 + b[6] * t6 + b[7] * t7;
		fprintf(p, "%f %f\n", x, y);
	}

	fprintf(p, "e\n");
	fflush(p);

	// on ne ferme pas le programme tout de suite pour permettre de zoomer par exemple
	getchar();

	fclose(p);

	return 0;
}
