#include "kernel/math/poly7.h"
#include <stdio.h>
#include <math.h>

int main()
{
	float x1 = 100;
	float y1 = 150;
	float alpha1 = 0;
	float v1 = 0;
	float w1 = 0;

	float x2 = 1000;
	float y2 = 50;
	float alpha2 = 0;
	float v2 = 0;
	float w2 = 0;

	float a[8];
	float b[8];
//	float dx = x2 - x1;
//	float dy = y2 - y1;
	float n = 300;//sqrtf( dx * dx + dy * dy);
	float u[6] = { n, n, 0, 0, 0, 0};
	poly7f_full(x1, y1, alpha1, v1, w1, x2, y2, alpha2, v2, w2, a, b, u);

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
		float x = a[0] + a[1] * t + a[2] * t * t + a[3] * t * t * t + a[4] * t * t * t * t + a[5] * t * t * t * t * t + a[6] * t * t * t * t * t * t + a[7] * t * t * t * t * t * t * t;
		float y = b[0] + b[1] * t + b[2] * t * t + b[3] * t * t * t + b[4] * t * t * t * t + b[5] * t * t * t * t * t + b[6] * t * t * t * t * t * t + b[7] * t * t * t * t * t * t * t;
		fprintf(p, "%f %f\n", x, y);
	}

	fprintf(p, "e\n");
	fflush(p);

	// on ne ferme pas le programme tout de suite pour permettre de zoomer par exemple
	getchar();

	fclose(p);

	return 0;
}
