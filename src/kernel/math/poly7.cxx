#include "kernel/math/poly7.h"
#include <math.h>

void computePoly7Traj(VectPlan start, VectPlan end, float* a, float* b, float n1, float n2)
{
	float ca1 = cosf(start.theta);
	float sa1 = sinf(start.theta);

	float ca2 = cosf(end.theta);
	float sa2 = sinf(end.theta);

	float dx = end.x - start.x;
	float dy = end.y - start.y;

	a[0] = start.x;
	b[0] = start.y;

	a[1] = n1 * ca1;
	b[1] = n1 * sa1;

	a[2] = 0;
	b[2] = 0;

	a[3] = 0;
	b[3] = 0;

	a[4] = 35 * dx - 20 * n1 * ca1 - 15 * n2 * ca2;
	b[4] = 35 * dy - 20 * n1 * sa1 - 15 * n2 * sa2;

	a[5] = -84 * dx + 45 * n1 * ca1 + 39 * n2 * ca2;
	b[5] = -84 * dy + 45 * n1 * sa1 + 39 * n2 * sa2;

	a[6] = 70 * dx - 36 * n1 * ca1 - 34 * n2 * ca2;
	b[6] = 70 * dy - 36 * n1 * sa1 - 34 * n2 * sa2;

	a[7] = -20 * dx + 10 * n1 * ca1 + 10 * n2 * ca2;
	b[7] = -20 * dy + 10 * n1 * sa1 + 10 * n2 * sa2;
}

#if 0
void computePoly7Traj(VectPlan start, VectPlan end, float* a, float* b, float* u)
{
	float ca1 = cosf(start.theta);
	float sa1 = sinf(start.theta);
	float k1 = 0;//sigma1;
	float dk1 = 0;

	float ca2 = cosf(end.theta);
	float sa2 = sinf(end.theta);
	float k2 = 0;//sigma2;
	float dk2 = 0;

	float dx = end.x - start.x;
	float dy = end.y - start.y;

	a[0] = start.x;
	b[0] = start.y;

	a[1] = u[0] * ca1;
	b[1] = u[0] * sa1;

	a[2] = 0.5 * (u[2] * ca1 - u[0] * u[0] * k1 * sa1);
	b[2] = 0.5 * (u[2] * sa1 + u[0] * u[0] * k1 * ca1);

	a[3] = (1/6.0f) * (u[4] * ca1 - (u[0] * u[0] * u[0] * dk1 + 3 * u[0] * u[2]) * sa1);
	b[3] = (1/6.0f) * (u[4] * sa1 + (u[0] * u[0] * u[0] * dk1 + 3 * u[0] * u[2]) * ca1);

	a[4] = 35 * dx - (20 * u[0] + 5 * u[2] + 2/3.0f * u[4]) * ca1 + (5 * u[0] * u[0] * k1 + 2/3.0f * u[0] * u[0] * u[0] * dk1 + 2 * u[0] * u[2] * k1) * sa1
			- (15 * u[1] - 2.5f * u[3] + 1/6.0f * u[5]) * ca2 - (2.5 * u[1] * u[1] * k2 - 1 /6.0f * u[1] * u[1] * u[1] * dk2 - 0.5 * u[1] * u[3] * k2) * sa2;
	b[4] = 35 * dy - (20 * u[0] + 5 * u[2] + 2/3.0f * u[4]) * sa1 - (5 * u[0] * u[0] * k1 + 2/3.0f * u[0] * u[0] * u[0] * dk1 + 2 * u[0] * u[2] * k1) * ca1
			- (15 * u[1] - 2.5f * u[3] + 1/6.0f * u[5]) * sa2 + (2.5 * u[1] * u[1] * k2 - 1 /6.0f * u[1] * u[1] * u[1] * dk2 - 0.5 * u[1] * u[3] * k2) * ca2;

	a[5] = -84 * dx + (45 * u[0] + 10 * u[2] + u[4]) * ca1 - (10 * u[0] * u[0] * k1 + u[0] * u[0] * u[0] * dk1 + 3 * u[0] * u[2] * k1) * sa1
			+ (39 * u[1] - 7 * u[3] + 0.5 * u[5]) * ca2 + ( 7 * u[1] * u[1] * k2 - 0.5 * u[1] * u[1] * u[1] * dk2 - 1.5f * u[1] * u[3] * k2) * sa2;
	b[5] = -84 * dy + (45 * u[0] + 10 * u[2] + u[4]) * sa1 + (10 * u[0] * u[0] * k1 + u[0] * u[0] * u[0] * dk1 + 3 * u[0] * u[2] * k1) * ca1
			+ (39 * u[1] - 7 * u[3] + 0.5 * u[5]) * sa2 - ( 7 * u[1] * u[1] * k2 - 0.5 * u[1] * u[1] * u[1] * dk2 - 1.5f * u[1] * u[3] * k2) * ca2;

	a[6] = 70 * dx - (36 * u[0] + 7.5f * u[2] + 2/3.0f * u[4]) * ca1 + ( 7.5f * u[0] * u[0] * k1 + 2/3.0f * u[0] * u[0] * u[0] * dk1 + 2 * u[0] * u[2] * k1) * sa1
			- (34 * u[1] - 13/2.0f * u[3] + 0.5f * u[5]) * ca2 - (13/2.0f * u[1] * u[1] * k2 - 0.5f * u[1] * u[1] * u[1] * dk2 - 1.5f * u[1] * u[3] * k2) * sa2;
	b[6] = 70 * dy - (36 * u[0] + 7.5f * u[2] + 2/3.0f * u[4]) * sa1 - ( 7.5f * u[0] * u[0] * k1 + 2/3.0f * u[0] * u[0] * u[0] * dk1 + 2 * u[0] * u[2] * k1) * ca1
			- (34 * u[1] - 13/2.0f * u[3] + 0.5f * u[5]) * sa2 + (13/2.0f * u[1] * u[1] * k2 - 0.5f * u[1] * u[1] * u[1] * dk2 - 1.5f * u[1] * u[3] * k2) * ca2;

	a[7] = -20 * dx + (10 * u[0] + 2 * u[2] + 1/6.0f * u[4]) * ca1 - (2 * u[0] * u[0] * k1 + 1/6.0f * u[0] * u[0] * u[0] * dk1 + 0.5f * u[0] * u[3] * k1) * sa1
			+ (10 * u[1] - 2 * u[3] + 1/6.0f * u[5]) * ca2 + (2 * u[1] * u[1] * k2 - 1/6.0f * u[1] * u[1] * u[1] * dk2 - 0.5 * u[1] * u[3] * k2) * sa2;
	b[7] = -20 * dy + (10 * u[0] + 2 * u[2] + 1/6.0f * u[4]) * sa1 + (2 * u[0] * u[0] * k1 + 1/6.0f * u[0] * u[0] * u[0] * dk1 + 0.5f * u[0] * u[3] * k1) * ca1
			+ (10 * u[1] - 2 * u[3] + 1/6.0f * u[5]) * sa2 - (2 * u[1] * u[1] * k2 - 1/6.0f * u[1] * u[1] * u[1] * dk2 - 0.5 * u[1] * u[3] * k2) * ca2;
}
#endif

