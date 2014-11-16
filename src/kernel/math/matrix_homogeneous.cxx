#include "matrix_homogeneous.h"
#include <math.h>
#include <string.h>

#define EPSILON                1e-6

MatrixHomogeneous::MatrixHomogeneous()
{
	setIdentity();
}

void MatrixHomogeneous::setIdentity()
{
	memset(val, 0, sizeof(val));
	val[0] = 1;
	val[5] = 1;
	val[10] = 1;
}

void MatrixHomogeneous::translate(float x, float y, float z)
{
	val[3] += val[0] * x + val[1] * y + val[2] * z;
	val[7] += val[4] * x + val[5] * y + val[6] * z;
	val[11] += val[8] * x + val[9] * y + val[10] * z;
}

void MatrixHomogeneous::scale(float s)
{
	for(int i = 0; i < 12; i++)
	{
		val[i] *= s;
	}
}

MatrixHomogeneous MatrixHomogeneous::invert()
{
	MatrixHomogeneous m;

	// rotation R2 = Rt
	m.val[0] = val[0];
	m.val[1] = val[4];
	m.val[2] = val[8];
	m.val[4] = val[1];
	m.val[5] = val[5];
	m.val[6] = val[9];
	m.val[8] = val[2];
	m.val[9] = val[6];
	m.val[10] = val[10];

	// translation T2 = -R2 * T
	m.val[3] = m.val[0] * val[3] + m.val[1] * val[7] + m.val[2] * val[11];
	m.val[7] = m.val[4] * val[3] + m.val[5] * val[7] + m.val[6] * val[11];
	m.val[11] = m.val[8] * val[3] + m.val[9] * val[7] + m.val[10] * val[11];

	return m;
}

void MatrixHomogeneous::rotateX(float theta)
{
	float c = cosf(theta);
	float s = sinf(theta);

	float a = val[1];
	float b = val[2];
	val[1] = a * c + b * s;
	val[2] = -a * s + b * c;

	a = val[5];
	b = val[6];
	val[5] = a * c + b * s;
	val[6] = -a * s + b * c;

	a = val[9];
	b = val[10];
	val[9] = a * c + b * s;
	val[10] = -a * s + b * c;
}

void MatrixHomogeneous::rotateY(float theta)
{
	float c = cosf(theta);
	float s = sinf(theta);

	float a = val[0];
	float b = val[2];
	val[0] = a * c + b * s;
	val[2] = -a * s + b * c;

	a = val[4];
	b = val[6];
	val[4] = a * c + b * s;
	val[6] = -a * s + b * c;

	a = val[8];
	b = val[10];
	val[8] = a * c + b * s;
	val[10] = -a * s + b * c;
}

void MatrixHomogeneous::rotateZ(float theta)
{
	float c = cosf(theta);
	float s = sinf(theta);

	float a = val[0];
	float b = val[1];
	val[0] = a * c + b * s;
	val[1] = -a * s + b * c;

	a = val[4];
	b = val[5];
	val[4] = a * c + b * s;
	val[5] = -a * s + b * c;

	a = val[8];
	b = val[9];
	val[8] = a * c + b * s;
	val[9] = -a * s + b * c;
}
