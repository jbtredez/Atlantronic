#include "kernel/math/segment_intersection.h"

#define EPSILON               1e-5

//! calcule l'intersection h entre deux segments [a b] et [c d]
//! @return 0 si h est trouvé, < 0 sinon
int segment_intersection(const vect2 a, const vect2 b, const vect2 c, const vect2 d, vect2* h)
{
	float den = (b.y - a.y) * (d.x - c.x) - (d.y - c.y) * (b.x - a.x);
	float num = (d.x - c.x) * (c.y - a.y) - (d.y - c.y) * (c.x - a.x);

	if(fabsf(den) < EPSILON)
	{
		// droites (a b) et (c d) parallèles
		return -1;
	}

	float alpha = num / den;

	// on n'est pas dans [a b]
	if(alpha < 0 || alpha > 1)
	{
		return -1;
	}

	num = (b.y - a.y) * (a.x - c.x) - (b.x - a.x) * (a.y - c.y);

	float beta = num / den;

	// on n'est pas dans [c d]
	if(beta < 0 || beta > 1)
	{
		return -2;
	}

	*h = a + alpha * (b - a);

	return 0;
}
