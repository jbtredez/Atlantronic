#include "kernel/math/line_intersection.h"

#define EPSILON               1e-5

//! calcule l'intersection h entre deux droies (a b) et (c d)
//! @return 0 si h est trouvé, < 0 sinon
int lineIntersection( Vect2 a,  Vect2 b,  Vect2 c,  Vect2 d, Vect2* h)
{
	float den = (b.y - a.y) * (d.x - c.x) - (d.y - c.y) * (b.x - a.x);
	float num = (d.x - c.x) * (c.y - a.y) - (d.y - c.y) * (c.x - a.x);

	if(fabsf(den) < EPSILON)
	{
		// droites (a b) et (c d) parallèles
		return -1;
	}

	*h = a +  (num / den) * (b - a);

	return 0;
}
