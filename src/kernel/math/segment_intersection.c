#include "kernel/math/segment_intersection.h"

//! calcule l'intersection h entre deux segments [a b] et [c d]
//! @return 0 si h est trouvé, < 0 sinon
int segment_intersection(const struct fx_vect2 a, const struct fx_vect2 b, const struct fx_vect2 c, const struct fx_vect2 d, struct fx_vect2* h)
{
	int64_t den = (((int64_t)(b.y - a.y)) * ((int64_t)(d.x - c.x)) - ((int64_t)(d.y - c.y)) * ((int64_t)(b.x - a.x))) >> 16;
	int64_t num = ((int64_t)(d.x - c.x)) * ((int64_t)(c.y - a.y)) - ((int64_t)(d.y - c.y)) * ((int64_t)(c.x - a.x));

	if(den == 0)
	{
		// droites (a b) et (c d) parallèles
		return -1;
	}

	int32_t alpha = num / den;

	// on n'est pas dans [a b]
	if(alpha < 0 || alpha >> 16 != 0)
	{
		return -1;
	}

	num = ((int64_t)(b.y - a.y)) * ((int64_t)(a.x - c.x)) - ((int64_t)(b.x - a.x)) * ((int64_t)(a.y - c.y));

	int32_t beta = num / den;

	// on n'est pas dans [c d]
	if(beta < 0 || beta >> 16 != 0)
	{
		return -2;
	}

	h->x = a.x + (((int64_t)alpha * (b.x - a.x)) >> 16);
	h->y = a.y + (((int64_t)alpha * (b.y - a.y)) >> 16);

	return 0;
}
