//! @file distance.c
//! @brief Fonctions de calcul de distances
//! @author Atlantronic

#include "kernel/math/distance.h"
#include <math.h>

//!< @function distance_point_to_point_squared
//!< @brief calcule le carré de la distance entre deux points A et B
//!< @param a coordonnées du point A
//!< @param b coordonnées du point B
//!< @return distance en mm^2
int32_t distance_point_to_point_squared(const struct fx_vect2* a, const struct fx_vect2* b)
{
	int32_t ax = a->x >> 16;
	int32_t bx = b->x >> 16;
	int32_t ay = a->y >> 16;
	int32_t by = b->y >> 16;
	return (bx-ax)*(bx-ax)+(by-ay)*(by-ay);
}

//!< @function distance_point_to_point
//!< @brief calcule la distance entre deux points A et B
//!< @param a coordonnées du point A
//!< @param b coordonnées du point B
//!< @return distance en mm^2
int32_t distance_point_to_point(const struct fx_vect2* a, const struct fx_vect2* b)
{
	float distance;
	distance = distance_point_to_point_squared(a,b);
	distance = sqrtf(distance);
	distance *= 1<<16;
	return (int32_t)distance;
}


//!< @function distance_point_to_segment
//!< @brief calcule la distance entre un point M et un segment [AB]
//!< @param m coordonnées du point M
//!< @param a coordonnées du point A
//!< @param b coordonnées du point B
//!< @return distance en 2^-16 mm
int32_t distance_point_to_segment(const struct fx_vect2* m, const struct fx_vect2* a, const struct fx_vect2* b)
{
	const int32_t ab_squared = distance_point_to_point_squared(a, b);
	if (ab_squared == 0)
	{
		return distance_point_to_point(m,a);
	}
	else
	{

		// Soit P la projection orthogonale de M sur (AB)
		// Soit t un scalaire tel que vect(AP) = t*(vect(AB))
		// t = (vect(AM).vect(AB))/((AB)^2)
		const struct fx_vect2 am = {(m->x)-(a->x), (m->y)-(a->y)};
		const struct fx_vect2 ab = {(b->x)-(a->x), (b->y)-(a->y)};
		//calculs sur variables en mm
		const float scalaire_am_ab = ((am.x>>16)*(ab.x>>16))+((am.y>>16)*(ab.y>>16));
		const float t=scalaire_am_ab/ab_squared;

		if (t < 0.0f) return distance_point_to_point(m,a); // M est avant A
		else if (t > 1.0f) return distance_point_to_point(m,b); // M est après B
		else
		{
			//projection dans le segment
			const struct fx_vect2 p = {(a->x)+t*(ab.x), (a->y)+t*(ab.y)};
			return distance_point_to_point(m, &p);
		}
	}
}

