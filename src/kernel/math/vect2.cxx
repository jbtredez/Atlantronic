//! @file vect2.c
//! @brief vect2
//! @author Atlantronic

#include "kernel/math/vect2.h"
#define EPSILON                 1e-5

//!< @function distance_point_to_segment
//!< @brief calcule la distance entre un point M et un segment [AB]
//!< @param m coordonnées du point M
//!< @param a coordonnées du point A
//!< @param b coordonnées du point B
//!< @return distance
float distance_point_to_segment(const vect2& m, const vect2& a, const vect2& b)
{
	// Soit P la projection orthogonale de M sur (AB)
	// Soit t un scalaire tel que vect(AP) = t*(vect(AB))
	// t = (vect(AM).vect(AB))/((AB)^2)
	vect2 am = m - a;
	vect2 ab = b - a;
	float scalaire_am_ab = scalar_product(am, ab);
	float nab2 = ab.norm2();

	if( scalaire_am_ab <= 0 )
	{
		return am.norm(); // M est avant A
	}
	else if( scalaire_am_ab >= nab2 )
	{
		vect2 bm = m - b;
		return bm.norm(); // M est après B
	}
	else
	{
		return fabsf( cross_product_z(am, ab) ) / sqrtf(nab2);
	}
}

