//! @file vect2.c
//! @brief vect2
//! @author Atlantronic

#include "kernel/math/vect2.h"

//!< @function distance_point_to_segment
//!< @brief calcule la distance entre un point M et un segment [AB]
//!< @param m coordonnées du point M
//!< @param a coordonnées du point A
//!< @param b coordonnées du point B
//!< @return distance
float distance_point_to_segment( Vect2 m,  Vect2 a,  Vect2 b)
{
	// Soit P la projection orthogonale de M sur (AB)
	// Soit t un scalaire tel que vect(AP) = t*(vect(AB))
	// t = (vect(AM).vect(AB))/((AB)^2)
	Vect2 am = m - a;
	Vect2 ab = b - a;
	float scalaire_am_ab = am.scalarProd(ab);
	float nab2 = ab.norm2();

	if( scalaire_am_ab <= 0 )
	{
		return am.norm(); // M est avant A
	}
	else if( scalaire_am_ab >= nab2 )
	{
		Vect2 bm = m - b;
		return bm.norm(); // M est après B
	}
	else
	{
		return fabsf( am.crossProd_z(ab) ) / sqrtf(nab2);
	}
}

