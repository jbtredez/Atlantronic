//! @file odometry.c
//! @brief Odometry
//! @author Atlantronic

#include "odometry.h"

VectPlan odometry2turret(const VectPlan &cp, const VectPlan &A, const VectPlan &B, const VectPlan &v1, const VectPlan &v2, float* slippageSpeed)
{
	VectPlan res(0,0,0);
	float dx = B.x - A.x;
	float dy = B.y - A.y;
	float dv = 0;

	// on divise par le plus grand pour eviter les pb numeriques
	if( fabs(dx) > fabs(dy) )
	{
		res.theta = (v2.y - v1.y) / dx;
		dv = fabsf(v1.x - v2.x - dy * res.theta);
	}
	else if( fabsf(dy) > 0 )
	{
		res.theta = (v1.x - v2.x) / dy;
		dv = fabsf(v2.y - v1.y - dx * res.theta);
	}
	else
	{
		// calcul non realisable, A et B sont au même endroit
		// on retourne une vitesse nulle
		goto end;
	}

	res.x = 0.5 * (v1.x + v2.x + res.theta * (A.y + B.y - 2 * cp.y));
	res.y = 0.5 * (v1.y + v2.y + res.theta * ( 2 * cp.x - A.x - B.x));

end:
	if( slippageSpeed )
	{
		*slippageSpeed = dv;
	}

	return res;
}
