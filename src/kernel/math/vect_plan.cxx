//! @file vect_pos.c
//! @brief Changements de reperes
//! @author Atlantronic

#include "kernel/math/vect_plan.h"


VectPlan transferSpeed(const VectPlan &A, const VectPlan &B, const VectPlan &speed)
{
	VectPlan res = speed;

	res.x += speed.theta * (A.y - B.y);
	res.y += speed.theta * (B.x - A.x);

	return res;
}
