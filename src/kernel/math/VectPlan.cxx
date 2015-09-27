//! @file vect_pos.c
//! @brief Changements de reperes
//! @author Atlantronic

#include "kernel/math/VectPlan.h"

VectPlan transferSpeed(const VectPlan &A, const VectPlan &B, const VectPlan &speed)
{
	VectPlan res = speed;

	res.x += speed.theta * (A.y - B.y);
	res.y += speed.theta * (B.x - A.x);

	return res;
}

//! changement de repere du repère local au repere absolu
//! origin : origine du repère local dans le repère absolu
VectPlan loc_to_abs(const VectPlan& origin, const VectPlan& pos)
{
	VectPlan res;
	float c = cosf(origin.theta);
	float s = sinf(origin.theta);

	res.x = origin.x + c * pos.x - s * pos.y;
	res.y = origin.y + s * pos.x + c * pos.y;
	res.theta = origin.theta + pos.theta;

	return res;
}

//! changement de repere du repère local au repere absolu
//! origin : origine du repère local dans le repère absolu
Vect2 loc_to_abs(const VectPlan& origin, const Vect2& pos)
{
	Vect2 res;
	float c = cosf(origin.theta);
	float s = sinf(origin.theta);
	res.x = origin.x + c * pos.x - s * pos.y;
	res.y = origin.y + s * pos.x + c * pos.y;

	return res;
}

//! changement de repere du repère absolu au repere local
//! origin : origine du repère local dans le repère absolu
VectPlan abs_to_loc(const VectPlan& origin, const VectPlan& pos)
{
	VectPlan res;
	float c = cosf(origin.theta);
	float s = sinf(origin.theta);

	float dx = pos.x - origin.x;
	float dy = pos.y - origin.y;
	res.x =  c * dx + s * dy;
	res.y = -s * dx + c * dy;
	res.theta = pos.theta - origin.theta;

	return res;
}

//! changement de repere du repère absolu au repere local
//! origin : origine du repère local dans le repère absolu
Vect2 abs_to_loc(const VectPlan& origin, const Vect2& pos)
{
	Vect2 res;
	float c = cosf(origin.theta);
	float s = sinf(origin.theta);

	float dx = pos.x - origin.x;
	float dy = pos.y - origin.y;
	res.x =  c * dx + s * dy;
	res.y = -s * dx + c * dy;

	return res;
}

VectPlan loc_to_abs_speed(const double theta, const VectPlan &speed)
{
	VectPlan res;
	float c = cosf(theta);
	float s = sinf(theta);
	res.x = c * speed.x - s * speed.y;
	res.y = s * speed.x + c * speed.y;
	res.theta = speed.theta;

	return res;
}

