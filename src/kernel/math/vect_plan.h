#ifndef VECT_PLAN_H
#define VECT_PLAN_H

//! @file vect_plan.h
//! @brief vect_plan
//! @author Atlantronic

#include <stdint.h>
#include <math.h>
#include "vect2.h"

//! @struct VectPlan
//! représentation d'un vecteur sur la table
class VectPlan
{
	public:
		VectPlan()
		{
			x = 0;
			y = 0;
			theta = 0;
		}

		VectPlan(float X, float Y, float Theta)
		{
			x = X;
			y = Y;
			theta = Theta;
		}

		inline void symetric(int color)
		{
			if( color != 1 )
			{
				x *= -1;
				theta = M_PI - theta;
			}
		}

		inline float norm()
		{
			return sqrtf(x * x + y * y);
		}

		inline float norm2()
		{
			return x * x + y * y;
		}

		float x;          //!< coordonnée selon l'axe x en mm
		float y;          //!< coordonnée selon l'axe y en mm
		float theta;      //!< orientation en rd
}__attribute__((packed));

inline VectPlan operator+(VectPlan a, VectPlan b)
{
	return VectPlan(a.x + b.x, a.y + b.y, a.theta + b.theta);
}

inline VectPlan operator-(VectPlan a, VectPlan b)
{
	return VectPlan(a.x - b.x, a.y - b.y, a.theta - b.theta);
}

inline VectPlan operator*(float k, VectPlan a)
{
	return VectPlan(k * a.x, k * a.y, k * a.theta);
}

inline VectPlan operator*(VectPlan a, float k)
{
	return VectPlan(k * a.x, k * a.y, k * a.theta);
}

inline VectPlan operator/(VectPlan a, float k)
{
	return VectPlan(a.x / k, a.y / k, a.theta / k);
}

inline VectPlan operator-(VectPlan a)
{
	return VectPlan(-a.x, -a.y, -a.theta);
}

VectPlan transferSpeed(const VectPlan &A, const VectPlan &B, const VectPlan &speed);

VectPlan loc_to_abs(const VectPlan& origin, const VectPlan& pos);
Vect2 loc_to_abs(const VectPlan& origin, const Vect2& pos);

VectPlan abs_to_loc(const VectPlan& origin, const VectPlan& pos);
Vect2 abs_to_loc(const VectPlan& origin, const Vect2& pos);

VectPlan loc_to_abs_speed(const double theta, const VectPlan &speed);

inline VectPlan abs_to_loc_speed(const double theta, const VectPlan &speed)
{
	return loc_to_abs_speed(-theta, speed);
}

#endif
