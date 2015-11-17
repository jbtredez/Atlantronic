#ifndef VECT_PLAN_H
#define VECT_PLAN_H

//! @file VectPlan.h
//! @brief VectPlan
//! @author Atlantronic

#include <stdint.h>
#include <math.h>
#include "Vect2.h"

#define EPSILON                                 1e-4

//! @struct VectPlan
//! représentation d'un vecteur sur la table
class VectPlan : public Vect2
{
	public:
		VectPlan() : Vect2(0,0)
		{
			theta = 0;
		}

		VectPlan(float X, float Y, float Theta):Vect2(X,Y)
		{

			theta = Theta;
		}

		VectPlan(Vect2 a, float Theta) : Vect2(a)
		{
			theta = Theta;
		}

		inline VectPlan symetric(int color)
		{
			if( color == 1)
			{
				return *this;
			}
			else
			{
				return VectPlan(-x, y, M_PI - theta);
			}
		}

		inline bool operator ==(const VectPlan &v) const
		{
			return (fabsf(v.x - x) < EPSILON) && (fabsf(v.y - y) < EPSILON) && (fabsf(v.theta - theta) < EPSILON);
		}

		inline VectPlan operator+( VectPlan b) const
		{
			return VectPlan(x + b.x, y + b.y, theta + b.theta);
		};

		inline VectPlan operator-(VectPlan b) const
		{
			return VectPlan(x - b.x,y - b.y, theta - b.theta);
		};

		inline VectPlan operator*(float k) const
		{
			return VectPlan(k * x, k * y, k * theta);
		};


		inline VectPlan operator/( float k) const
		{
			return VectPlan(x / k, y / k, theta / k);
		};

		inline float norm(float thetaWeight = 0) const
		{
			return sqrtf(x * x + y * y + thetaWeight * thetaWeight * theta * theta);
		}

		inline float scalarProd(const VectPlan& v, float thetaWeight = 0) const
		{
			return x * v.x + y * v.y + thetaWeight * thetaWeight * theta * v.theta;
		}

		float theta;      //!< orientation en rd
}__attribute__((packed));

inline VectPlan operator*(float k, VectPlan a)
{
	return VectPlan(k * a.x, k * a.y, k * a.theta);
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
