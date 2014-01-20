#ifndef VECT2_H
#define VECT2_H

//! @file vect2.h
//! @brief vect2
//! @author Atlantronic

#include <stdint.h>
#include <math.h>

struct fx_vect2
{
	int32_t x;
	int32_t y;
};

class vect2
{
	public:
		vect2()
		{
			x = 0;
			y = 0;
		}

		vect2(float X, float Y)
		{
			x = X;
			y = Y;
		}

		inline float norm()
		{
			return sqrtf(x * x + y * y);
		}

		inline float norm2()
		{
			return x * x + y * y;
		}

		float x;
		float y;
};

inline vect2 operator+(vect2 a, vect2 b)
{
	return vect2(a.x + b.x, a.y + b.y);
}

inline vect2 operator-(vect2 a, vect2 b)
{
	return vect2(a.x - b.x, a.y - b.y);
}

inline vect2 operator*(float k, vect2 a)
{
	return vect2(k * a.x, k * a.y);
}

inline vect2 operator*(vect2 a, float k)
{
	return vect2(k * a.x, k * a.y);
}

inline vect2 operator/(vect2 a, float k)
{
	return vect2(a.x / k, a.y / k);
}

inline vect2 operator-(vect2 a)
{
	return vect2(-a.x, -a.y);
}

//!< @function fx_vect2_scalar_product
//!< @brief calcul le produit scalaire de deux vecteurs u et v
//!< @param vecteur u
//!< @param vecteur v
//!< @return produit scalaire calculé avec les positions en mm
inline float scalar_product(const vect2& u, const vect2& v)
{
	return u.x * v.x +u.y * v.y;
}

//!< @function fx_vect2_vector_product_z
//!< @brief calcule la compose sur Oz du produit vectoriel de u et v (appartenant à (Ox,Ox))
//!< @param vecteur u
//!< @param vecteur v
//!< @return composante sur Oz du produit vectoriel calculé avec les positions en mm
inline float cross_product_z(const vect2& u, const vect2& v)
{
	return u.x * v.y - u.y * v.x;
}

float distance_point_to_segment(const vect2& m, const vect2& a, const vect2& b);

#endif
