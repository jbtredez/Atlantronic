#ifndef VECT2_H
#define VECT2_H

//! @file vect2.h
//! @brief vect2
//! @author Atlantronic

#include <stdint.h>
#include <math.h>

class Vect2
{
	public:
		Vect2()
		{
			x = 0;
			y = 0;
		}

		Vect2(float X, float Y)
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

inline Vect2 operator+(Vect2 a, Vect2 b)
{
	return Vect2(a.x + b.x, a.y + b.y);
}

inline Vect2 operator-(Vect2 a, Vect2 b)
{
	return Vect2(a.x - b.x, a.y - b.y);
}

inline Vect2 operator*(float k, Vect2 a)
{
	return Vect2(k * a.x, k * a.y);
}

inline Vect2 operator*(Vect2 a, float k)
{
	return Vect2(k * a.x, k * a.y);
}

inline Vect2 operator/(Vect2 a, float k)
{
	return Vect2(a.x / k, a.y / k);
}

inline Vect2 operator-(Vect2 a)
{
	return Vect2(-a.x, -a.y);
}

//!< @function fx_Vect2_scalar_product
//!< @brief calcul le produit scalaire de deux vecteurs u et v
//!< @param vecteur u
//!< @param vecteur v
//!< @return produit scalaire calculé avec les positions en mm
inline float scalar_product(const Vect2& u, const Vect2& v)
{
	return u.x * v.x +u.y * v.y;
}

//!< @function fx_Vect2_vector_product_z
//!< @brief calcule la compose sur Oz du produit vectoriel de u et v (appartenant à (Ox,Ox))
//!< @param vecteur u
//!< @param vecteur v
//!< @return composante sur Oz du produit vectoriel calculé avec les positions en mm
inline float cross_product_z(const Vect2& u, const Vect2& v)
{
	return u.x * v.y - u.y * v.x;
}

float distance_point_to_segment(const Vect2& m, const Vect2& a, const Vect2& b);

#endif
