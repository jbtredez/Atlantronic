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

		inline Vect2 operator+(const Vect2 b)
		{
			return Vect2(x + b.x, y + b.y);
		}

		inline Vect2 operator*(float k)
		{
			return Vect2(k * x, k * y);
		}

		inline Vect2 operator/( float k)
		{
			return Vect2(x / k, y / k);
		}

		inline Vect2 operator-(const Vect2 b)
		{
			return Vect2(x-b.x, y-b.y);
		}

		//!< @brief calcul le produit avec le vecteur v
		//!< @param vecteur v
		//!< @return produit calculé avec les positions en mm
		inline Vect2 operator*(const Vect2 v)
		{
			return Vect2(x * v.x, y * v.y);
		}

		//!< @brief calcul le produit scalaire avec le vecteur v
		//!< @param vecteur v
		//!< @return produit scalaire calculé avec les positions en mm
		inline float scalarProd(const Vect2& v)
		{
			return x * v.x + y * v.y;
		}

		//!< @function fx_Vect2_vector_product_z
		//!< @brief calcule la compose sur Oz du produit vectoriel avec v (appartenant à (Ox,Oy))
		//!< @param vecteur v
		//!< @return composante sur Oz du produit vectoriel calculé avec les positions en mm
		inline float crossProd_z(const Vect2& v)
		{
			return x * v.y - y * v.x;
		}

		float x;
		float y;
};

inline Vect2 operator*(float k, Vect2 a)
{
	return Vect2(k * a.x, k * a.y);
}

inline Vect2 operator-(Vect2 a)
{
	return Vect2(-a.x, -a.y);
}

float distance_point_to_segment( Vect2 m,  Vect2 a,  Vect2 b);

#endif
