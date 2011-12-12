//! @file regression.c
//! @brief Calcul d'une régression linéaire
//! @author Atlantronic

#include "kernel/math/regression.h"
#include "kernel/error_codes.h"

//!   | a |     | Somme( wi*xi*xi )    Somme( wi*xi )   |^-1      | Somme( wi*xi*yi )  |
//!   | b |   = | Somme( wi*xi )       Somme( wi )      |      *  | Somme( wi*yi )     |
//!
int regression_linear(float* x, float* y, float* w, int size, float* a, float* b)
{
	int i = 0;
	int err = 0;

	float sw = 0;      // Somme( wi )
	float swx = 0;     // Somme( wi*xi )
	float swx2 = 0;    // Somme( wi*xi*xi )
	float swy = 0;     // Somme( wi*yi )
	float swxy = 0;    // Somme( wi*xi*yi )

	float wx;

	for(i = 0; i < size; i++)
	{
		sw += w[i];
		wx = w[i] * x[i];
		swx += wx;
		swx2 += wx * x[i];
		swxy += wx * y[i];
		swy += w[i] * y[i];
	}

	// inversion de la matrice 2x2 et multiplication par le vecteur
	float det = swx2 * sw - swx * swx;
	if(det == 0)
	{
		err = ERR_LINEAR_REG_NULL_DET;
		goto end;
	}

	*a = (sw * swxy - swx * swy) / det;
	*b = (- swx * swxy + swx2 * swy) / det;

end:
	return err;
}