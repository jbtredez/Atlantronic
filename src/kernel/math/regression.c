//! @file regression.c
//! @brief Calcul d'une régression linéaire
//! @author Atlantronic

#include "kernel/math/regression.h"
#include "kernel/error_codes.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

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

int regression_poly(struct vect_pos* pt, int size, float seuil, char* type)
{
	int err = 0;
	int a = 0;
	int b = size-1;
	int c = 0;
	int id_max = 0;
	int dist = 0;
	int dist_max = -1;
	float nab = 0;
	struct vect2i ab;
	struct vect2i ac;

	memset(type, 0x00, size);

	while(1)
	{
		dist_max = -1;
		type[a] = 1;
		type[b] = 1;
		ab.x = (int)pt[b].x - (int)pt[a].x;
		ab.y = (int)pt[b].y - (int)pt[a].y;
		nab = sqrtf(ab.x*ab.x+ab.y*ab.y);

		for(c = a + 1; c<b; c++)
		{
			ac.x = (int)pt[c].x - (int)pt[a].x;
			ac.y = (int)pt[c].y - (int)pt[a].y;
			dist = abs(ab.x * ac.y - ab.y * ac.x);
			if(dist > dist_max)
			{
				dist_max = dist;
				id_max = c;
			}
		}

		if(dist_max > seuil * nab && a != b)
		{
			// nouveau point
			type[id_max] = 1;

			// choix du segment a regarder
			if(id_max > a+1)
			{
				b = id_max;
				continue;
			}

			if(id_max < b - 1)
			{
				a = id_max;
				continue;
			}
		}

		a = b;
		b++;
		while(b < size && type[b] != 1)
		{
			b++;
		}

		if(b >= size)
		{
			goto end;
		}
	}

end:
	return err;
}