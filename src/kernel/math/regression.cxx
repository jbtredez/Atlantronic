//! @file regression.c
//! @brief Calcul d'une régression linéaire
//! @author Atlantronic

#include "kernel/math/regression.h"
#include "kernel/error_codes.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

#ifdef LINUX
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
		err = -1;
		goto end;
	}

	*a = (sw * swxy - swx * swy) / det;
	*b = (- swx * swxy + swx2 * swy) / det;

end:
	return err;
}
#endif

int regression_poly(Vect2* pt, int size, int seuil, Vect2* regression_pt, int reg_size)
{
	int a = 0;
	int b = size-1;
	int c = 0;
	int id_max = 0;
	float dist = 0;
	float dist_max = -1;
	float nab = 0;
	Vect2 ab;
	Vect2 ac;

	int regression_num = 0;
	int pta = 0;

	while( a < size && pt[a].x == 0 && pt[a].y == 0)
	{
		a++;
	}

	if(a >= size)
	{
		goto end;
	}

	regression_pt[0] = pt[a];
	regression_num++;

	while( b > a && pt[b].x == 0 && pt[b].y == 0)
	{
		b--;
	}

	if( a == b)
	{
		goto end;
	}

	regression_pt[1] = pt[b];
	regression_num++;

	while(regression_num < reg_size)
	{
		dist_max = -1;
		ab = pt[b] - pt[a];
		nab = ab.norm();

		for(c = a + 1; c<b; c++)
		{
			ac = pt[c] - pt[a];
			dist = fabsf(ab.x * ac.y - ab.y * ac.x);
			if(dist > dist_max && (pt[c].x != 0 || pt[c].y != 0))
			{
				dist_max = dist;
				id_max = c;
			}
		}

		if(dist_max > seuil * nab && b - a > 2)
		{
			// nouveau point
			int i;
			for( i = regression_num - 1 ; i > pta ; i--)
			{
				regression_pt[i + 1] = regression_pt[i];
			}
			regression_pt[pta + 1] = pt[id_max];
			regression_num++;

			// choix du segment a regarder
			if(id_max > a+1)
			{
				b = id_max;
				continue;
			}

			if(id_max < b - 1)
			{
				pta++;
				a = id_max;
				continue;
			}

			pta++;
		}

		pta++;
		a = b;
		b++;
		while(b < size && (pt[b].x != regression_pt[pta+1].x || pt[b].y != regression_pt[pta+1].y) )
		{
			b++;
		}

		if(b >= size)
		{
			goto end;
		}
	}

end:
	return regression_num;
}
