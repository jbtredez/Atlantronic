#include "MatriceHomogene.h"
#include <cmath>

MatriceHomogene::MatriceHomogene()
{
	for(int i=0;i<15;i++)
	{
		matrice[i] = 0;
	}

	matrice[0] = 1.0f;
	matrice[5] = 1.0f;
	matrice[10] = 1.0f;
	matrice[15] = 1.0f;
}

void MatriceHomogene::translation(float dx,float dy,float dz)
{
// matrice homogène:
// on multiplie par une matrice de translation:
// ( m0 m4 m8  m12 )   ( 1  0  0  dx )
// ( m1 m5 m9  m13 ) * ( 0  1  0  dy )
// ( m2 m6 m10 m14 )   ( 0  0  1  dz )
// ( m3 m7 m11 m15 )   ( 0  0  0  1  )

	matrice[12] += matrice[0] * dx + matrice[4] * dy + matrice[8] * dz;
	matrice[13] += matrice[1] * dx + matrice[5] * dy + matrice[9] * dz;
	matrice[14] += matrice[2] * dx + matrice[6] * dy + matrice[10] * dz;
	matrice[15] += matrice[3] * dx + matrice[7] * dy + matrice[11] * dz;
}

void MatriceHomogene::rotation(float alpha,float x,float y,float z)
{
// matrice homogène:
// on multiplie par une matrice de rotation:
// ( m0 m4 m8  m12 )   ( x²(1-c)+c   xy(1-c)-zs  xz(1-c)+ys  0 )
// ( m1 m5 m9  m13 ) * ( xy(1-c)+zs  y²(1-c)+c   yz(1-c)-xs  0 )
// ( m2 m6 m10 m14 )   ( xz(1-c)-ys  yz(1-c)-xs  z²(1-c)+c   0 )
// ( m3 m7 m11 m15 )   ( 0           0           0           1 )

	// produit mat fait "à l'arrache"
	float c = cos(alpha);
	float s = sin(alpha);
	float unMc = 1-c;
	float xyUnMc = x*y*unMc;
	float xzUnMc = x*z*unMc;
	float yzUnMc = y*z*unMc;
	float xs = x*s;
	float ys = y*s;
	float zs = z*s;

	float rot[9] = { x*x*unMc + c, xyUnMc + zs, xzUnMc - ys, xyUnMc - zs, y*y*unMc + c, yzUnMc - xs, xzUnMc + ys, yzUnMc - xs, z*z*unMc + c};

	float m0  = matrice[0] * rot[0] + matrice[4] * rot[1] + matrice[8]  * rot[2];
	float m1  = matrice[1] * rot[0] + matrice[5] * rot[1] + matrice[9]  * rot[2];
	float m2  = matrice[2] * rot[0] + matrice[6] * rot[1] + matrice[10] * rot[2];
	float m4  = matrice[0] * rot[3] + matrice[4] * rot[4] + matrice[8]  * rot[5];
	float m5  = matrice[1] * rot[3] + matrice[5] * rot[4] + matrice[9]  * rot[5];
	float m6  = matrice[2] * rot[3] + matrice[6] * rot[4] + matrice[10] * rot[5];
	float m8  = matrice[0] * rot[6] + matrice[4] * rot[7] + matrice[8]  * rot[8];
	float m9  = matrice[1] * rot[6] + matrice[5] * rot[7] + matrice[9]  * rot[8];
	float m10 = matrice[2] * rot[6] + matrice[6] * rot[7] + matrice[10] * rot[8];

	matrice[0]  = m0;
	matrice[1]  = m1;
	matrice[2]  = m2;
	matrice[4]  = m4;
	matrice[5]  = m5;
	matrice[6]  = m6;
	matrice[8]  = m8;
	matrice[9]  = m9;
	matrice[10] = m10;
}

