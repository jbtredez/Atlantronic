#include <math.h>
#include "findRotation.h"

float findRotation(float start, float end)
{
	float dtheta = end - start;
	bool neg = dtheta < 0;

	// modulo 1 tour => retour dans [ 0 ; 2*M_PI [
	dtheta = fmodf(dtheta, 2*M_PI);
	if( neg )
	{
		dtheta += 2*M_PI;
	}

	// retour dans ] -M_PI ; M_PI ] tour
	if( dtheta > M_PI )
	{
		dtheta -= 2*M_PI;
	}

	return dtheta;
}

//! on met l'angle entre [-M_PI et M_PI]
float modulo2pi(float theta)
{
	bool neg = theta < 0;

	theta = fmodf(theta, 2*M_PI);
	if( neg )
	{
		theta += 2*M_PI;
	}

	// retour dans ] -M_PI ; M_PI ] tour
	if( theta > M_PI )
	{
		theta -= 2*M_PI;
	}

	return theta;
	/*if( theta >= -M_PI && theta < M_PI )
	{
		return theta;
	}

	return theta - (floor( (theta - M_PI) / (2*M_PI))) * 2 * M_PI - 2 * M_PI;*/
}
