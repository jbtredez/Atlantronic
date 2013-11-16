#include "kinematics.h"
#include <math.h>

void Kinematics::setSpeed(float wantedSpeed, const KinematicsParameters& param, float dt)
{
	float vmin;
	float vmax;

	// choix de vmin et vmax pour respecter l'acceleration max et la deceleration max
	if( v >= 0)
	{
		vmin = v - param.dMax * dt;
		vmax = v + param.aMax * dt;
	}
	else
	{
		vmin = v - param.aMax * dt;
		vmax = v + param.dMax * dt;
	}

	// saturation de vmin pour respecter la vitesse maximale
	if( vmin < -param.vMax )
	{
		vmin = -param.vMax;
	}

	// saturation de vmax pour respecter la vitesse maximale
	if( vmax > param.vMax )
	{
		vmax = param.vMax;
	}

	// il faut v dans [vmin vmax]
	if(wantedSpeed < vmin)
	{
		wantedSpeed = vmin;
	}
	else if(wantedSpeed > vmax)
	{
		wantedSpeed = vmax;
	}

	// mise a jour de la position et de l'acceleration
	a = (wantedSpeed - v) / dt;
	v = wantedSpeed;
	pos = pos + v * dt;
}

void Kinematics::setPosition(float wantedPos, float wantedSpeed, KinematicsParameters param, float dt)
{
	float d = wantedPos - pos;
	float vMax = param.vMax;
	wantedSpeed = fabsf(wantedSpeed);

	// saturation de vMax pour la rampe de deceleration
	// formule theorique : float vMaxStop = sqrt( v * v + 2 * fabs(d) * dmax);
	// on calcule une compensation a cause de l'echantillonage
	float corr = param.dMax * dt / 2;
	float vc = wantedSpeed + corr;
	float vMaxSlowDown = sqrtf(vc * vc + 2 * fabsf(d) * param.dMax) - corr;

	// on evite de depasser si on peut s'arreter avec 2*dmax (vitesse faible)
	float dv = fabsf(vMaxSlowDown);
	if( dv * dt > fabsf(d) && dv / dt <  2 * param.dMax)
	{
		vMaxSlowDown = wantedSpeed + fabsf(d) / dt;
		param.dMax *= 2;
	}

	if(vMaxSlowDown < vMax)
	{
		vMax = vMaxSlowDown;
	}

	if( d < 0 )
	{
		vMax = -vMax;
	}

	setSpeed(vMax, param, dt);
}
