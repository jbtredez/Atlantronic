//! @file pid.cxx
//! @brief PID control
//! @author Atlantronic

#include "pid.h"

Pid::Pid(float Kp, float Ki, float Kd, float Max)
{
	kp = Kp;
	ki = Ki;
	kd = Kd;
	max_out = Max;
	max_integral = Max;

	integral = 0;
	lastError = 0;
}

float Pid::compute(float error, float dt)
{
	float out;

	float derivate = (error - lastError) / dt;
	lastError = error;
	integral += error * dt;

	// saturation de l'integrale
	if(max_integral)
	{
		if(integral > max_integral)
		{
			integral = max_integral;
		}
		else if(integral < -max_integral)
		{
			integral = -max_integral;
		}
	}

	// calcul du PID
	out = kp * error + ki * integral + kd * derivate;

	// saturation de la sortie
	if(max_out)
	{
		if(out > max_out)
		{
			out = max_out;
		}
		else if(out < -max_out)
		{
			out = -max_out;
		}
	}

	return out;
}

void Pid::reset()
{
	integral = 0;
	lastError = 0;
}
