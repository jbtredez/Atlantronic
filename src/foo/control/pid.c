//! @file pid.c
//! @brief PID control
//! @author Atlantronic

#include "control/pid.h"

void pid_init(struct pid *pid, float kp, float ki, float kd, float max)
{
	pid->kp = kp * max;
	pid->ki = ki * max;
	pid->kd = kd * max;

	pid->integral = 0;
	pid->previous_in = 0;
	pid->max_integral = 0;
	pid->max_out = max;
}

float pid_apply(struct pid *pid, float in)
{
	float derivate;
	float out;

	derivate = in - pid->previous_in;
	pid->previous_in = in;
	pid->integral += in;

	// saturation de l'integrale
	if(pid->max_integral)
	{
		if(pid->integral > pid->max_integral)
		{
			pid->integral = pid->max_integral;
		}
		else if(pid ->integral < -pid->max_integral)
		{
			pid->integral = -pid->max_integral;
		}
	}

	// calcul du PID
	out = pid->kp * in + pid->ki * pid->integral + pid->kd * derivate;

	// saturation de la sortie
	if(pid->max_out)
	{
		if(out > pid->max_out)
		{
			out = pid->max_out;
		}
		else if(out < -pid->max_out)
		{
			out = -pid->max_out;
		}
	}

	return out;
}

void pid_reset(struct pid *pid)
{
	pid->integral = 0;
	pid->previous_in = 0;
}