//! @file pid.c
//! @brief PID control
//! @author Atlantronic

#include "control/pid.h"

void pid_init(struct pid *pid, int32_t kp, int32_t ki, int32_t kd, int32_t max, int fx_unit)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->fx_unit = fx_unit;

	pid->integral = 0;
	pid->previous_in = 0;
	pid->max_integral = 0;
	pid->max_out = max;
}

int32_t pid_apply(struct pid *pid, int32_t in)
{
	int32_t derivate;
	int32_t out;

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
	out = ((int64_t)pid->kp * (int64_t)in + (int64_t)pid->ki * (int64_t)pid->integral + (int64_t)pid->kd * (int64_t)derivate) >> pid->fx_unit;

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