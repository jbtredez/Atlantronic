//! @file pid.c
//! @brief PID control
//! @author Atlantronic

#include "control/pid.h"

void pid_init(struct pid *pid, float kp, float ki, float kd, float max)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	pid->integral = 0;
	pid->derivate = 0;
	pid->max_integral = 0;
	pid->max_out = max;
}

float pid_apply(struct pid *pid, float in)
{
	float out;

	pid->derivate = in - pid->derivate;
	pid->integral += in;

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

	out = pid->kp * in + pid->ki * pid->integral + pid->kd * pid->derivate;

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
