#ifndef PID_H
#define PID_H

//! @file pid.h
//! @brief PID control
//! @author Atlantronic

#include <stdint.h>

struct pid
{
	float kp;              //!< proportional gain
	float ki;              //!< integral gain (* te)
	float kd;              //!< derivate gain (/ te)

	float integral;        //!< previous integral
	float previous_in;     //!< previous input (for derivative)
	float max_integral;    //!< saturate integral
	float max_out;         //!< saturate output
};

void pid_init(struct pid *pid, float kp, float ki, float kd, float max);

float pid_apply(struct pid *pid, float in);

void pid_reset(struct pid *pid);

#endif
