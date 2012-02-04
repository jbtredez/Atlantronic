#ifndef PID_H
#define PID_H

//! @file pid.h
//! @brief PID control
//! @author Atlantronic

#include <stdint.h>

struct pid
{
	int32_t kp;            //!< proportional gain
	int32_t ki;            //!< integral gain (* te)
	int32_t kd;            //!< derivate gain (/ te)
	int fx_unit;           //!< gains en 2^-fx_unit

	int32_t integral;      //!< previous integral
	int32_t previous_in;   //!< previous input (for derivative)
	int32_t max_integral;    //!< saturate integral
	int32_t max_out;       //!< saturate output
};

void pid_init(struct pid *pid, int32_t kp, int32_t ki, int32_t kd, int32_t max, int fx_unit);

int32_t pid_apply(struct pid *pid, int32_t in);

void pid_reset(struct pid *pid);

#endif
