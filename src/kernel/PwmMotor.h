#ifndef PWM_MOTOR_H
#define PWM_MOTOR_H

#include <stdint.h>
#include "kernel/control/kinematics.h"
#include "kernel/fault.h"
#include "MotorInterface.h"


class PwmMotor : public MotorInterface
{
	public:
		void set_speed(float v);
		int pwmId;
};

#endif
