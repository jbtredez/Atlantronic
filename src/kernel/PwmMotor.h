#ifndef PWM_MOTOR_H
#define PWM_MOTOR_H

#include <stdint.h>
#include "kernel/control/kinematics.h"
#include "kernel/fault.h"
#include "MotorInterface.h"
#include "kernel/driver/pwm.h"
#include "middleware/motion/pid.h"
#include "kernel/driver/encoder/EncoderInterface.h"


class PwmMotor : public MotorInterface
{
	public:
		void set_speed(float v);
		bool is_in_motion();
		void disable();
		int pwmId;
		Pid pid;
		EncoderInterface* encoder;
};

#endif
