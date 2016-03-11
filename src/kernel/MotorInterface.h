#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#include <stdint.h>
#include "kernel/control/kinematics.h"
#include "kernel/portmacro.h"

enum
{
	MOTION_MOTOR_LEFT = 0,
	MOTION_MOTOR_RIGHT,
	MOTION_MOTOR_MAX,
};

class MotorInterface
{
	public:
		virtual void update(portTickType absTimeout)
		{
			(void) absTimeout;
		}

		virtual void set_speed(float v)
		{
			(void) v;
		}

		virtual void enable(bool enable)
		{
			(void) enable;
		}

		virtual bool is_in_motion()
		{
			return false;
		}

		virtual bool is_op_enable()
		{
			return true;
		}

		virtual void set_position(float pos)
		{
			(void) pos;
		}

		virtual void set_max_current(float val)
		{
			(void) val;
		}

		const char* name;
		float inputGain;    //!< gain pour convertir la vitesse en unites moteurs (rpm ou V)
		float current;
};

#endif
